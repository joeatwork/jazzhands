//-----------------------------------------------------------------------------
// Entaro ChucK Developer!
// This is a Chugin boilerplate, generated by chugerate!
//-----------------------------------------------------------------------------

// this should align with the correct versions of these ChucK files
#include "chuck_dl.h"
#include "chuck_def.h"

// This could be broken
#include "chuck_type.h"
#include "chuck_instr.h"

// general includes
#include <stdio.h>
#include <limits.h>
#include <pthread.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

// declaration of chugin constructor
CK_DLL_CTOR(glovechugin_ctor);
// declaration of chugin desctructor
CK_DLL_DTOR(glovechugin_dtor);

CK_DLL_MFUN(glovechugin_connect);
CK_DLL_MFUN(glovechugin_close);
CK_DLL_MFUN(glovechugin_startMessage);

CK_DLL_MFUN(glovechugin_messageNumber);
CK_DLL_MFUN(glovechugin_finger1);
CK_DLL_MFUN(glovechugin_finger2);
CK_DLL_MFUN(glovechugin_finger3);
CK_DLL_MFUN(glovechugin_finger4);
CK_DLL_MFUN(glovechugin_finger5);

CK_DLL_MFUN(glovechugin_accelX);
CK_DLL_MFUN(glovechugin_accelY);
CK_DLL_MFUN(glovechugin_accelZ);

CK_DLL_MFUN(glovechugin_gyroX);
CK_DLL_MFUN(glovechugin_gyroY);
CK_DLL_MFUN(glovechugin_gyroZ);

CK_DLL_MFUN(glovechugin_magX);
CK_DLL_MFUN(glovechugin_magY);
CK_DLL_MFUN(glovechugin_magZ);

CK_DLL_MFUN(glovechugin_quaternion0);
CK_DLL_MFUN(glovechugin_quaternion1);
CK_DLL_MFUN(glovechugin_quaternion2);
CK_DLL_MFUN(glovechugin_quaternion3);

// this is a special offset reserved for Chugin internal data
t_CKINT glovechugin_data_offset = 0;

#define MESSAGE_LENGTH 50

#define CHECKED_OP(op) {\
    int code = (op);\
    if (code) {\
        fprintf(stderr, "ERROR IN " #op "\n");\
	fprintf(stderr, "%s\n", strerror(code));\
        exit(1);\
    }\
}

// Doing this for now because Chuck_String->str seems to have
// crazy corrupt garbage in it...
const char * HARDCODED_DEVICE_PATHS[] = {
    "/dev/tty.usbserial-FTE3RR3T",
    "/dev/tty.RNBT-C9F7-RNI-SPP"
};

const size_t HARDCODED_DEVICE_MAX = 1;

void reader_thread(void *glove_chugin);

// class definition for internal Chugin data
// (note: this isn't strictly necessary, but serves as example
// of one recommended approach)
class GloveChugin
{
public:
    // constructor
    GloveChugin() {
	m_running = false;
	m_should_close = false;
	pthread_mutex_init(&m_mutex, NULL);

	m_front_buffer = m_a_buffer;
	m_back_buffer = m_b_buffer;
    }

    ~GloveChugin()
    {
	close();
	pthread_mutex_destroy(&m_mutex);
    }

    // Interface to reading thread

    const char * t_getPath() {
	return m_path;
    }

    unsigned char * t_getReadingBuffer() {
	return m_reading_buffer;
    }

    void t_sendReadingBuffer() {
	CHECKED_OP(pthread_mutex_lock(&m_mutex));

	if (checkMessage()) {
	    memcpy(m_back_buffer, m_reading_buffer, MESSAGE_LENGTH);
	    m_message_ready = TRUE;
	}

	CHECKED_OP(pthread_mutex_unlock(&m_mutex));
    }

    bool t_shouldClose() {
	bool ret;
	CHECKED_OP(pthread_mutex_lock(&m_mutex));

	ret = m_should_close;

	CHECKED_OP(pthread_mutex_unlock(&m_mutex));

	return ret;
    }

    // Interface to ChucK code

    void connect(const char * path) {
	if (m_running) {
	    fprintf(stderr, "Glove device is already open and running");
	    return;
	}

	m_path = path;

	if (int create_err = pthread_create(&m_reader_thread, NULL, (void *(*) (void *))reader_thread, this)) {
	    fprintf(stderr, "Can't create reader thread!");
	    fprintf(stderr, "%s", strerror(create_err));
	}
    }

    void close() {
	CHECKED_OP(pthread_mutex_lock(&m_mutex));

	m_should_close = true;

	CHECKED_OP(pthread_mutex_unlock(&m_mutex));

	CHECKED_OP(pthread_join(m_reader_thread, NULL));
	m_running = false; // ONLY AFTER join!
    }

    bool checkMessage() {
	if ('N' != m_reading_buffer[0]) {
	    // fprintf(stderr, "Message has bad header (No N)\n");
	    return FALSE;
	} else if ('H' != m_reading_buffer[4]) {
	    // fprintf(stderr, "Message has bad header (No H)\n");
	    return FALSE;
	} else if ('A' != m_reading_buffer[16]) {
	    // fprintf(stderr, "Message has bad header (No A)\n");
	    return FALSE;
	} else if ('G' != m_reading_buffer[24]) {
	    // fprintf(stderr, "Message has bad header (No G)\n");
	    return FALSE;
	} else if ('M' != m_reading_buffer[32]) {
	    // fprintf(stderr, "Message has bad header (No G)\n");
	    return FALSE;
	} else if ('Q' != m_reading_buffer[40]) {
	    // fprintf(stderr, "Message has bad header (No Q)\n");
	    return FALSE;
	}

	return TRUE;
    }

    // fill array with message
    int startMessage() {
	CHECKED_OP(pthread_mutex_lock(&m_mutex));

	// TODO- if the device goes away or goes screwy, this
	// will oscillate between the last two good messages,
	// or maybe an old message and crazy garbage if we've
	// only ever written one good buffer.
	unsigned char * tmp = m_back_buffer;
	m_back_buffer = m_front_buffer;
	m_front_buffer = tmp;
	int ret = m_message_ready ? 1 : 0;

	CHECKED_OP(pthread_mutex_unlock(&m_mutex));

	return ret;
    }
    
    int16_t getMessageInt(size_t offset) {
	if (m_message_ready) {
	    unsigned char high = m_front_buffer[offset];
	    unsigned char low = m_front_buffer[offset + 1];
	    return (high << 8 & 0xFF00) | (low & 0xFF);
	} else {
	    return 0;
	}
    }

private:

    bool m_running;
    bool m_message_ready;

    bool m_should_close;
    const char * m_path;

    pthread_t m_reader_thread;
    pthread_mutex_t m_mutex;

    unsigned char * m_front_buffer;
    unsigned char * m_back_buffer;

    unsigned char m_reading_buffer[MESSAGE_LENGTH];
    unsigned char m_a_buffer[MESSAGE_LENGTH];
    unsigned char m_b_buffer[MESSAGE_LENGTH];
};

void reader_thread(void *glove_chugin) {
    unsigned char scratch_buffer[MESSAGE_LENGTH];
    GloveChugin *bcdata = (GloveChugin *) glove_chugin;
    unsigned char *reading_buffer = bcdata->t_getReadingBuffer();

    const char *path_str = bcdata->t_getPath();

    fprintf(stderr, "Attempting to open %s\n", path_str);

    int fd = open(path_str, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
	fprintf(stderr, "Can't open path %s\n", path_str);
	return;
    }

    struct termios toptions;
    tcgetattr(fd, &toptions);
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);

    // 8 bits per character
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;

    toptions.c_cflag &= ~PARENB; // no parity
    toptions.c_cflag &= ~CSTOPB; // one stop bit

    toptions.c_cflag &= ~CRTSCTS; // No hardwre flow control
    toptions.c_cflag |= CREAD; // Enable read
    toptions.c_cflag |= CLOCAL; // ignore modem control lines

    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // No flow control
    toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // No echo, not in canonical mode

    toptions.c_oflag &= ~OPOST; // No output processing

    toptions.c_cc[VMIN] = 24; // Buffer at least 24 characters before returning from read
    toptions.c_cc[VTIME] = 1; // OR return from read after 0.1 second of silence

    // Drop everything that has happened, start fresh
    tcsetattr(fd, TCSAFLUSH, &toptions);

    size_t reading_buffer_offset = 0;
    while (!bcdata->t_shouldClose()) {
	ssize_t bytes_read = read(fd, scratch_buffer, MESSAGE_LENGTH);
	if (-1 == bytes_read) {
	    if (errno == EAGAIN) {
		usleep(10000); // TODO is this needed?
	    } else {
		perror("can't read from device");
		break;
	    }
	} else {
	    size_t scratch_buffer_offset = 0;
	    size_t read_length = bytes_read;

	    if (reading_buffer_offset == 0) {
		// Scan to the next 'N' and copy from there
		size_t i;
		for (i = 0; i < bytes_read && 'N' != scratch_buffer[i]; i++) {
		    ;
		}
		scratch_buffer_offset = i;
		read_length = read_length - i;

		assert(0 == reading_buffer_offset);
		if (i == bytes_read) {
		    assert(0 == read_length);
		} else {
		    assert('N' == scratch_buffer[i]);
		}
	    }

	    while (read_length > 0) {
		size_t copy_length = read_length;
		size_t message_space = MESSAGE_LENGTH - reading_buffer_offset;
		if (copy_length > message_space) {
		    copy_length = message_space;
		}

		memcpy(reading_buffer + reading_buffer_offset,
		       scratch_buffer + scratch_buffer_offset,
		       copy_length);

		read_length -= copy_length;
		scratch_buffer_offset += copy_length;
		reading_buffer_offset += copy_length;

		if (reading_buffer_offset == MESSAGE_LENGTH) {
		    bcdata->t_sendReadingBuffer();
		    reading_buffer_offset = 0;
		    read_length = 0;
		}
	    }
	}
    } // while not shouldclose

    fprintf(stderr, "Closing serial connection\n");

    ::close(fd);
}


// query function: chuck calls this when loading the Chugin
// NOTE: developer will need to modify this function to
// add additional functions to this Chugin
CK_DLL_QUERY( GloveChugin )
{
    // hmm, don't change this...
    QUERY->setname(QUERY, "GloveChugin");
    
    // begin the class definition
    // can change the second argument to extend a different ChucK class
    QUERY->begin_class(QUERY, "GloveChugin", "Object");

    // register the constructor (probably no need to change)
    QUERY->add_ctor(QUERY, glovechugin_ctor);
    // register the destructor (probably no need to change)
    QUERY->add_dtor(QUERY, glovechugin_dtor);

    QUERY->add_mfun(QUERY, glovechugin_connect, "void", "connect");
    QUERY->add_arg(QUERY, "int", "deviceIndex");

    QUERY->add_mfun(QUERY, glovechugin_close, "void", "close");

    QUERY->add_mfun(QUERY, glovechugin_startMessage, "int", "startMessage");

    QUERY->add_mfun(QUERY, glovechugin_messageNumber, "int", "messageNumber");

    QUERY->add_mfun(QUERY, glovechugin_finger1, "int", "finger1");
    QUERY->add_mfun(QUERY, glovechugin_finger2, "int", "finger2");
    QUERY->add_mfun(QUERY, glovechugin_finger3, "int", "finger3");
    QUERY->add_mfun(QUERY, glovechugin_finger4, "int", "finger4");
    QUERY->add_mfun(QUERY, glovechugin_finger5, "int", "finger5");

    QUERY->add_mfun(QUERY, glovechugin_accelX, "int", "accelX");
    QUERY->add_mfun(QUERY, glovechugin_accelY, "int", "accelY");
    QUERY->add_mfun(QUERY, glovechugin_accelZ, "int", "accelZ");

    QUERY->add_mfun(QUERY, glovechugin_gyroX, "int", "gyroX");
    QUERY->add_mfun(QUERY, glovechugin_gyroY, "int", "gyroY");
    QUERY->add_mfun(QUERY, glovechugin_gyroZ, "int", "gyroZ");

    QUERY->add_mfun(QUERY, glovechugin_magX, "int", "magX");
    QUERY->add_mfun(QUERY, glovechugin_magY, "int", "magY");
    QUERY->add_mfun(QUERY, glovechugin_magZ, "int", "magZ");

    QUERY->add_mfun(QUERY, glovechugin_quaternion0, "int", "quaternion0");
    QUERY->add_mfun(QUERY, glovechugin_quaternion1, "int", "quaternion1");
    QUERY->add_mfun(QUERY, glovechugin_quaternion2, "int", "quaternion2");
    QUERY->add_mfun(QUERY, glovechugin_quaternion3, "int", "quaternion3");
    
    // this reserves a variable in the ChucK internal class to store 
    // referene to the c++ class we defined above
    glovechugin_data_offset = QUERY->add_mvar(QUERY, "int", "@gc_data", false);

    // end the class definition
    // IMPORTANT: this MUST be called!
    QUERY->end_class(QUERY);

    // wasn't that a breeze?
    return TRUE;
}

// implementation for the constructor
CK_DLL_CTOR(glovechugin_ctor)
{
    // get the offset where we'll store our internal c++ class pointer
    OBJ_MEMBER_INT(SELF, glovechugin_data_offset) = 0;
    
    // instantiate our internal c++ class representation
    GloveChugin * bcdata = new GloveChugin();
    
    // store the pointer in the ChucK object member
    OBJ_MEMBER_INT(SELF, glovechugin_data_offset) = (t_CKINT) bcdata;
}

// implementation for the destructor
CK_DLL_DTOR(glovechugin_dtor)
{
    // get our c++ class pointer
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    // check it
    if( bcdata )
    {
        // clean up
        delete bcdata;
        OBJ_MEMBER_INT(SELF, glovechugin_data_offset) = 0;
        bcdata = NULL;
    }
}

CK_DLL_MFUN(glovechugin_connect)
{
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);

    // TODO- why does deviceName->str point to garbage?
    int deviceIndex = GET_NEXT_INT(ARGS);
    if (deviceIndex < 0 || HARDCODED_DEVICE_MAX < deviceIndex) {
	fprintf(stderr, "Can't connect to device index %d. Known devices are:\n", deviceIndex);
	for (size_t i = 0; i < HARDCODED_DEVICE_MAX; i++) {
	    fprintf(stderr, "   %lu: %s\n", i, HARDCODED_DEVICE_PATHS[i]);
	}
	return;
    }

    const char * deviceName = HARDCODED_DEVICE_PATHS[deviceIndex];
    printf("Got input device name %s\n", deviceName);
    bcdata->connect(deviceName);
}

CK_DLL_MFUN(glovechugin_close)
{
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    bcdata->close();
}

CK_DLL_MFUN(glovechugin_startMessage)
{
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->startMessage();
}

CK_DLL_MFUN(glovechugin_messageNumber)
{
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(1);
}

CK_DLL_MFUN(glovechugin_finger1) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(5);
}

CK_DLL_MFUN(glovechugin_finger2) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(7);
}

CK_DLL_MFUN(glovechugin_finger3) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(9);
}

CK_DLL_MFUN(glovechugin_finger4) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(11);
}

CK_DLL_MFUN(glovechugin_finger5) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(13);
}

CK_DLL_MFUN(glovechugin_accelX) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(17);
}

CK_DLL_MFUN(glovechugin_accelY) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(19);
}

CK_DLL_MFUN(glovechugin_accelZ) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(21);
}

CK_DLL_MFUN(glovechugin_gyroX) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(25);
}

CK_DLL_MFUN(glovechugin_gyroY) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(27);
}

CK_DLL_MFUN(glovechugin_gyroZ) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(29);
}

CK_DLL_MFUN(glovechugin_magX) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(33);
}

CK_DLL_MFUN(glovechugin_magY) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(35);
}

CK_DLL_MFUN(glovechugin_magZ) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(37);
}

CK_DLL_MFUN(glovechugin_quaternion0) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(41);
}

CK_DLL_MFUN(glovechugin_quaternion1) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(43);
}

CK_DLL_MFUN(glovechugin_quaternion2) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(45);
}

CK_DLL_MFUN(glovechugin_quaternion3) {
    GloveChugin * bcdata = (GloveChugin *) OBJ_MEMBER_INT(SELF, glovechugin_data_offset);
    RETURN->v_int = bcdata->getMessageInt(47);
}
