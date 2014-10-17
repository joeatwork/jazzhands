package net.culturematic.glove;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Arrays;
import java.util.UUID;
import java.util.TooManyListenersException;

import gnu.io.CommPortIdentifier;
import gnu.io.PortInUseException;
import gnu.io.NoSuchPortException;
import gnu.io.UnsupportedCommOperationException;
import gnu.io.SerialPort;
import gnu.io.SerialPortEventListener;
import gnu.io.SerialPortEvent;

public class GloveSerialConnection {

    public GloveSerialConnection(final String portName)
	throws PortInUseException, NoSuchPortException, UnsupportedCommOperationException, IOException, TooManyListenersException {
	final CommPortIdentifier portId = CommPortIdentifier.getPortIdentifier(portName);
	final String connectName = UUID.randomUUID().toString();
	
	mMessage = null;

	mPort = (SerialPort) portId.open(connectName, CONNECT_TIMEOUT);
	mPort.setSerialPortParams(BAUD_RATE, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
	mListener = new GloveEventListener(mPort.getInputStream());

	mPort.addEventListener(mListener);
	mPort.notifyOnDataAvailable(true);

	mOutputStream = mPort.getOutputStream();
    }

    public byte[] message() {
	return mMessage;
    }

    public void write(final byte[] message) throws IOException {
	mOutputStream.write(message);
    }

    public void close() {
	mPort.removeEventListener();
	mPort.close();
    }

    private class GloveEventListener implements SerialPortEventListener {

	public GloveEventListener(final InputStream inStream) {
	    mInStream = inStream;
	    mReadOffset = 0;
	    mReadBuffer = new byte[MESSAGE_SIZE_BYTES];
	}

	@Override
	public void serialEvent(final SerialPortEvent event) {
	    if (event.getEventType() == SerialPortEvent.DATA_AVAILABLE) {
		try {
		    readFromInputStream();
		} catch (IOException e) {
		    e.printStackTrace();
		    mReadOffset = 0;
		}
	    }
	}

	private void readFromInputStream() throws IOException {
	    int totalBytesRead = 0;
	    int totalBytesSkipped = 0;
	    int totalMessagesSent = 0;
	    while (mInStream.available() > 0) {
		if (0 == mReadOffset) {
		    // Read stream one byte at a time until we reach an 'N', which is maybe the start of a message,
		    // to align the buffer with a discrete MESSAGE_SIZE_BYTES packet
		    final int maybeStart = mInStream.read();
		    if (maybeStart == CAPITAL_N_BYTE) {
			mReadBuffer[0] = CAPITAL_N_BYTE;
			mReadOffset = 1;
		    }
		    totalBytesSkipped += 1;
		} else {
		    // If We're already aligned in the buffer, slurp as fast as we can
		    int bufferSpace = mReadBuffer.length - mReadOffset;

		    final int newBytesRead = mInStream.read(mReadBuffer, mReadOffset, bufferSpace);
		    mReadOffset = mReadOffset + newBytesRead;
		    if (mReadBuffer.length == mReadOffset) {
			mMessage = Arrays.copyOf(mReadBuffer, mReadBuffer.length);
			mReadOffset = 0;
			totalMessagesSent++;
		    }

		    totalBytesRead += newBytesRead;
		}
	    }
	}

	private int mReadOffset;
	private final byte[] mReadBuffer;
	private final InputStream mInStream;
    }

    private volatile byte[] mMessage;
    private final GloveEventListener mListener;
    private final SerialPort mPort;
    private final OutputStream mOutputStream;

    private static final byte CAPITAL_N_BYTE = 0x4E;
    private static final int MESSAGE_SIZE_BYTES = 50;
    private static final int BAUD_RATE = 115200;
    private static final int CONNECT_TIMEOUT = 2000;
}
