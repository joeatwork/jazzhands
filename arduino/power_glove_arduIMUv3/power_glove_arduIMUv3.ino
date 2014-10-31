#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "MadgwickAHRS.h"

/* This file is essentially a deep reading of

   https://github.com/SebMadgwick/ArduIMU-Gloves

   with lots of shiny new bugs. Readers and potential users
   are strongly encouraged to start with that code rather
   than this. */

static const float RADIANS_PER_DEGREE = 0.0174532925f;

/* MPU6000 Registers and settings, from
   http://invensense.com/mems/gyro/documents/RM-MPU-6000A.pdf */

static const int REG_USER_CTRL = 106;
static const int REG_PWR_MGMT_1 = 107;
static const int REG_GYRO_CONFIG = 27;
static const int REG_ACCEL_CONFIG = 28;
static const int REG_ACCEL_XOUT_H = 59;
static const int REG_ACCEL_XOUT_L = 60;
static const int REG_ACCEL_YOUT_H = 61;
static const int REG_ACCEL_YOUT_L = 62;
static const int REG_ACCEL_ZOUT_H = 63;
static const int REG_ACCEL_ZOUT_L = 64;
static const int REG_GYRO_XOUT_H = 67;
static const int REG_GYRO_XOUT_L = 68;
static const int REG_GYRO_YOUT_H = 69;
static const int REG_GYRO_YOUT_L = 70;
static const int REG_GYRO_ZOUT_H = 71;
static const int REG_GYRO_ZOUT_L = 72;

/* MPU6000 register values and magic bits */

/* For REG_USER_CTRL */
static const int I2C_IF_DIS = 16;

/* For REG_PWR_MGMT_1 */
static const int CLKSEL_INTERNAL = 0;
static const int CLKSEL_PLLX = 1;

/* For REG_GYRO_CONFIG */
static const int FS_SEL_2000DEG_PERSEC = 0b00011000;

/* For REG_ACCEL_CONFIG */
static const int AFS_SEL_16G = 0b00011000;

/* Pins. Definitions from https://github.com/SebMadgwick/ArduIMU-Gloves */

static const int PIN_MPU6000_CS = 4;
static const int PIN_RED_LED = 5;
static const int PIN_BLUE_LED = 6;

struct sensors {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
  int16_t flex_00; // TODO RENAME TO FINGER?
  int16_t flex_01;
  int16_t flex_02;
  int16_t flex_03;
  int16_t flex_04;
};

enum request_type_t {
  REQUEST_TYPE_NONE,
  REQUEST_TYPE_LED
};

struct client_request {
  request_type_t type;
  byte r;
  byte g;
  byte b;
};

static struct client_request serial_read_request() {
  struct client_request ret;
  ret.type = REQUEST_TYPE_NONE;

  // ALL INBOUND MESSAGES are five bytes long
  while (5 <= Serial.available()) {
    byte header = Serial.read();
    switch (header) {
    case 'L': {
      ret.type = REQUEST_TYPE_LED;
      ret.r = Serial.read();
      ret.g = Serial.read();
      ret.b = Serial.read();
      Serial.read(); // Drop newline on the floor
    }; break;
    default:
      ret.type = REQUEST_TYPE_NONE; // !! Out of sync! Skip on ahead
    }
  }

  return ret;
}

static void mpu6000_write_register(const char addr, const char data) {
  digitalWrite(PIN_MPU6000_CS, LOW);
  SPI.transfer(addr);
  SPI.transfer(data);
  digitalWrite(PIN_MPU6000_CS, HIGH);  
}

static char mpu6000_read_register(const char addr) {
  char ret;
  digitalWrite(PIN_MPU6000_CS, LOW);
  SPI.transfer(addr | 0x80);
  ret = SPI.transfer(0);
  digitalWrite(PIN_MPU6000_CS, HIGH);
  
  return ret;
}

void mpu6000_init() {
  pinMode(PIN_MPU6000_CS, OUTPUT);
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  delay(10); // ???
  
  /* Set bit to use SPI instead of I2C. I2C If Disabled = SPI If Enabled */
  mpu6000_write_register(REG_USER_CTRL, I2C_IF_DIS);
  
  /* Set value to use internal 8MHZ Clock. Sample code
     Uses "PLL With X Axis Gyroscope Reference", which I don't quite grok. */
  mpu6000_write_register(REG_PWR_MGMT_1, CLKSEL_INTERNAL);
  
  /* Set Gyroscope range to += 2000 deg/sec */
  mpu6000_write_register(REG_GYRO_CONFIG, FS_SEL_2000DEG_PERSEC);
  
  /* Set Accelerometer range to Plus or Minus 16 G range */
  mpu6000_write_register(REG_ACCEL_CONFIG, AFS_SEL_16G);
}

static inline int16_t from_bytes(char high_byte, char low_byte) {
  return
    (high_byte << 8 & 0xFF00) |
    (low_byte & 0xFF);
}

static inline int16_t mpu6000_read_register_pair(int reg_high, int reg_low) {
  char high_byte = mpu6000_read_register(reg_high);
  char low_byte = mpu6000_read_register(reg_low);
  return from_bytes(high_byte, low_byte);
}

void mpu6000_read_sensors(struct sensors *const out) {
  out->accel_x = mpu6000_read_register_pair(REG_ACCEL_XOUT_H, REG_ACCEL_XOUT_L);
  out->accel_y = mpu6000_read_register_pair(REG_ACCEL_YOUT_H, REG_ACCEL_YOUT_L);
  out->accel_z = mpu6000_read_register_pair(REG_ACCEL_ZOUT_H, REG_ACCEL_ZOUT_L);
  out->gyro_x = mpu6000_read_register_pair(REG_GYRO_XOUT_H, REG_GYRO_XOUT_L);
  out->gyro_y = mpu6000_read_register_pair(REG_GYRO_YOUT_H, REG_GYRO_YOUT_L);
  out->gyro_z = mpu6000_read_register_pair(REG_GYRO_ZOUT_H, REG_GYRO_ZOUT_L);
}

/* hmc5883 stuff from
   https://github.com/SebMadgwick/ArduIMU-Gloves/blob/master/ArduIMU_Glove/I2CBus.cpp */

static const int HMC_ADDR = 0x1E;
static const int CONFIG_REG_A = 0x00;
static const int RATE_75HZ = 0b00011000;
static const int GAIN_4P7G = 0b10100000;
static const int MODE_CONT = 0b00000000;
static const int DOUT_X_MSB = 0x03;

void hmc5883_init() {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(CONFIG_REG_A);
  Wire.write(RATE_75HZ);
  Wire.write(GAIN_4P7G);
  Wire.write(MODE_CONT); // write to this register else HMC5883 initialisation may fail
  Wire.endTransmission();
}

void hmc5883_read_sensors(struct sensors *const sensors) {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(DOUT_X_MSB);
  Wire.endTransmission();
  Wire.requestFrom(HMC_ADDR, 6);
  char x_high = Wire.read();
  char x_low = Wire.read();
  char z_high = Wire.read();
  char z_low = Wire.read();
  char y_high = Wire.read();
  char y_low = Wire.read();
  sensors->mag_x = from_bytes(x_high, x_low);
  sensors->mag_y = from_bytes(y_high, y_low);
  sensors->mag_z = from_bytes(z_high, z_low);
  Wire.endTransmission();
}

void flex_sensors_init() {
  // Enable internal pull ups as for simple voltage dividers
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
}

void flex_read_sensors(struct sensors *const out) {
  out->flex_00 = analogRead(A0);
  out->flex_01 = analogRead(A1);
  out->flex_02 = analogRead(A2);
  out->flex_03 = analogRead(A3);
  out->flex_04 = analogRead(A4);
}

// MinM docs, including default addr and command language
// at http://thingm.com/fileadmin/thingm/downloads/BlinkM_datasheet.pdf
static const int MINM_ADDR = 0x09;

void led_set_rgb(byte r, byte g, byte b) {
  Wire.beginTransmission(MINM_ADDR);
  Wire.write('n'); // set RGB
  Wire.write(r);
  Wire.write(g);
  Wire.write(b);
  Wire.endTransmission();
}

void led_init() {
  Wire.beginTransmission(MINM_ADDR);
  Wire.write('o'); // stop script
  Wire.endTransmission();
  led_set_rgb(0, 0, 0);
}

static void serial_write_int16(int16_t val) {
  byte high = (val >> 8) & 0xFF;
  byte low = val & 0xFF;
  Serial.write(high);
  Serial.write(low);
}

// Write 4 bytes
static void serial_write_number(int16_t number) {
  Serial.write('N'); // 0
  serial_write_int16(number); // 1
  Serial.write('\n'); // 3

  // 4 bytes of serial number
}

// Writes 36 bytes to serial port
static void serial_write_sensors(struct sensors sensors) {
  Serial.write('H'); // 0
  serial_write_int16(sensors.flex_03); // 1 Thumb
  serial_write_int16(sensors.flex_01); // 3 Index
  serial_write_int16(sensors.flex_02); // 5 Middle
  serial_write_int16(sensors.flex_00); // 7 Ring
  serial_write_int16(sensors.flex_04); // 9 Pinky
  Serial.write('\n'); // 11

  // 12 bytes of Finger data

  Serial.write('A'); // 0
  serial_write_int16(sensors.accel_x); // 1 x
  serial_write_int16(sensors.accel_y); // 3 y
  serial_write_int16(sensors.accel_z); // 5 z
  Serial.write('\n'); // 7

  // 8 bytes of Accelerometer data

  Serial.write('G');
  serial_write_int16(sensors.gyro_x); // x
  serial_write_int16(sensors.gyro_y); // y
  serial_write_int16(sensors.gyro_z); // x
  Serial.write('\n');

  // 8 bytes of gyro data

  Serial.write('M');
  serial_write_int16(sensors.mag_x); // x
  serial_write_int16(sensors.mag_y); // y
  serial_write_int16(sensors.mag_z); // z
  Serial.write('\n');
  Serial.flush();

  // 8 bytes of compass data
}

static const float INT16_SCALE = 32767;

// Writes 10 bytes to serial port
static void serial_write_quaternion(struct MadgwickState state) {
  int16_t scaled_q0 = (int16_t) (state.q0 * INT16_SCALE);
  int16_t scaled_q1 = (int16_t) (state.q1 * INT16_SCALE);
  int16_t scaled_q2 = (int16_t) (state.q2 * INT16_SCALE);
  int16_t scaled_q3 = (int16_t) (state.q3 * INT16_SCALE);

  Serial.write('Q'); // 0
  serial_write_int16(scaled_q0); // 1
  serial_write_int16(scaled_q1); // 3
  serial_write_int16(scaled_q2); // 5
  serial_write_int16(scaled_q3); // 7
  Serial.write('\n'); // 9

  // 10 bytes total
}

static const float GYRO_SENSITIVITY = 25.0f/41.0f;
static const float ACCEL_SENSITIVITY = 125.0f/256.0f;

void madgwick_update(struct MadgwickState *state, struct sensors *sensors) {
  static unsigned long previous_update_time = 0;
  unsigned long current_time = millis();
  if (0 == previous_update_time) {
    previous_update_time = current_time;
    return;
  }

  float gxRad = ((float) sensors->gyro_x) * RADIANS_PER_DEGREE * GYRO_SENSITIVITY;
  float gyRad = ((float) sensors->gyro_y) * RADIANS_PER_DEGREE * GYRO_SENSITIVITY;
  float gzRad = ((float) sensors->gyro_z) * RADIANS_PER_DEGREE * GYRO_SENSITIVITY;

  float accelX = sensors->accel_x * ACCEL_SENSITIVITY;
  float accelY = sensors->accel_y * ACCEL_SENSITIVITY;
  float accelZ = sensors->accel_z * ACCEL_SENSITIVITY;

  unsigned long delta_time = current_time - previous_update_time;
  state->sampleFreq = 1000.0f / delta_time;
  MadgwickAHRSupdate(state,
		     gxRad, gyRad, gzRad,
		     accelX, accelY, accelZ,
		     sensors->mag_x, sensors->mag_y, sensors->mag_z);
}

void setup() {
  Wire.begin();
  delay(10);
  mpu6000_init();
  hmc5883_init();
  flex_sensors_init();
  led_init();

  Serial.begin(115200);
}

static const unsigned int SEND_RATE_MILLIS = 10;
void loop() {
  static unsigned long last_send_time = 0;
  static int16_t sequence_number = 1;
  static struct MadgwickState state = {
    100, // sampleFreq
    0.1f, // beta
    1.0f, // q0
    0.0f, // q1
    0.0f, // q2
    0.0f  // q3
  };

  struct sensors sensors;

  client_request request = serial_read_request();
  switch (request.type) {
    break;
  case REQUEST_TYPE_LED:
    led_set_rgb(request.r, request.g, request.b);
    break;
  case REQUEST_TYPE_NONE:
    break;
  default:
    ; // !! ERROR! WTF!
  }

  mpu6000_read_sensors(&sensors);
  hmc5883_read_sensors(&sensors);
  madgwick_update(&state, &sensors);
  flex_read_sensors(&sensors);

  unsigned long current_time = millis();
  if (SEND_RATE_MILLIS < current_time - last_send_time) {
    serial_write_number(sequence_number); // 4 bytes
    serial_write_sensors(sensors); // 36 bytes
    serial_write_quaternion(state); // 10 bytes
    // total write of 50 bytes

    sequence_number++;
    last_send_time = current_time;
  }
}
