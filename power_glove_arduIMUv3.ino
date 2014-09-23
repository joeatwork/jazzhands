#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

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
};

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

static void serial_write_sensors(struct sensors sensors) {
  Serial.print("AX:");
  Serial.print(sensors.accel_x);
  Serial.print(",");

  Serial.print("AY:");
  Serial.print(sensors.accel_y);
  Serial.print(",");

  Serial.print("AZ:");
  Serial.print(sensors.accel_z);
  Serial.print(",");

  Serial.print("GX:");
  Serial.print(sensors.gyro_x);
  Serial.print(",");

  Serial.print("GY:");
  Serial.print(sensors.gyro_y);
  Serial.print(",");

  Serial.print("GZ:");
  Serial.print(sensors.gyro_z);
  Serial.print(",");
  Serial.print("\n\r");

  Serial.print("MX:");
  Serial.print(sensors.mag_x);
  Serial.print("MY:");
  Serial.print(sensors.mag_y);
  Serial.print("MZ:");
  Serial.print(sensors.mag_z);

  Serial.flush();
}

void setup()
{
  Wire.begin();
  delay(10);
  mpu6000_init();
  hmc5883_init();
  Serial.begin(115200);
  pinMode(PIN_BLUE_LED, OUTPUT);
}

void loop()
{
  struct sensors sensors;
  digitalWrite(PIN_BLUE_LED, HIGH);   // sets the LED on
  mpu6000_read_sensors(&sensors);
  hmc5883_read_sensors(&sensors);
  serial_write_sensors(sensors);
  delay(1000);
  digitalWrite(PIN_BLUE_LED, LOW);    // sets the LED off
}
