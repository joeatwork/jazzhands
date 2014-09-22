#include <Arduino.h>
#include <SPI.h>

/* MPU6000 Registers and settings, from
   http://invensense.com/mems/gyro/documents/RM-MPU-6000A.pdf */

#define REG_USER_CTRL 106
#define REG_PWR_MGMT_1 107
#define REG_GYRO_CONFIG 27
#define REG_ACCEL_CONFIG 28
#define REG_ACCEL_XOUT_H 59
#define REG_ACCEL_XOUT_L 60
#define REG_ACCEL_YOUT_H 61
#define REG_ACCEL_YOUT_L 62
#define REG_ACCEL_ZOUT_H 63
#define REG_ACCEL_ZOUT_L 64
#define REG_GYRO_XOUT_H 67
#define REG_GYRO_XOUT_L 68
#define REG_GYRO_YOUT_H 69
#define REG_GYRO_YOUT_L 70
#define REG_GYRO_ZOUT_H 71
#define REG_GYRO_ZOUT_L 72

/* MPU6000 register values and magic bits */

/* For REG_USER_CTRL */
#define I2C_IF_DIS 16

/* For REG_PWR_MGMT_1 */
#define CLKSEL_INTERNAL 0
#define CLKSEL_PLLX 1

/* For REG_GYRO_CONFIG */
#define FS_SEL_2000DEG_PERSEC 0b00011000

/* For REG_ACCEL_CONFIG */
#define AFS_SEL_16G 0b00011000

/* Pins. Definitions from https://github.com/SebMadgwick/ArduIMU-Gloves */

#define PIN_MPU6000_CS 4
#define PIN_RED_LED 5
#define PIN_BLUE_LED 6

struct mpu6000_sensors {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
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

static inline int16_t read_int16_registers(int reg_high, int reg_low) {
  return
    (mpu6000_read_register(reg_high) << 8 & 0xFF00) |
    (mpu6000_read_register(reg_low) & 0xFF);
}

struct mpu6000_sensors mpu6000_read_sensors() {
  struct mpu6000_sensors ret;
  ret.accel_x = read_int16_registers(REG_ACCEL_XOUT_H, REG_ACCEL_XOUT_L);
  ret.accel_y = read_int16_registers(REG_ACCEL_YOUT_H, REG_ACCEL_YOUT_L);
  ret.accel_z = read_int16_registers(REG_ACCEL_ZOUT_H, REG_ACCEL_ZOUT_L);
  ret.gyro_x = read_int16_registers(REG_GYRO_XOUT_H, REG_GYRO_XOUT_L);
  ret.gyro_y = read_int16_registers(REG_GYRO_YOUT_H, REG_GYRO_YOUT_L);
  ret.gyro_z = read_int16_registers(REG_GYRO_ZOUT_H, REG_GYRO_ZOUT_L);
  return ret;
}

static void serial_write_sensors(struct mpu6000_sensors sensors) {
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

  Serial.flush();
}

void setup()
{
  mpu6000_init();
  Serial.begin(115200);
  pinMode(PIN_BLUE_LED, OUTPUT);
}

void loop()
{
  digitalWrite(PIN_BLUE_LED, HIGH);   // sets the LED on
  struct mpu6000_sensors sensors = mpu6000_read_sensors();
  serial_write_sensors(sensors);
  delay(1000);
  digitalWrite(PIN_BLUE_LED, LOW);    // sets the LED off
}
