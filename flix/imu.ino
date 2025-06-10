// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Work with the IMU sensor

#include <ICM42688.h>
#include <SPI.h>

#include "lpf.h"
#include "vector.h"

#define ONE_G 9.80665
#define DEG2RAD 3.14159265358979323846264338327950288f/180.0f;
#define SPIDEV_HOST SPI3_HOST
// #define SPIDEV_MOSI_PIN GPIO_NUM_39
// #define SPIDEV_MISO_PIN GPIO_NUM_38
// #define SPIDEV_CLK_PIN 	GPIO_NUM_40
// #define SPIDEV_CS 		GPIO_NUM_41

#define SPIDEV_MOSI_PIN GPIO_NUM_37
#define SPIDEV_MISO_PIN GPIO_NUM_36
#define SPIDEV_CLK_PIN GPIO_NUM_38
#define SPIDEV_CS GPIO_NUM_39

#define SPI_HS_CLOCK 24 * 1000 * 1000  // 24 МГц
#define SPI_LS_CLOCK 1000000           // 1 МГц

Vector accBias;
Vector gyroBias;
Vector accScale(1, 1, 1);

static const uint8_t CS_PIN = SPIDEV_CS;
static const uint8_t MOSI_PIN = SPIDEV_MOSI_PIN;
static const uint8_t MISO_PIN = SPIDEV_MISO_PIN;
static const uint8_t SCLK_PIN = SPIDEV_CLK_PIN;
// static SPIClass spi = SPIClass(HSPI);

ICM42688 IMU(SPI, CS_PIN, SPI_HS_CLOCK);

void setupIMU() {
  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  Serial.println("Setup IMU");
  bool status = IMU.begin();
  if (status < 0) {
    while (true) {
      Serial.println("IMU begin error");
      delay(1000);
    }
  }
  // 	gyro bias: -0.616379, -0.257088, 0.424461
  // accel bias: -0.001576, 0.000501, 0.038439
  // accel scale: 0.999742, 1.000630, 1.001210

  IMU.setAccelCalX(-0.001576, 0.999742);
  IMU.setAccelCalY(0.000501, 1.000630);
  IMU.setAccelCalZ(0.038439, 1.001210);
  Serial.println("IMU started");
  Serial.println("setup IMU filters");
  if (IMU.setGyroLPF() != 1) {
    Serial.println("gyro lpf error");
  }
  IMU.calibrateGyro();
}

void configureIMU() {
  IMU.setGyroFS(IMU.dps2000);
  IMU.setAccelFS(IMU.gpm4);
  IMU.setGyroODR(IMU.odr1k);
  IMU.setAccelODR(IMU.odr1k);
  IMU.setFilters(true, true);
  IMU.disableDataReadyInterrupt();
}

void readIMU() {
  IMU.getAGT();

  // Преобразование гироскопа
  float original_gyrX = IMU.gyrX() * DEG2RAD;
  float original_gyrY = IMU.gyrY() * DEG2RAD;
  float original_gyrZ = IMU.gyrZ() * DEG2RAD;
  gyro.x = -original_gyrY;
  gyro.y = original_gyrX;
  gyro.z = original_gyrZ;

  // Преобразование акселерометра
  float original_accX = IMU.accX();
  float original_accY = IMU.accY();
  float original_accZ = IMU.accZ();

  acc.x = -original_accY * ONE_G;
  acc.y = original_accX * ONE_G;
  acc.z = original_accZ * ONE_G;
}

void calibrateGyro() {
  IMU.calibrateGyro();
  printIMUCal();
}

void calibrateAccel() {
  Serial.setTimeout(60000);
  Serial.print("Place level [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();
  Serial.print("Place nose up [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();
  Serial.print("Place nose down [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();
  Serial.print("Place on right side [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();
  Serial.print("Place on left side [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();
  Serial.print("Place upside down [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();

  printIMUCal();
}

void calibrateAccelOnce() { IMU.calibrateAccel(); }

void printIMUCal() {
  Serial.printf("gyro bias: %f, %f, %f\n", IMU.getGyroBiasX(),
                IMU.getGyroBiasY(), IMU.getGyroBiasZ());
  Serial.printf("accel bias: %f, %f, %f\n", IMU.getAccelBiasX_mss(),
                IMU.getAccelBiasY_mss(), IMU.getAccelBiasZ_mss());
  Serial.printf("accel scale: %f, %f, %f\n", IMU.getAccelScaleFactorX(),
                IMU.getAccelScaleFactorY(), IMU.getAccelScaleFactorZ());
}

void plotGyro() {
  while (true) {
    Serial.println(IMU.gyrX());
    delay(100);
  }
}

void printIMUInfo() { Serial.printf("who am I: 0x%02X\n", IMU.whoAmI()); }
