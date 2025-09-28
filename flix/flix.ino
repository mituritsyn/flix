// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Main firmware file

#include "vector.h"
#include "quaternion.h"
#include "util.h"
#include <esp_now.h>
#include <WiFi.h>

#define SERIAL_BAUDRATE 115200
#define WIFI_ENABLED 0
#define PIN_NEOPIXEL 38

double t = NAN; // current step time, s
float dt; // time delta from previous step, s
float controlRoll, controlPitch, controlYaw, controlThrottle; // pilot's inputs, range [-1, 1]
float controlMode = NAN;
Vector gyro; // gyroscope data
Vector acc; // accelerometer data, m/s/s
Vector rates; // filtered angular rates, rad/s
Quaternion attitude; // estimated attitude
bool landed; // are we landed and stationary
float motors[4]; // normalized motors thrust in range [0..1]

void setup() {
	Serial.begin(SERIAL_BAUDRATE);
	neopixelWrite(PIN_NEOPIXEL, 0, 255, 0);  // Red
	print("Initializing flix\n");
	disableBrownOut();
	setupParameters();
	setupLED();
	setupMotors();
	setLED(true);
#if WIFI_ENABLED
	setupWiFi();
#else
	setupEspNowRX();
#endif
	setupIMU();
	// setupRC();
	setLED(false);
	print("Initializing complete\n");
}

void loop() {
	readIMU();
	step();
	readRC();
	estimate();
	control();
	sendMotors();
	handleInput();
#if WIFI_ENABLED
	processMavlink();
#else
	readEspNowRX();
#endif
	logData();
	syncParameters();
}
