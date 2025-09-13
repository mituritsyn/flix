// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Motors output control using MOSFETs
// In case of using ESCs, change PWM_STOP, PWM_MIN and PWM_MAX to appropriate values in μs, decrease PWM_FREQUENCY (to 400)

#define MOTOR_COUNT 4
#define PWM_RESOLUTION 12
#define PWM_FREQUENCY (80000000 / (1UL << PWM_RESOLUTION))
#define MAX_DUTY (1 << PWM_RESOLUTION) - 1

const int MOTOR_PINS[] = {1, 5, 14, 18}; 
// Motors array indexes:
const int MOTOR_REAR_LEFT = 0;
const int MOTOR_REAR_RIGHT = 1;
const int MOTOR_FRONT_RIGHT = 2;
const int MOTOR_FRONT_LEFT = 3;

void setupMotors() {
	Serial.print("Setup Motors\n");
	for (int i = 0; i < MOTOR_COUNT; i++) {
		ledcAttach(MOTOR_PINS[i], PWM_FREQUENCY, PWM_RESOLUTION);
	}
	sendMotors();
	Serial.print("Motors initialized\n");
}

int getDutyCycle(float value) {
	value = constrain(value, 0.0f, 1.0f);
    return round(value * MAX_DUTY);
}

void sendMotors() {
	for (int i = 0; i < MOTOR_COUNT; i++) {
		ledcWrite(MOTOR_PINS[i], getDutyCycle(motors[i]));
	}
}

bool motorsActive() {
	for (int i = 0; i < MOTOR_COUNT; i++) {
		if (motors[i] != 0) return true;
	}
	return false;
}

void testMotor(int n) {
	Serial.printf("Testing motor %d\n", n);
	motors[n] = 1;
	delay(50); // ESP32 may need to wait until the end of the current cycle to change duty https://github.com/espressif/arduino-esp32/issues/5306
	sendMotors();
	pause(3);
	motors[n] = 0;
	sendMotors();
	Serial.print("Done\n");
}

