#include <Servo.h>

Servo payloadServo;

#define SERVO_PIN 9
#define LOCK_ANGLE 20
#define RELEASE_ANGLE 100

void setup() {
  payloadServo.attach(SERVO_PIN);
  payloadServo.write(LOCK_ANGLE);
}

void loop() {
  // Basic test cycle. Replace with receiver/Pixhawk AUX trigger later.
  delay(5000);
  payloadServo.write(RELEASE_ANGLE);
  delay(2000);
  payloadServo.write(LOCK_ANGLE);
  delay(5000);
}