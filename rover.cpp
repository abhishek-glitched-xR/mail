#include <Arduino.h>
const uint8_t LEFT_IN1 = 5;
const uint8_t LEFT_IN2 = 6;
const uint8_t RIGHT_IN1 = 9;
const uint8_t RIGHT_IN2 = 10;
const uint8_t TRIG_PIN = 2;
const uint8_t ECHO_PIN = 3;
const int MOTOR_SPEED = 180;
const int TURN_SPEED = 160;
const int SAFE_DISTANCE_CM = 30;
char command = 'S';
void forward();
void backward();
void left();
void right();
void stopRover();
int getDistanceCm();
void setup() {
  Serial.begin(9600);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  stopRover();
}
void loop() {
  if (Serial.available() > 0) {
    command = Serial.read();
  }
  int distance = getDistanceCm();
  if (distance > 0 && distance < SAFE_DISTANCE_CM) {
    stopRover();
    return;
  }
  if (command == 'F') {
    forward();
  } else if (command == 'B') {
    backward();
  } else if (command == 'L') {
    left();
  } else if (command == 'R') {
    right();
  } else {
    stopRover();
  }
}
void forward() {
  analogWrite(LEFT_IN1, MOTOR_SPEED);
  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, MOTOR_SPEED);
  analogWrite(RIGHT_IN2, 0);
}
void backward() {
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, MOTOR_SPEED);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, MOTOR_SPEED);
}
void left() {
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, TURN_SPEED);
  analogWrite(RIGHT_IN1, TURN_SPEED);
  analogWrite(RIGHT_IN2, 0);
}
void right() {
  analogWrite(LEFT_IN1, TURN_SPEED);
  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, TURN_SPEED);
}
void stopRover() {
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, 0);
}
int getDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);