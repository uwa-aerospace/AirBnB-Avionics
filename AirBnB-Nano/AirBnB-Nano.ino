#include <Servo.h>

Servo servo_1;

int servo_pin = 3; // PWM pin for servo control
int pos = 0;    // servo starting position

void setup() {
  servo_1.attach(servo_pin); // start servo control
  pinMode(7, INPUT);
}

void loop() {
  if (digitalRead(7) == HIGH) {
    servo_1.write(20);
  } else {
    servo_1.write(90);
  }
}
