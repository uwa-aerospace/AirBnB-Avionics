#include <Servo.h>

Servo servo_1;

int servo_pin = 3; // PWM pin for servo control

void setup() {
  Serial.begin(9600);
  servo_1.attach(servo_pin); // start servo control
  pinMode(7, INPUT);
  servo_1.write(90);
  delay(1000);
  servo_1.write(20);
  delay(1000);
  servo_1.write(90);
  delay(1000);
  Serial.println("Setup complete, starting...");
}

void loop() {
  if (digitalRead(7) == HIGH) {
    servo_1.write(20);
  } else {
//    servo_1.write(90);
  }
}
