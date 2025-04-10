#include <Servo.h>

Servo X;
Servo Y;

void setup() {
  X.attach(9);
  Y.attach(10);
  pinMode(6, OUTPUT);

}

void loop() {
  digitalWrite(6,HIGH);
  for (int i = 0; i <= 50; i++) {
    X.write(i);
    Y.write(i);
    delay(50);
  }
  for (int i = 50; i >= 0; i--) {
    X.write(i);
    Y.write(i);
    delay(50);
  }

}
