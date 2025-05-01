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
  for (int i = 30; i <= 80; i++) {
    X.write(i);
    Y.write(i);
    delay(50);
  }
  for (int i = 80; i >= 30; i--) {
    X.write(i);
    Y.write(i);
    delay(50);
  }

}
