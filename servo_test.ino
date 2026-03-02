/*
 * Servo Motor Test - Arduino Mega 2560
 * Signal pin: Digital 9
 *
 * Sweeps the servo back and forth between 0° and 180°.
 * Open Serial Monitor at 115200 to see the current angle.
 */

#include <Servo.h>

#define SERVO_PIN 9

Servo myServo;

void setup() {
  Serial.begin(115200);
  Serial.println("Servo Test - Sweeping 0 to 180");
  myServo.attach(SERVO_PIN);
}

void loop() {
  // Sweep from 0 to 180
  for (int angle = 0; angle <= 180; angle += 5) {
    myServo.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(100);
  }

  // Sweep from 180 to 0
  for (int angle = 180; angle >= 0; angle -= 5) {
    myServo.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(100);
  }
}
