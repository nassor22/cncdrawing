#include <Servo.h>

#define SERVO_PIN 9

Servo myServo;

void setup() {
  Serial.begin(115200);
  Serial.println("Servo Test - Sweeping 0 to 180");
  myServo.attach(SERVO_PIN);
}

void loop() {

  for (int angle = 0; angle <= 180; angle += 5) {
    myServo.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(100);
  }

  for (int angle = 180; angle >= 0; angle -= 5) {
    myServo.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(100);
  }
}
