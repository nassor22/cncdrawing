#include <Servo.h>
#include <math.h>

#define S1_IN1  4
#define S1_IN2  5
#define S1_IN3  6
#define S1_IN4  7

#define S2_IN1  10
#define S2_IN2  11
#define S2_IN3  12
#define S2_IN4  13

#define SERVO_PIN  9

#define STEPS_PER_REV  4096
#define STEP_DELAY_US  1200

const float DEG_PER_STEP = 360.0 / STEPS_PER_REV;

#define L1  140.0
#define L2  115.0

#define PAPER_X_OFFSET   -105.0
#define PAPER_Y_OFFSET    120.0

#define A4_WIDTH   210.0
#define A4_HEIGHT  297.0

#define CIRCLE_RADIUS   15.0
#define CIRCLE_SEGMENTS 72
#define CORNER_MARGIN   30.0

#define PEN_DOWN_ANGLE  60
#define PEN_UP_ANGLE    120

Servo penServo;

long currentStepsS1 = 0;
long currentStepsS2 = 0;

float currentAngle1 = 0.0;
float currentAngle2 = 0.0;

const uint8_t halfStepSeq[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

int phaseS1 = 0;
int phaseS2 = 0;

void setStepperPins(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                    const uint8_t phase[4]) {
  digitalWrite(in1, phase[0]);
  digitalWrite(in2, phase[1]);
  digitalWrite(in3, phase[2]);
  digitalWrite(in4, phase[3]);
}

void disableStepper1() {
  digitalWrite(S1_IN1, LOW); digitalWrite(S1_IN2, LOW);
  digitalWrite(S1_IN3, LOW); digitalWrite(S1_IN4, LOW);
}

void disableStepper2() {
  digitalWrite(S2_IN1, LOW); digitalWrite(S2_IN2, LOW);
  digitalWrite(S2_IN3, LOW); digitalWrite(S2_IN4, LOW);
}

void stepS1(int dir) {
  phaseS1 = (phaseS1 + dir + 8) % 8;
  setStepperPins(S1_IN1, S1_IN2, S1_IN3, S1_IN4, halfStepSeq[phaseS1]);
  delayMicroseconds(STEP_DELAY_US);
}

void stepS2(int dir) {
  phaseS2 = (phaseS2 + dir + 8) % 8;
  setStepperPins(S2_IN1, S2_IN2, S2_IN3, S2_IN4, halfStepSeq[phaseS2]);
  delayMicroseconds(STEP_DELAY_US);
}

void moveSteppers(long targetS1, long targetS2) {
  long deltaS1 = targetS1 - currentStepsS1;
  long deltaS2 = targetS2 - currentStepsS2;

  int dirS1 = (deltaS1 > 0) ? 1 : -1;
  int dirS2 = (deltaS2 > 0) ? 1 : -1;

  long absS1 = abs(deltaS1);
  long absS2 = abs(deltaS2);

  long maxSteps = max(absS1, absS2);
  if (maxSteps == 0) return;

  long errS1 = 0;
  long errS2 = 0;

  for (long i = 0; i < maxSteps; i++) {
    errS1 += absS1;
    errS2 += absS2;

    bool stepped = false;

    if (errS1 >= maxSteps) {
      errS1 -= maxSteps;
      stepS1(dirS1);
      currentStepsS1 += dirS1;
      stepped = true;
    }
    if (errS2 >= maxSteps) {
      errS2 -= maxSteps;
      stepS2(dirS2);
      currentStepsS2 += dirS2;
      stepped = true;
    }

    if (!stepped) {
      delayMicroseconds(STEP_DELAY_US);
    }
  }
}

bool inverseKinematics(float x, float y, float &angle1, float &angle2) {
  float distSq = x * x + y * y;
  float dist   = sqrt(distSq);

  if (dist > (L1 + L2) || dist < fabs(L1 - L2)) {
    return false;
  }

  float cosTheta2 = (distSq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  cosTheta2 = constrain(cosTheta2, -1.0, 1.0);
  float theta2 = acos(cosTheta2);

  float k1 = L1 + L2 * cos(theta2);
  float k2 = L2 * sin(theta2);
  float theta1 = atan2(y, x) - atan2(k2, k1);

  angle1 = theta1 * 180.0 / M_PI;
  angle2 = theta2 * 180.0 / M_PI;

  return true;
}

bool moveTo(float x, float y) {
  float targetAngle1, targetAngle2;

  if (!inverseKinematics(x, y, targetAngle1, targetAngle2)) {
    Serial.print("  !! Unreachable: (");
    Serial.print(x, 1); Serial.print(", "); Serial.print(y, 1);
    Serial.println(")");
    return false;
  }

  float dAngle1 = targetAngle1 - currentAngle1;
  float dAngle2 = targetAngle2 - currentAngle2;

  long targetStepsS1 = currentStepsS1 + (long)(dAngle1 / DEG_PER_STEP);
  long targetStepsS2 = currentStepsS2 + (long)(dAngle2 / DEG_PER_STEP);

  moveSteppers(targetStepsS1, targetStepsS2);

  currentAngle1 = targetAngle1;
  currentAngle2 = targetAngle2;

  return true;
}

void penUp() {
  penServo.write(PEN_UP_ANGLE);
  delay(300);
}

void penDown() {
  penServo.write(PEN_DOWN_ANGLE);
  delay(300);
}

void drawCircle(float cx, float cy, float radius) {
  Serial.print("Drawing circle at (");
  Serial.print(cx, 1); Serial.print(", "); Serial.print(cy, 1);
  Serial.println(")");

  float startX = cx + radius;
  float startY = cy;

  penUp();
  if (!moveTo(startX, startY)) return;
  penDown();

  for (int i = 1; i <= CIRCLE_SEGMENTS; i++) {
    float angle = (2.0 * M_PI * i) / CIRCLE_SEGMENTS;
    float px = cx + radius * cos(angle);
    float py = cy + radius * sin(angle);
    moveTo(px, py);
  }

  penUp();
  Serial.println("  Circle done.");
}

void homePosition() {

  currentAngle1 = 0.0;
  currentAngle2 = 0.0;
  currentStepsS1 = 0;
  currentStepsS2 = 0;

  float readyX = (L1 + L2) * 0.7;
  float readyY = 0;
  inverseKinematics(readyX, readyY, currentAngle1, currentAngle2);
}

void setup() {
  Serial.begin(115200);
  Serial.println("CNC Drawing Robot - Starting up");

  pinMode(S1_IN1, OUTPUT); pinMode(S1_IN2, OUTPUT);
  pinMode(S1_IN3, OUTPUT); pinMode(S1_IN4, OUTPUT);

  pinMode(S2_IN1, OUTPUT); pinMode(S2_IN2, OUTPUT);
  pinMode(S2_IN3, OUTPUT); pinMode(S2_IN4, OUTPUT);

  penServo.attach(SERVO_PIN);
  penUp();

  delay(1000);

  float cx = PAPER_X_OFFSET + A4_WIDTH  / 2.0;
  float cy = PAPER_Y_OFFSET + A4_HEIGHT / 2.0;

  Serial.println("=== Drawing 1 circle at A4 centre ===");
  Serial.print("Paper origin: ("); Serial.print(PAPER_X_OFFSET);
  Serial.print(", "); Serial.print(PAPER_Y_OFFSET); Serial.println(")");
  Serial.print("Centre target: ("); Serial.print(cx, 1);
  Serial.print(", "); Serial.print(cy, 1); Serial.println(")");
  Serial.print("Arm reach: "); Serial.print(L1 + L2); Serial.println(" mm");

  Serial.println("\n--- Circle: Centre ---");
  drawCircle(cx, cy, CIRCLE_RADIUS);

  Serial.println("\n=== Circle complete! ===");
  penUp();
  disableStepper1();
  disableStepper2();

  Serial.println("Motors disabled. Done.");
}

void loop() {

}
