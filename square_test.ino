/*
 * Stepper Motor Test - Draw a Circle with 2-Link Arm
 * Arduino Mega 2560
 *
 * This is a ROTARY 2-link arm, NOT a Cartesian gantry.
 * Each stepper rotates a joint, so we must use inverse kinematics
 * to convert Cartesian (x,y) targets into joint angles.
 *
 * Stepper 1 (Base joint):  pins 4,5,6,7   (IN1-IN4)
 * Stepper 2 (Elbow joint): pins 10,11,12,13 (IN1-IN4)
 *
 * Arm: L1 = 140mm, L2 = 115mm
 */

#include <math.h>

// ── Pin definitions ──
#define S1_IN1  4
#define S1_IN2  5
#define S1_IN3  6
#define S1_IN4  7

#define S2_IN1  10
#define S2_IN2  11
#define S2_IN3  12
#define S2_IN4  13

// ── Motor parameters ──
#define STEPS_PER_REV  4096       // 28BYJ-48 half-step
#define STEP_DELAY_US  1200       // speed (lower = faster, min ~900)

const float DEG_PER_STEP = 360.0 / STEPS_PER_REV;

// ── Arm geometry (mm) ──
#define L1  140.0    // base to elbow
#define L2  115.0    // elbow to pen

// ── Circle parameters ──
#define CIRCLE_RADIUS    20.0   // mm
#define CIRCLE_SEGMENTS  72     // points per circle (5° per segment)

// ── Global state ──
long currentStepsS1 = 0;
long currentStepsS2 = 0;
float currentAngle1 = 0.0;  // degrees
float currentAngle2 = 0.0;  // degrees
int phaseS1 = 0;
int phaseS2 = 0;

// Half-step sequence (8 phases)
const uint8_t seq[8][4] = {
  {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
  {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};

// ── Low-level stepping ──
void setPhase(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, const uint8_t ph[4]) {
  digitalWrite(p1, ph[0]); digitalWrite(p2, ph[1]);
  digitalWrite(p3, ph[2]); digitalWrite(p4, ph[3]);
}

void stepS1(int dir) {
  phaseS1 = (phaseS1 + dir + 8) % 8;
  setPhase(S1_IN1, S1_IN2, S1_IN3, S1_IN4, seq[phaseS1]);
  delayMicroseconds(STEP_DELAY_US);
}

void stepS2(int dir) {
  phaseS2 = (phaseS2 + dir + 8) % 8;
  setPhase(S2_IN1, S2_IN2, S2_IN3, S2_IN4, seq[phaseS2]);
  delayMicroseconds(STEP_DELAY_US);
}

void disableMotors() {
  digitalWrite(S1_IN1, LOW); digitalWrite(S1_IN2, LOW);
  digitalWrite(S1_IN3, LOW); digitalWrite(S1_IN4, LOW);
  digitalWrite(S2_IN1, LOW); digitalWrite(S2_IN2, LOW);
  digitalWrite(S2_IN3, LOW); digitalWrite(S2_IN4, LOW);
}

// ── Bresenham simultaneous move ──
void moveSteppers(long tgtS1, long tgtS2) {
  long dS1 = tgtS1 - currentStepsS1;
  long dS2 = tgtS2 - currentStepsS2;
  int d1 = (dS1 > 0) ? 1 : -1;
  int d2 = (dS2 > 0) ? 1 : -1;
  long a1 = abs(dS1), a2 = abs(dS2);
  long mx = max(a1, a2);
  if (mx == 0) return;

  long e1 = 0, e2 = 0;
  for (long i = 0; i < mx; i++) {
    e1 += a1; e2 += a2;
    if (e1 >= mx) { e1 -= mx; stepS1(d1); currentStepsS1 += d1; }
    if (e2 >= mx) { e2 -= mx; stepS2(d2); currentStepsS2 += d2; }
  }
}

// ── Inverse Kinematics ──
bool inverseKinematics(float x, float y, float &ang1, float &ang2) {
  float dSq = x * x + y * y;
  float d = sqrt(dSq);
  if (d > (L1 + L2) || d < fabs(L1 - L2)) return false;

  float cosT2 = (dSq - L1*L1 - L2*L2) / (2.0 * L1 * L2);
  cosT2 = constrain(cosT2, -1.0, 1.0);
  float t2 = acos(cosT2);

  float k1 = L1 + L2 * cos(t2);
  float k2 = L2 * sin(t2);
  float t1 = atan2(y, x) - atan2(k2, k1);

  ang1 = t1 * 180.0 / M_PI;
  ang2 = t2 * 180.0 / M_PI;
  return true;
}

// ── Move to Cartesian point ──
bool moveTo(float x, float y) {
  float tA1, tA2;
  if (!inverseKinematics(x, y, tA1, tA2)) {
    Serial.print("!! Unreachable: "); Serial.print(x,1);
    Serial.print(", "); Serial.println(y,1);
    return false;
  }

  float dA1 = tA1 - currentAngle1;
  float dA2 = tA2 - currentAngle2;
  long tS1 = currentStepsS1 + (long)(dA1 / DEG_PER_STEP);
  long tS2 = currentStepsS2 + (long)(dA2 / DEG_PER_STEP);

  moveSteppers(tS1, tS2);
  currentAngle1 = tA1;
  currentAngle2 = tA2;
  return true;
}

// ── Get current pen position via forward kinematics ──
void getCurrentXY(float &x, float &y) {
  float t1r = currentAngle1 * M_PI / 180.0;
  float t2r = currentAngle2 * M_PI / 180.0;
  x = L1 * cos(t1r) + L2 * cos(t1r + t2r);
  y = L1 * sin(t1r) + L2 * sin(t1r + t2r);
}

// ── Draw a circle around current position ──
// Pen starts at current pos, offsets to the right by radius,
// then traces a full circle back to start.
void drawCircle(float radius) {
  float cx, cy;
  getCurrentXY(cx, cy);  // centre = current pen position

  // Move to first point (right of centre)
  moveTo(cx + radius, cy);

  // Trace the circle
  for (int i = 1; i <= CIRCLE_SEGMENTS; i++) {
    float angle = (2.0 * M_PI * i) / CIRCLE_SEGMENTS;
    float px = cx + radius * cos(angle);
    float py = cy + radius * sin(angle);
    moveTo(px, py);
  }

  // Return to centre
  moveTo(cx, cy);
}

// ── SETUP ──
void setup() {
  Serial.begin(115200);
  Serial.println("Circle Test - 2-Link Arm with IK");

  pinMode(S1_IN1, OUTPUT); pinMode(S1_IN2, OUTPUT);
  pinMode(S1_IN3, OUTPUT); pinMode(S1_IN4, OUTPUT);
  pinMode(S2_IN1, OUTPUT); pinMode(S2_IN2, OUTPUT);
  pinMode(S2_IN3, OUTPUT); pinMode(S2_IN4, OUTPUT);

  delay(1000);
}

// ── LOOP ──
void loop() {
  Serial.println("Drawing circle from current position...");

  drawCircle(CIRCLE_RADIUS);

  Serial.println("Circle complete!");
  delay(3000);
}
