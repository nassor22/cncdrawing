/*
 * CNC Drawing Robot - 2-Link Arm (SCARA style)
 * Draws a single circle at the centre of an A4 paper.
 *
 * Hardware:
 *   - Arduino Mega 2560
 *   - Stepper Motor 1 (base joint):  28BYJ-48 via ULN2003, IN1-IN4 → pins 4-7
 *   - Stepper Motor 2 (elbow joint): 28BYJ-48 via ULN2003, IN1-IN4 → pins 10-13
 *   - Servo Motor (pen lift):        signal → pin 9
 *
 * IMPORTANT: Measure your actual arm link lengths and paper position,
 *            then update L1, L2, PAPER_X_OFFSET, PAPER_Y_OFFSET below.
 */

#include <Servo.h>
#include <math.h>

// ─────────────────────────────────────────────
// Pin definitions
// ─────────────────────────────────────────────
// Stepper 1 (Base) – ULN2003 IN1..IN4
#define S1_IN1  4
#define S1_IN2  5
#define S1_IN3  6
#define S1_IN4  7

// Stepper 2 (Elbow) – ULN2003 IN1..IN4
#define S2_IN1  10
#define S2_IN2  11
#define S2_IN3  12
#define S2_IN4  13

// Servo
#define SERVO_PIN  9

// ─────────────────────────────────────────────
// Motor parameters
// ─────────────────────────────────────────────
// 28BYJ-48: 2048 steps per revolution in half-step mode (4096 half-steps)
// Using half-step sequence (8 phases) for smoother motion
#define STEPS_PER_REV  4096        // half-step mode
#define STEP_DELAY_US  1200        // microseconds between steps (lower = faster, min ~900)

// Degrees per step
const float DEG_PER_STEP = 360.0 / STEPS_PER_REV;

// ─────────────────────────────────────────────
// Arm geometry (mm) — MEASURE YOUR ARM!
// ─────────────────────────────────────────────
#define L1  140.0   // Length of link 1 (base pivot to elbow pivot) — 14 cm
#define L2  115.0   // Length of link 2 (elbow pivot to pen tip) — 11.5 cm

// ─────────────────────────────────────────────
// Paper placement
// ─────────────────────────────────────────────
// The robot base (stepper 1 shaft) is the origin (0,0).
// PAPER_X_OFFSET, PAPER_Y_OFFSET = coordinates of the bottom-left
// corner of the A4 sheet relative to the robot base (in mm).
// Adjust these so the arm can reach all 4 corners!
#define PAPER_X_OFFSET   -105.0   // mm (centred on X)
#define PAPER_Y_OFFSET    120.0   // mm (in front of base)

// A4 dimensions
#define A4_WIDTH   210.0   // mm  (short edge)
#define A4_HEIGHT  297.0   // mm  (long edge)

// ─────────────────────────────────────────────
// Drawing parameters
// ─────────────────────────────────────────────
#define CIRCLE_RADIUS   15.0   // mm
#define CIRCLE_SEGMENTS 72     // points per circle (5° per segment)
#define CORNER_MARGIN   30.0   // mm inset from paper edge to circle centre

// Servo angles
#define PEN_DOWN_ANGLE  60     // servo angle when pen touches paper
#define PEN_UP_ANGLE    120    // servo angle when pen is lifted

// ─────────────────────────────────────────────
// Global state
// ─────────────────────────────────────────────
Servo penServo;

// Current stepper positions (in steps, relative to home = 0)
long currentStepsS1 = 0;   // base
long currentStepsS2 = 0;   // elbow

// Current joint angles in degrees (from home)
float currentAngle1 = 0.0;
float currentAngle2 = 0.0;

// Half-step sequence for 28BYJ-48 (8 phases)
// Phase order: IN1, IN2, IN3, IN4
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

// Current phase index for each stepper
int phaseS1 = 0;
int phaseS2 = 0;

// ─────────────────────────────────────────────
// Stepper low-level
// ─────────────────────────────────────────────
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

// Step stepper 1 by one half-step (+1 = CW, -1 = CCW)
void stepS1(int dir) {
  phaseS1 = (phaseS1 + dir + 8) % 8;
  setStepperPins(S1_IN1, S1_IN2, S1_IN3, S1_IN4, halfStepSeq[phaseS1]);
  delayMicroseconds(STEP_DELAY_US);
}

// Step stepper 2 by one half-step
void stepS2(int dir) {
  phaseS2 = (phaseS2 + dir + 8) % 8;
  setStepperPins(S2_IN1, S2_IN2, S2_IN3, S2_IN4, halfStepSeq[phaseS2]);
  delayMicroseconds(STEP_DELAY_US);
}

// ─────────────────────────────────────────────
// Move steppers to target step counts (simultaneous Bresenham)
// ─────────────────────────────────────────────
void moveSteppers(long targetS1, long targetS2) {
  long deltaS1 = targetS1 - currentStepsS1;
  long deltaS2 = targetS2 - currentStepsS2;

  int dirS1 = (deltaS1 > 0) ? 1 : -1;
  int dirS2 = (deltaS2 > 0) ? 1 : -1;

  long absS1 = abs(deltaS1);
  long absS2 = abs(deltaS2);

  long maxSteps = max(absS1, absS2);
  if (maxSteps == 0) return;

  // Bresenham-like simultaneous stepping
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

// ─────────────────────────────────────────────
// Inverse Kinematics (2-link planar arm)
// ─────────────────────────────────────────────
// Returns true if (x, y) is reachable; sets angle1, angle2 in degrees.
bool inverseKinematics(float x, float y, float &angle1, float &angle2) {
  float distSq = x * x + y * y;
  float dist   = sqrt(distSq);

  // Reachability check
  if (dist > (L1 + L2) || dist < fabs(L1 - L2)) {
    return false;
  }

  // Elbow angle (θ2)
  float cosTheta2 = (distSq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  cosTheta2 = constrain(cosTheta2, -1.0, 1.0);
  float theta2 = acos(cosTheta2);          // elbow-down solution (positive)

  // Base angle (θ1)
  float k1 = L1 + L2 * cos(theta2);
  float k2 = L2 * sin(theta2);
  float theta1 = atan2(y, x) - atan2(k2, k1);

  // Convert radians to degrees
  angle1 = theta1 * 180.0 / M_PI;
  angle2 = theta2 * 180.0 / M_PI;

  return true;
}

// ─────────────────────────────────────────────
// Move pen tip to Cartesian (x, y) in mm
// ─────────────────────────────────────────────
bool moveTo(float x, float y) {
  float targetAngle1, targetAngle2;

  if (!inverseKinematics(x, y, targetAngle1, targetAngle2)) {
    Serial.print("  !! Unreachable: (");
    Serial.print(x, 1); Serial.print(", "); Serial.print(y, 1);
    Serial.println(")");
    return false;
  }

  // Convert angle differences to steps
  float dAngle1 = targetAngle1 - currentAngle1;
  float dAngle2 = targetAngle2 - currentAngle2;

  long targetStepsS1 = currentStepsS1 + (long)(dAngle1 / DEG_PER_STEP);
  long targetStepsS2 = currentStepsS2 + (long)(dAngle2 / DEG_PER_STEP);

  moveSteppers(targetStepsS1, targetStepsS2);

  currentAngle1 = targetAngle1;
  currentAngle2 = targetAngle2;

  return true;
}

// ─────────────────────────────────────────────
// Pen control
// ─────────────────────────────────────────────
void penUp() {
  penServo.write(PEN_UP_ANGLE);
  delay(300);   // wait for servo to move
}

void penDown() {
  penServo.write(PEN_DOWN_ANGLE);
  delay(300);
}

// ─────────────────────────────────────────────
// Draw a circle centred at (cx, cy) with given radius
// ─────────────────────────────────────────────
void drawCircle(float cx, float cy, float radius) {
  Serial.print("Drawing circle at (");
  Serial.print(cx, 1); Serial.print(", "); Serial.print(cy, 1);
  Serial.println(")");

  // Move to first point on circle with pen up
  float startX = cx + radius;
  float startY = cy;

  penUp();
  if (!moveTo(startX, startY)) return;
  penDown();

  // Draw the circle
  for (int i = 1; i <= CIRCLE_SEGMENTS; i++) {
    float angle = (2.0 * M_PI * i) / CIRCLE_SEGMENTS;
    float px = cx + radius * cos(angle);
    float py = cy + radius * sin(angle);
    moveTo(px, py);
  }

  penUp();
  Serial.println("  Circle done.");
}

// ─────────────────────────────────────────────
// Home / calibration
// ─────────────────────────────────────────────
void homePosition() {
  // Assumes the arm starts at its "home" position when powered on.
  // Home = both angles at 0 → arm fully extended along X-axis.
  // If you add endstops, add homing logic here.
  currentAngle1 = 0.0;
  currentAngle2 = 0.0;
  currentStepsS1 = 0;
  currentStepsS2 = 0;

  // Compute initial angles from a known safe position
  // Move arm to a neutral "ready" position (straight ahead)
  float readyX = (L1 + L2) * 0.7;   // 70% of max reach
  float readyY = 0;
  inverseKinematics(readyX, readyY, currentAngle1, currentAngle2);
}

// ─────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("CNC Drawing Robot - Starting up");

  // Stepper 1 pins
  pinMode(S1_IN1, OUTPUT); pinMode(S1_IN2, OUTPUT);
  pinMode(S1_IN3, OUTPUT); pinMode(S1_IN4, OUTPUT);

  // Stepper 2 pins
  pinMode(S2_IN1, OUTPUT); pinMode(S2_IN2, OUTPUT);
  pinMode(S2_IN3, OUTPUT); pinMode(S2_IN4, OUTPUT);

  // Servo
  penServo.attach(SERVO_PIN);
  penUp();

  delay(1000);

  // ── Define the circle centre (A4 centre) ──
  float cx = PAPER_X_OFFSET + A4_WIDTH  / 2.0;
  float cy = PAPER_Y_OFFSET + A4_HEIGHT / 2.0;

  Serial.println("=== Drawing 1 circle at A4 centre ===");
  Serial.print("Paper origin: ("); Serial.print(PAPER_X_OFFSET);
  Serial.print(", "); Serial.print(PAPER_Y_OFFSET); Serial.println(")");
  Serial.print("Centre target: ("); Serial.print(cx, 1);
  Serial.print(", "); Serial.print(cy, 1); Serial.println(")");
  Serial.print("Arm reach: "); Serial.print(L1 + L2); Serial.println(" mm");

  // ── Draw circle at centre ──
  Serial.println("\n--- Circle: Centre ---");
  drawCircle(cx, cy, CIRCLE_RADIUS);

  // ── Done — return to neutral and disable motors ──
  Serial.println("\n=== Circle complete! ===");
  penUp();
  disableStepper1();
  disableStepper2();

  Serial.println("Motors disabled. Done.");
}

// ─────────────────────────────────────────────
// LOOP (nothing to do — one-shot drawing)
// ─────────────────────────────────────────────
void loop() {
  // Nothing — drawing is done in setup()
}
