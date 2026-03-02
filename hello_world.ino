/*
 * CNC Drawing Robot - Write "HELLO WORLD"
 * Arduino Mega 2560
 *
 * 2-Link rotary arm with inverse kinematics.
 * Stepper 1 (Base):   pins 4,5,6,7
 * Stepper 2 (Elbow):  pins 10,11,12,13
 * Servo (pen lift):   pin 9
 *
 * Arm: L1 = 140mm, L2 = 115mm
 */

#include <Servo.h>
#include <math.h>

// ── Pins ──
#define S1_IN1  4
#define S1_IN2  5
#define S1_IN3  6
#define S1_IN4  7

#define S2_IN1  10
#define S2_IN2  11
#define S2_IN3  12
#define S2_IN4  13

#define SERVO_PIN  9

// ── Motor ──
#define STEPS_PER_REV  4096
#define STEP_DELAY_US  1200

const float DEG_PER_STEP = 360.0 / STEPS_PER_REV;

// ── Arm ──
#define L1  140.0
#define L2  115.0

// ── Text layout ──
// Characters are drawn on a 4x6 unit grid, scaled to mm.
#define CHAR_HEIGHT  12.0    // mm tall
#define CHAR_GAP      3.0    // mm gap between characters
#define WORD_GAP      8.0    // mm extra gap for space
#define INTERP_STEPS  10     // interpolation points per line segment

const float SC = CHAR_HEIGHT / 6.0;  // scale factor (mm per unit)
const float CHAR_W = 4.0 * SC;       // character width in mm

// ── Where to draw (absolute coordinates from arm base) ──
// Arm starts straight at (255, 0). Text is placed within comfortable reach.
// Y = distance in front of arm base, X = left/right of centre
#define TEXT_START_X  -60.0   // mm — start to the left
#define TEXT_START_Y   150.0  // mm — in front of arm base (well within 255mm reach)

// ── Servo ──
#define PEN_UP_ANGLE    120
#define PEN_DOWN_ANGLE   60

// ── State ──
Servo penServo;
long currentStepsS1 = 0, currentStepsS2 = 0;
float currentAngle1 = 0.0, currentAngle2 = 0.0;
int phaseS1 = 0, phaseS2 = 0;
bool penIsDown = false;

const uint8_t seq[8][4] = {
  {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
  {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};

// ── Cursor position (bottom-left of next character) ──
float cursorX, cursorY;

// ═══════════════════════════════════════════
// Low-level motor functions
// ═══════════════════════════════════════════
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

void moveSteppers(long tgtS1, long tgtS2) {
  long dS1 = tgtS1 - currentStepsS1, dS2 = tgtS2 - currentStepsS2;
  int d1 = (dS1 > 0) ? 1 : -1, d2 = (dS2 > 0) ? 1 : -1;
  long a1 = abs(dS1), a2 = abs(dS2), mx = max(a1, a2);
  if (mx == 0) return;
  long e1 = 0, e2 = 0;
  for (long i = 0; i < mx; i++) {
    e1 += a1; e2 += a2;
    if (e1 >= mx) { e1 -= mx; stepS1(d1); currentStepsS1 += d1; }
    if (e2 >= mx) { e2 -= mx; stepS2(d2); currentStepsS2 += d2; }
  }
}

// ═══════════════════════════════════════════
// Kinematics
// ═══════════════════════════════════════════
bool inverseKinematics(float x, float y, float &a1, float &a2) {
  float dSq = x*x + y*y, d = sqrt(dSq);
  if (d > (L1+L2) || d < fabs(L1-L2)) return false;
  float cT2 = (dSq - L1*L1 - L2*L2) / (2.0*L1*L2);
  cT2 = constrain(cT2, -1.0, 1.0);
  float t2 = acos(cT2);
  float k1 = L1 + L2*cos(t2), k2 = L2*sin(t2);
  float t1 = atan2(y, x) - atan2(k2, k1);
  a1 = t1 * 180.0 / M_PI;
  a2 = t2 * 180.0 / M_PI;
  return true;
}

// Raw move to a single point (no interpolation)
bool moveToRaw(float x, float y) {
  float tA1, tA2;
  if (!inverseKinematics(x, y, tA1, tA2)) {
    Serial.print("!! Unreachable: "); Serial.print(x,1);
    Serial.print(","); Serial.println(y,1);
    return false;
  }
  long tS1 = currentStepsS1 + (long)((tA1 - currentAngle1) / DEG_PER_STEP);
  long tS2 = currentStepsS2 + (long)((tA2 - currentAngle2) / DEG_PER_STEP);
  moveSteppers(tS1, tS2);
  currentAngle1 = tA1; currentAngle2 = tA2;
  return true;
}

// Get current pen (x,y) from forward kinematics
void getCurrentXY(float &x, float &y) {
  float t1 = currentAngle1 * M_PI / 180.0;
  float t2 = currentAngle2 * M_PI / 180.0;
  x = L1*cos(t1) + L2*cos(t1+t2);
  y = L1*sin(t1) + L2*sin(t1+t2);
}

// Interpolated move: breaks the path into small segments
// so the pen follows a straight Cartesian line, not an arc.
bool moveTo(float x2, float y2) {
  float x1, y1;
  getCurrentXY(x1, y1);

  float dx = x2 - x1, dy = y2 - y1;
  float dist = sqrt(dx*dx + dy*dy);

  // For very short moves, just go directly
  if (dist < 1.0) {
    return moveToRaw(x2, y2);
  }

  // Break into small steps
  int steps = max((int)(dist / 1.5), INTERP_STEPS);
  for (int i = 1; i <= steps; i++) {
    float t = (float)i / steps;
    float px = x1 + dx * t;
    float py = y1 + dy * t;
    if (!moveToRaw(px, py)) return false;
  }
  return true;
}

// ═══════════════════════════════════════════
// Pen control
// ═══════════════════════════════════════════
void penUp() {
  if (!penIsDown) return;  // avoid redundant servo moves
  penServo.write(PEN_UP_ANGLE);
  delay(250);
  penIsDown = false;
}

void penDown() {
  if (penIsDown) return;
  penServo.write(PEN_DOWN_ANGLE);
  delay(250);
  penIsDown = true;
}

// ═══════════════════════════════════════════
// Drawing helpers
// ═══════════════════════════════════════════

// Travel to (x,y) with pen up
void travelTo(float x, float y) {
  penUp();
  moveTo(x, y);
}

// Draw a line from current position to (x,y) with pen down
void drawTo(float x, float y) {
  penDown();
  moveTo(x, y);
}

// Stroke: pen up → go to (x1,y1) → pen down → draw to (x2,y2)
void stroke(float x1, float y1, float x2, float y2) {
  travelTo(x1, y1);
  drawTo(x2, y2);
}

// Convert character-local grid coords to absolute coords
// Grid: (0,0)=bottom-left, (4,6)=top-right of character cell
float gx(float u) { return cursorX + u * SC; }
float gy(float v) { return cursorY + v * SC; }

// Advance cursor to next character position
void advanceCursor() {
  cursorX += CHAR_W + CHAR_GAP;
}
void advanceSpace() {
  cursorX += CHAR_W + WORD_GAP;
}

// ═══════════════════════════════════════════
// Character drawing functions (4x6 grid)
// ═══════════════════════════════════════════

void drawH() {
  stroke(gx(0), gy(0), gx(0), gy(6));   // left vertical
  stroke(gx(0), gy(3), gx(4), gy(3));   // middle bar
  stroke(gx(4), gy(0), gx(4), gy(6));   // right vertical
  advanceCursor();
}

void drawE() {
  travelTo(gx(4), gy(0));               // start at bottom-right
  drawTo(gx(0), gy(0));                 // bottom bar
  drawTo(gx(0), gy(3));                 // up to middle
  drawTo(gx(3), gy(3));                 // middle bar
  travelTo(gx(0), gy(3));               // back to left
  drawTo(gx(0), gy(6));                 // up to top
  drawTo(gx(4), gy(6));                 // top bar
  advanceCursor();
}

void drawL() {
  travelTo(gx(0), gy(6));               // start top-left
  drawTo(gx(0), gy(0));                 // down
  drawTo(gx(4), gy(0));                 // right along bottom
  advanceCursor();
}

void drawO() {
  travelTo(gx(0), gy(0));
  drawTo(gx(4), gy(0));
  drawTo(gx(4), gy(6));
  drawTo(gx(0), gy(6));
  drawTo(gx(0), gy(0));
  advanceCursor();
}

void drawW() {
  travelTo(gx(0), gy(6));
  drawTo(gx(1), gy(0));
  drawTo(gx(2), gy(4));
  drawTo(gx(3), gy(0));
  drawTo(gx(4), gy(6));
  advanceCursor();
}

void drawR() {
  travelTo(gx(0), gy(0));               // start bottom
  drawTo(gx(0), gy(6));                 // up full left
  drawTo(gx(3), gy(6));                 // across top
  drawTo(gx(4), gy(4.5));               // bump out
  drawTo(gx(3), gy(3));                 // back to mid
  drawTo(gx(0), gy(3));                 // back to left
  travelTo(gx(2), gy(3));               // start of leg
  drawTo(gx(4), gy(0));                 // diagonal leg
  advanceCursor();
}

void drawD() {
  travelTo(gx(0), gy(0));               // start bottom-left
  drawTo(gx(0), gy(6));                 // up
  drawTo(gx(3), gy(6));                 // across top
  drawTo(gx(4), gy(4.5));               // curve down
  drawTo(gx(4), gy(1.5));               // curve down more
  drawTo(gx(3), gy(0));                 // back in
  drawTo(gx(0), gy(0));                 // close
  advanceCursor();
}

// ═══════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("Hello World Writer - 2-Link Arm");

  pinMode(S1_IN1, OUTPUT); pinMode(S1_IN2, OUTPUT);
  pinMode(S1_IN3, OUTPUT); pinMode(S1_IN4, OUTPUT);
  pinMode(S2_IN1, OUTPUT); pinMode(S2_IN2, OUTPUT);
  pinMode(S2_IN3, OUTPUT); pinMode(S2_IN4, OUTPUT);

  penServo.attach(SERVO_PIN);
  penServo.write(PEN_UP_ANGLE);
  penIsDown = false;
  delay(1000);

  // ── Set initial arm position ──
  // Arm starts fully extended straight (both angles 0)
  // Pen is at (255, 0) along the X+ axis.
  currentAngle1 = 0.0;    // base angle — arm points right
  currentAngle2 = 0.0;    // elbow straight

  // Set cursor to the text start position
  cursorX = TEXT_START_X;
  cursorY = TEXT_START_Y;

  Serial.print("Text origin: (");
  Serial.print(cursorX, 1); Serial.print(", ");
  Serial.print(cursorY, 1); Serial.println(")");
  Serial.print("Arm reach: ");
  Serial.println(L1 + L2);

  // ── Travel to text start position ──
  Serial.println("Traveling to start...");
  travelTo(cursorX, cursorY);
  delay(500);

  // ── Write "HELLO WORLD" ──
  Serial.println("Writing: HELLO WORLD");

  drawH();  Serial.println("H done");
  drawE();  Serial.println("E done");
  drawL();  Serial.println("L done");
  drawL();  Serial.println("L done");
  drawO();  Serial.println("O done");

  advanceSpace();  // space between words

  drawW();  Serial.println("W done");
  drawO();  Serial.println("O done");
  drawR();  Serial.println("R done");
  drawL();  Serial.println("L done");
  drawD();  Serial.println("D done");

  // Done
  penUp();
  Serial.println("\nAll done!");
  disableMotors();
}

// ═══════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════
void loop() {
  // one-shot
}
