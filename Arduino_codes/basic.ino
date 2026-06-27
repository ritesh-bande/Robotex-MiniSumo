
//
//  HOW THE 4 ToF SENSORS SHARE ONE I2C BUS (XSHUT method):
//  All 4 VL53L0X SDA/SCL pins are wired together to the SAME
//  ESP32 SDA/SCL lines. Every VL53L0X boots at the same default
//  I2C address (0x29), so two awake at once would collide.
//  XSHUT (held LOW) puts a sensor into hardware shutdown, fully
//  disconnecting it from the bus electrically. At boot we:
//    1. Hold ALL 4 sensors in shutdown (XSHUT = LOW)
//    2. Release ONE sensor's XSHUT (HIGH)
//    3. While it's the only one awake, give it a new unique
//       I2C address via setAddress()
//    4. Repeat for the next sensor
//  After this, all 4 live on the same bus at 4 different
//  addresses — no multiplexer chip needed.
//
//  Requires libraries (Arduino Library Manager):
//    - "Adafruit VL53L0X"          (distance sensors)
//    - "SparkFun TB6612FNG Arduino Library"   (motor driver)
//    - "QTRSensors" by Pololu      (edge sensors)
// ============================================================

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <SparkFun_TB6612.h>
#include <QTRSensors.h>

// ------------------------------------------------------------
//  I2C bus pins (shared by all 4 VL53L0X)
// ------------------------------------------------------------
#define I2C_SDA  21
#define I2C_SCL  22

// ------------------------------------------------------------
//  XSHUT pins — one dedicated GPIO per sensor
//  U26 -> FRONT, U27 -> RIGHT, U28 -> LEFT, U29 -> BACK
// ------------------------------------------------------------
#define XSHUT_FRONT  16   // U26 -> RX2
#define XSHUT_RIGHT  17   // U27 -> TX2
#define XSHUT_LEFT   18   // U28 -> D18
#define XSHUT_BACK   19   // U29 -> D19

// ------------------------------------------------------------
//  Unique I2C addresses assigned to each sensor at boot
//  (0x29 is the factory default, never used after init)
// ------------------------------------------------------------
#define ADDR_FRONT  0x30
#define ADDR_RIGHT  0x31
#define ADDR_LEFT   0x32
#define ADDR_BACK   0x33

Adafruit_VL53L0X tofFront = Adafruit_VL53L0X();
Adafruit_VL53L0X tofRight = Adafruit_VL53L0X();
Adafruit_VL53L0X tofLeft  = Adafruit_VL53L0X();
Adafruit_VL53L0X tofBack  = Adafruit_VL53L0X();

// ------------------------------------------------------------
//  TB6612FNG Motor Driver Pins (SparkFun library)
// ------------------------------------------------------------
// Motor A = Left Motor
#define AIN1  32
#define AIN2  35
#define PWMA  34

// Motor B = Right Motor
#define BIN1  33
#define BIN2  25
#define PWMB  26

// STBY is tied directly to 3V3 on this board (confirmed) — not a GPIO.
// The SparkFun Motor() constructor still requires a pin number for its
// STBY argument, so we pass a harmless unused pin and never toggle it
// in software, since the hardware already holds it HIGH permanently.
#define STBY_UNUSED  -1

// these constants let the library's forward()/back()/left()/right()
// helpers match your physical wiring. Flip a value to -1 if a wheel
// spins backwards during testing instead of rewriting drive logic.
const int offsetA = 1;
const int offsetB = 1;

// Motor objects: Motor(IN1, IN2, PWM, offset, STBY)
Motor motorA = Motor(AIN1, AIN2, PWMA, offsetA, STBY_UNUSED);
Motor motorB = Motor(BIN1, BIN2, PWMB, offsetB, STBY_UNUSED);

// ------------------------------------------------------------
//  QTR-8A Edge Sensor Pins (only channel 1 and channel 8 used)
// ------------------------------------------------------------
#define EDGE_LEFT   12   // QTR Pin 1 -> D12
#define EDGE_RIGHT  13   // QTR Pin 8 -> D13

#define EDGE_THRESHOLD  500   // 0-1023 (QTRSensors raw analog range). Tune on real Dohyo.

QTRSensors qtr;
const uint8_t QTR_SENSOR_COUNT = 2;
uint16_t qtrValues[QTR_SENSOR_COUNT];

// ------------------------------------------------------------
//  DIP switch pins (U24) — handy for pre-match configuration,
//  e.g. selecting attack strategy or detection range without
//  reflashing. Read once at boot.
// ------------------------------------------------------------
#define DIP_SW_4  15   // D15
#define DIP_SW_5  2    // D2
#define DIP_SW_6  4    // D4

// ------------------------------------------------------------
//  ToF Detection Range (mm)
// ------------------------------------------------------------
#define DETECT_RANGE_MM   400
#define ATTACK_RANGE_MM   150

// ------------------------------------------------------------
//  Motor Speed Settings (0-255, used with the Motor library)
// ------------------------------------------------------------
#define SPEED_FULL    220
#define SPEED_SEARCH  160
#define SPEED_REVERSE 180
#define SPEED_TURN    170

// ------------------------------------------------------------
//  Timing
// ------------------------------------------------------------
#define START_DELAY_MS  5000   // Mandatory 5-second delay (Rule 5.1.2)
#define REVERSE_MS       300
#define TURN_MS          250

uint16_t distFront = 9999, distRight = 9999, distLeft = 9999, distBack = 9999;

// ============================================================
//  XSHUT-based sensor boot sequence
// ============================================================
bool initToFSensor(Adafruit_VL53L0X &sensor, uint8_t xshutPin, uint8_t newAddr, const char* name) {
  // Step 1: bring this sensor OUT of shutdown
  digitalWrite(xshutPin, HIGH);
  delay(10);  // sensor boot-up time

  // Step 2: only this sensor is alive on the bus right now,
  // still at its default address 0x29 — safe to talk to it.
  if (!sensor.begin(0x29)) {
    Serial.print(name);
    Serial.println(": FAILED to initialize at default address!");
    return false;
  }

  // Step 3: re-assign it to its permanent unique address
  sensor.setAddress(newAddr);
  Serial.print(name);
  Serial.print(": OK, now at address 0x");
  Serial.println(newAddr, HEX);
  return true;
}

void setupToFSensors() {
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_LEFT,  OUTPUT);
  pinMode(XSHUT_BACK,  OUTPUT);

  // Step 0: hold ALL sensors in shutdown first
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_LEFT,  LOW);
  digitalWrite(XSHUT_BACK,  LOW);
  delay(10);

  // Boot + address each sensor ONE AT A TIME
  initToFSensor(tofFront, XSHUT_FRONT, ADDR_FRONT, "FRONT");
  initToFSensor(tofRight, XSHUT_RIGHT, ADDR_RIGHT, "RIGHT");
  initToFSensor(tofLeft,  XSHUT_LEFT,  ADDR_LEFT,  "LEFT");
  initToFSensor(tofBack,  XSHUT_BACK,  ADDR_BACK,  "BACK");
}

void readAllSensors() {
  VL53L0X_RangingMeasurementData_t m;

  tofFront.rangingTest(&m, false);
  distFront = (m.RangeStatus != 4) ? m.RangeMm : 9999;

  tofRight.rangingTest(&m, false);
  distRight = (m.RangeStatus != 4) ? m.RangeMm : 9999;

  tofLeft.rangingTest(&m, false);
  distLeft = (m.RangeStatus != 4) ? m.RangeMm : 9999;

  tofBack.rangingTest(&m, false);
  distBack = (m.RangeStatus != 4) ? m.RangeMm : 9999;
}

// ============================================================
//  Motor Control (using SparkFun_TB6612 library)
//
//  Motor::drive(speed) takes -255..255. Positive/negative sign
//  picks direction; this depends on offsetA/offsetB above. If a
//  wheel spins the wrong way during testing, flip that motor's
//  offset constant from 1 to -1 (or vice versa) rather than
//  rewriting the drive functions below.
// ============================================================
void driveForward(int speed) {
  motorA.drive(speed);
  motorB.drive(speed);
}

void driveReverse(int speed) {
  motorA.drive(-speed);
  motorB.drive(-speed);
}

// Pivot so the RIGHT side comes around to face forward
void turnRight(int speed) {
  motorA.drive(speed);    // left wheel forward
  motorB.drive(-speed);   // right wheel backward
}

// Pivot so the LEFT side comes around to face forward
void turnLeft(int speed) {
  motorA.drive(-speed);   // left wheel backward
  motorB.drive(speed);    // right wheel forward
}

void stopMotors() {
  motorA.brake();
  motorB.brake();
}

// ============================================================
//  Edge Sensors (QTR-8A, channels 1 and 8 only, via QTRSensors library)
// ============================================================
void setupEdgeSensors() {
  uint8_t sensorPins[QTR_SENSOR_COUNT] = { EDGE_LEFT, EDGE_RIGHT };
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, QTR_SENSOR_COUNT);
}

// QTRSensors raw analog reading: higher value = LESS reflective
// (e.g. void/black). On a white Tawara line vs black Dohyo, check
// which direction your specific sensor reads and adjust the
// comparison below if it's inverted on your board.
bool edgeOnLeft() {
  qtr.read(qtrValues);
  return qtrValues[0] > EDGE_THRESHOLD;
}

bool edgeOnRight() {
  qtr.read(qtrValues);
  return qtrValues[1] > EDGE_THRESHOLD;
}

// Reads both channels in a single pass (more efficient than calling
// edgeOnLeft()/edgeOnRight() separately, since each call to qtr.read()
// re-reads every configured channel anyway).
void readEdges(bool &leftEdge, bool &rightEdge) {
  qtr.read(qtrValues);
  leftEdge  = qtrValues[0] > EDGE_THRESHOLD;
  rightEdge = qtrValues[1] > EDGE_THRESHOLD;
}

void avoidEdge(bool leftEdge, bool rightEdge) {
  driveReverse(SPEED_REVERSE);
  delay(REVERSE_MS);

  if (leftEdge && !rightEdge) {
    turnRight(SPEED_TURN);
    delay(TURN_MS);
  } else if (rightEdge && !leftEdge) {
    turnLeft(SPEED_TURN);
    delay(TURN_MS);
  } else {
    // both at once (hit a corner) — back off further, then turn
    driveReverse(SPEED_REVERSE);
    delay(REVERSE_MS);
    turnRight(SPEED_TURN);
    delay(TURN_MS * 2);
  }
}

// ============================================================
//  DIP switch read (optional pre-match configuration)
// ============================================================
void readDipSwitches() {
  pinMode(DIP_SW_4, INPUT_PULLUP);
  pinMode(DIP_SW_5, INPUT_PULLUP);
  pinMode(DIP_SW_6, INPUT_PULLUP);

  bool sw4 = digitalRead(DIP_SW_4) == LOW;  // LOW = ON, depends on wiring
  bool sw5 = digitalRead(DIP_SW_5) == LOW;
  bool sw6 = digitalRead(DIP_SW_6) == LOW;

  Serial.printf("DIP switches -> SW4:%d SW5:%d SW6:%d\n", sw4, sw5, sw6);
  // Use these flags to branch strategy if desired, e.g.:
  // if (sw4) DETECT_RANGE_MM stays default vs reduced, etc.
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  setupEdgeSensors();

  stopMotors();

  readDipSwitches();

  Serial.println("Booting ToF sensors via XSHUT sequence...");
  setupToFSensors();

  Serial.println("Mini Sumo Ready");
  Serial.println("Waiting 5 seconds (Rule 5.1.2)...");

  // ---- MANDATORY 5-SECOND DELAY (Robotex Rule 5.1.2) ----
  delay(START_DELAY_MS);

  Serial.println("GO!");
}

// ============================================================
//  MAIN LOOP
//
//  Priority:
//  1. Edge avoidance (never fall off Dohyo)
//  2. Attack — use 4-direction ToF data to find & charge enemy
//  3. Search — spin until ToF sees something
// ============================================================
void loop() {

  bool leftEdge, rightEdge;
  readEdges(leftEdge, rightEdge);

  // ---- 1. EDGE AVOIDANCE (highest priority) ----
  if (leftEdge || rightEdge) {
    Serial.println("EDGE DETECTED — avoiding!");
    avoidEdge(leftEdge, rightEdge);
    return;
  }

  // ---- Read all 4 ToF sensors ----
  readAllSensors();
  Serial.printf("F:%d R:%d L:%d B:%d\n", distFront, distRight, distLeft, distBack);

  bool seeFront = distFront < DETECT_RANGE_MM;
  bool seeRight = distRight < DETECT_RANGE_MM;
  bool seeLeft  = distLeft  < DETECT_RANGE_MM;
  bool seeBack  = distBack  < DETECT_RANGE_MM;

  // ---- 2. ATTACK ----
  if (seeFront) {
    int speed = (distFront < ATTACK_RANGE_MM) ? SPEED_FULL : SPEED_SEARCH;
    driveForward(speed);

  } else if (seeRight) {
    turnRight(SPEED_TURN);
    delay(60);

  } else if (seeLeft) {
    turnLeft(SPEED_TURN);
    delay(60);

  } else if (seeBack) {
    turnRight(SPEED_TURN);
    delay(150);

  } else {
    // ---- 3. SEARCH ----
    turnRight(SPEED_SEARCH);
  }

  delay(10);
}
