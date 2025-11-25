// Robot_code.ino
// Main sketch for the robot. This file wires together the
// movement and sensor helper modules and contains the high-level
// non-blocking state machine that controls robot behavior.
//
// Structure:
//  - Includes and context construction
//  - Hardware initialization in setup()
//  - Non-blocking state machine in loop() that calls helpers
//
// Note: This file is the owner of the concrete hardware objects and
// the context structures (MovementContext and SensorContext). Helper
// modules receive non-owning pointers to the contexts.

// === Required libraries ===
#include <DMotor_mod.h>
#include <Servo_ESP8266.h>
#include <math.h>
// Movement helper (class)
#include "movement/Movement.h"
// GetOut helper (class)
#include "get_out/get_out.h"
// Context definitions used to pass dependencies to helpers
#include "robot_shared.h"
// Slave helper (class)
#include "COMUNICACIONES_ROBOT/SlaveComm/src/SlaveComm.h"

// For Arduino build, include implementation .cpp files directly to
// ensure the IDE compiles them together as the original sketch did.
#include "movement/Movement.cpp"
#include "get_out/get_out.cpp"
#include "COMUNICACIONES_ROBOT/SlaveComm/src/SlaveComm.cpp"

// === Pins ===
#define ledPin D0
#define servoPin D7
#define sensorPin A0




// === Robot identifier and flags ===
const int id = 1; // unique robot id 
bool outFlag = false; // auxiliary flag for special modes
uint8_t masterMAC[] = {0xA4,0xCF,0x12,0xF5,0x20,0x53}; // MAC address of the master device

// === Motors and servo ===
// Right and left stepper motor controller objects, and the servo
// used for the distance sensor scanning mechanism.
AF_Stepper motorR(256, 1);    // right motor
AF_Stepper motorL(256, 2);    // left motor
Servo_ESP8266 sensorServo;    // servo used to sweep the distance sensor

// === Movement parameters ===
// stepToMicrostep: micro-stepping multiplier used together with the
// stepper resolution. baseSpeed: default speed for the stepper drivers in %.
int stepToMicrostep = 8;      // microsteps per full step
int baseSpeed = 100;          // base speed setting for steppers

// === Timing variables ===
// Non-blocking timing counters used to schedule sensor reads and
// periodic motor step events without blocking the main loop.
unsigned long previousMillisSensor = 0;
unsigned long previousMillisMotors = 0;
unsigned long previousMillisCom = 0;
const unsigned long intervalSensor = 50;  // sensor read interval (ms)
const unsigned long intervalCom = 5000;     // communication check interval (ms)
unsigned long motorDelay = 1;            // delay between micro-steps (smaller = faster)

// === Test mode configuration ===
// When TEST_MODE is enabled, the sketch feeds a predefined set of
// commands to the movement subsystem for automated testing.
#define TEST_MODE 0

const unsigned long testAutoInterval = 10000; // ms between automatic test commands
bool testAutoRun = true; // if true, test commands run automatically on a timer

#define SERIAL_STUB 0

const int TEST_CMD_COUNT = 6;
float testCommands[TEST_CMD_COUNT][3] = {
  {30.0, 0.0, 0.0},
  {20.0, 90.0, 0.0},
  {15.0, 180.0, 0.0},
  {10.0, 270.0, 0.0},
  {5.0, 45.0, 0.0},
  {0.0, 0.0, 1.0}
};
int testIndex = 0;
unsigned long previousMillisTest = 0;

// === Robot geometry parameters ===
// Wheel and chassis geometry used in step/angle computations.
const float WHEEL_RADIUS_CM = 3.2;      // wheel radius (cm)
const float WHEELS_AXIS_CM = 16.0;      // distance between wheel centers (cm)
const float WHEEL_DIAMETER_MM = WHEEL_RADIUS_CM * 2.0 * 10.0; // convert cm to mm
const float WHEEL_BASE_MM = WHEELS_AXIS_CM * 10.0;          // convert cm to mm
float twoPi = 2 * 3.1416;

int stepperResolution = 256;  // 8 bits, corresponde al primer parámetro de AF_Stepper
long stepsPerRevMicro() { return (long)stepperResolution * (long)stepToMicrostep; }

// Non-blocking movement state machine variable. The main loop uses
// this enum to determine what high-level behavior to run.
RobotMode currentMode = IDLE;

// Target and progress counters for each wheel (micro-steps)
long targetStepsR = 0;
long targetStepsL = 0;
long doneStepsR = 0;
long doneStepsL = 0;

// Current stepper directions (constants defined by the stepper API)
int dirR = FORWARD;
int dirL = FORWARD;

// Current and desired heading in degrees (0..360) and desired travel
// distance in centimeters as requested by remote commands or tests.
float currentHeading = 0.0;
float desiredHeading = 0.0;

// Last requested travel distance (cm)
float desiredDistanceCm = 0.0;

// === Sensor calibration and scan parameters ===
// tableEntries and mVinterval describe how the sensorDistance lookup
// table maps analog voltages to distances. SCAN_ANGLES defines the
// servo positions used for a sweep when searching for obstacles.
const int tableEntries = 10;
const int mVinterval = 100;
static float sensorDistance[tableEntries] = {80.0,70.0,60.0,50.0,40.0,30.0,25.0,20.0,15.0,10.0};

const float SAFE_DISTANCE = 27.0; // minimum safe distance (cm) before stopping
const int SCAN_ANGLES[] = {0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180};
const int NUM_SCAN_ANGLES = sizeof(SCAN_ANGLES) / sizeof(SCAN_ANGLES[0]);

// --- Movement tracking ---
// Non-blocking state used by higher-level maneuvers (scan/turn/drive).
bool moving = false;
unsigned long lastActionTime = 0;
unsigned long lastStepTime = 0;
unsigned long servoMoveStart = 0;
int currentScanIndex = 0;
float scanDistances[NUM_SCAN_ANGLES];
float lastDistance = 0;
int targetAngle = 90;
long currentSteps = 0;
long targetSteps = 0;

// Crear contexts especializados en vez de un RobotShared monolítico
MovementContext movementCtx = {
  stepsPerRevMicro,
  &WHEEL_BASE_MM,
  &WHEEL_DIAMETER_MM,
  &currentHeading,
  &desiredHeading,
  &desiredDistanceCm,
  &targetStepsR,
  &targetStepsL,
  &doneStepsR,
  &doneStepsL,
  &dirR,
  &dirL,
  &stepperResolution,
  &stepToMicrostep,
  &motorR,
  &motorL
};

SensorContext sensorCtx = {
  &tableEntries,
  &mVinterval,
  sensorDistance,
  sensorPin,
  &WHEEL_RADIUS_CM,
  &WHEELS_AXIS_CM,
  &motorR,
  &motorL,
  &sensorServo,
  &outFlag,
  &stepperResolution,
  &stepToMicrostep
};

// Crear instancias de las clases usando los contexts apropiados
Movement mover(&movementCtx);
GetOut getout(&sensorCtx);
SlaveComm slave;



// Initialize hardware and modules. Attach servos, configure motor
// speeds and initialize any test timers used when TEST_MODE is enabled.
void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Prepare servo used for scanning and park it at center (90°)
  sensorServo.attach(servoPin);
  sensorServo.write(90);

  // Configure stepper base speed
  motorR.setSpeed(baseSpeed);
  motorL.setSpeed(baseSpeed);

  #if TEST_MODE
    // Initialize automatic test command timer and inform via serial
    previousMillisTest = millis();
    Serial.println("*** TEST_MODE enabled: using built-in command list (auto-run) ***");
    Serial.print("Auto-run interval (ms): "); Serial.println(testAutoInterval);
  #endif

  // Initialize SlaveComm and set robot ID
  slave.setID(id);
  slave.setMasterMACAddress(masterMAC);
  slave.begin("OPPO A53", "611b10a883c5"); // WiFi credentials for ESP-NOW
}

void loop() {
  unsigned long currentMillis = millis();

  if (slave.dataChanged()) {
    Serial.println("Angulo: " + String(slave.getAngle()));
    Serial.println("Distancia: " + String(slave.getDistance()));
    Serial.println("Out: " + String(slave.getOut())); 
    mover.processRemoteCommand(slave.getDistance(), slave.getAngle(), slave.getOut());
    
  }

  #if TEST_MODE
    // Test mode: periodically feed a predefined command sequence to the
    // movement subsystem. This keeps the control flow exercised when
    // no remote input is present.
    bool haveNewCommand = false;
    if (testAutoRun && (currentMillis - previousMillisTest >= testAutoInterval)) {
      haveNewCommand = true;
    }

    if (haveNewCommand) {
      if (testCommands[testIndex][2] == 1){
        // Special command (out flag)
        mover.processRemoteCommand(0.0, 0.0, 1);
        testAutoRun = false;
      } else{
        mover.processRemoteCommand(testCommands[testIndex][0], testCommands[testIndex][1], 0);
      }
      previousMillisTest = currentMillis;
      testIndex++;
      if (testIndex >= TEST_CMD_COUNT) testIndex = 0;
    }
  #endif

  // Stepping state machine: run periodic motor-step updates and the
  // high-level behavior state machine. This section is intentionally
  // non-blocking; it advances motion one micro-step at a time.
  if (currentMillis - previousMillisMotors >= motorDelay) {
    previousMillisMotors = currentMillis;

    if (currentMode == ROTATING) {
      if (doneStepsR < targetStepsR) { motorR.step(1, dirR, SINGLE); doneStepsR++; }
      if (doneStepsL < targetStepsL) { motorL.step(1, dirL, SINGLE); doneStepsL++; }
      if (doneStepsR >= targetStepsR && doneStepsL >= targetStepsL) {
        currentHeading = desiredHeading;
        Serial.print("[TEST] Giro completado. Heading ahora: "); Serial.println(currentHeading);
        long moveSteps = mover.distanceCmToSteps(desiredDistanceCm);
        if (moveSteps > 0) {
          dirR = FORWARD; dirL = FORWARD;
          targetStepsR = moveSteps; targetStepsL = moveSteps;
          doneStepsR = 0; doneStepsL = 0;
          currentMode = MOVING;
          Serial.print("[TEST] Iniciando avance (pasos): "); Serial.println(moveSteps);
        } else {
          currentMode = IDLE;
        }
      }

    } else if (currentMode == MOVING) {
      if (doneStepsR < targetStepsR) { motorR.step(1, dirR, SINGLE); doneStepsR++; }
      if (doneStepsL < targetStepsL) { motorL.step(1, dirL, SINGLE); doneStepsL++; }
      if (doneStepsR >= targetStepsR && doneStepsL >= targetStepsL) {
        Serial.println("[TEST] Movimiento completado.");
        currentMode = IDLE;
      }

    } else if (currentMode == TURN_180) {
      if (!moving) {
        Serial.println("🔄 Turning 180°...");
        targetSteps = getout.computeSteps(180);
        currentSteps = 0;
        moving = true;
      }
      if (moving && (currentMillis - lastStepTime >= 2)) {
        getout.stepMotors('R');
        currentSteps++;
        lastStepTime = currentMillis;
        if (currentSteps >= targetSteps) {
          moving = false;
          currentScanIndex = 0;
          currentMode = SCAN;
          Serial.println("✅ Turn 180° done, starting scan.");
        }
      }

    } else if (currentMode == SCAN) {
      if (currentScanIndex < NUM_SCAN_ANGLES) {
        if (servoMoveStart == 0) {
          int angle = SCAN_ANGLES[currentScanIndex];
          sensorServo.write(angle);
          servoMoveStart = currentMillis;
        }
        if (currentMillis - servoMoveStart >= 250) {
          scanDistances[currentScanIndex] = getout.getDistanceFromSensor();
          Serial.print("Angle ");
          Serial.print(SCAN_ANGLES[currentScanIndex]);
          Serial.print("° -> ");
          Serial.print(scanDistances[currentScanIndex]);
          Serial.println(" cm");
          currentScanIndex++;
          servoMoveStart = 0;
        }
      } else {
        float minDist = scanDistances[0];
        int minIdx = 0;
        for (int i = 1; i < NUM_SCAN_ANGLES; i++) {
          if (scanDistances[i] < minDist) { minDist = scanDistances[i]; minIdx = i; }
        }
        targetAngle = SCAN_ANGLES[minIdx];
        Serial.print("Closest wall at ");
        Serial.print(targetAngle);
        Serial.print("° (");
        Serial.print(minDist);
        Serial.println(" cm)");
        sensorServo.write(90);
        currentMode = TURN_TO_TARGET;
      }

    } else if (currentMode == TURN_TO_TARGET) {
      if (!moving) {
        float diff = fabs(targetAngle - 90);
        if (diff < 3) {
          Serial.println("Already facing target.");
          currentMode = DRIVE_FORWARD;
        } else {
          char dir = (targetAngle < 90) ? 'R' : 'L';
          targetSteps = getout.computeSteps(diff);
          currentSteps = 0;
          moving = true;
          Serial.print("Turning "); Serial.print((dir == 'R') ? "right " : "left "); Serial.print(diff); Serial.println("°");
        }
      } else if (currentMillis - lastStepTime >= 2) {
        char dir = (targetAngle < 90) ? 'R' : 'L';
        getout.stepMotors(dir);
        currentSteps++;
        lastStepTime = currentMillis;
        if (currentSteps >= targetSteps) {
          moving = false;
          currentMode = DRIVE_FORWARD;
          Serial.println("✅ Aligned to target.");
        }
      }

    } else if (currentMode == DRIVE_FORWARD) {
      if (currentMillis - lastStepTime >= 2) {
        getout.stepMotors('F');
        lastStepTime = currentMillis;
      }
      if (currentMillis - lastActionTime >= 100) {
        lastDistance = getout.getDistanceFromSensor();
        Serial.print("Distance: ");
        Serial.println(lastDistance);
        lastActionTime = currentMillis;
      }
      if (lastDistance <= SAFE_DISTANCE && lastDistance > 0) {
        Serial.println("🧱 Close to wall. Stopping and preparing to turn back.");
        currentMode = TURN_BACK;
        moving = false;
      }

    } else if (currentMode == TURN_BACK) {
      if (!moving) {
        Serial.println("🔁 Turning back 180°...");
        targetSteps = getout.computeSteps(180);
        currentSteps = 0;
        moving = true;
      }
      if (moving && (currentMillis - lastStepTime >= 2)) {
        getout.stepMotors('R');
        currentSteps++;
        lastStepTime = currentMillis;
        if (currentSteps >= targetSteps) {
          moving = false;
          currentMode = IDLE;
          lastActionTime = currentMillis;
          Serial.println("✅ Turn back done. Waiting...");
        }
      }
    }
  }
}
