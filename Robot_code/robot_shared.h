#ifndef ROBOT_SHARED_H
#define ROBOT_SHARED_H

#include "robot_state.h"
#include <DMotor_mod.h>
#include <Servo_ESP8266.h>

// robot_shared.h
// Lightweight context definitions used to group related dependencies
// passed into helper modules. This reduces coupling by ensuring each
// module receives only the fields it needs instead of a monolithic
// shared structure.
//
// Two contexts are provided:
//  - MovementContext: contains pointers and parameters required by
//    movement logic and stepper control (headings, step counters,
//    wheel geometry and pointers to AF_Stepper instances).
//  - SensorContext: contains sensor calibration tables, the analog
//    pin number, servo reference, and a few fields needed by scan/
//    sensor helpers.

typedef long (*StepsFunc)();

// MovementContext bundles values used by movement calculations and
// by code that needs direct access to the stepper motors and
// heading/distance variables. Fields are pointers where the actual
// storage lives in the sketch to avoid duplicating state.
struct MovementContext {
  StepsFunc stepsPerRevMicro; // function pointer to compute steps per rev
  const float* WHEEL_BASE_MM; // distance between wheels (mm)
  const float* WHEEL_DIAMETER_MM; // wheel diameter (mm)
  float* currentHeading; // pointer to shared current heading (deg)
  float* desiredHeading; // pointer to desired heading (deg)
  float* desiredDistanceCm; // pointer to desired travel distance (cm)
  long* targetStepsR; // pointer to right wheel target steps
  long* targetStepsL; // pointer to left wheel target steps
  long* doneStepsR; // pointer to right wheel completed steps
  long* doneStepsL; // pointer to left wheel completed steps
  int* dirR; // pointer to right wheel direction
  int* dirL; // pointer to left wheel direction

  int* stepperResolution; // pointer to stepper resolution value
  int* stepToMicrostep; // pointer to microstep factor

  AF_Stepper* motorR; // pointer to right AF_Stepper instance
  AF_Stepper* motorL; // pointer to left AF_Stepper instance
};

// SensorContext groups fields required by sensor/scan helpers. It
// includes the calibration table, the analog pin used by the sensor,
// references to the servos/steppers used during scanning, and items
// needed for angle->steps computation.
struct SensorContext {
  const int* tableEntries; // pointer to number of entries in table
  const int* mVinterval; // pointer to mV bucket interval used by table
  float* sensorDistance; // pointer to calibration distance array
  int sensorPin; // analog pin number used by the distance sensor

  const float* WHEEL_RADIUS_CM; // pointer to wheel radius (cm)
  const float* WHEELS_AXIS_CM; // pointer to wheel axis (cm)

  AF_Stepper* motorR; // pointer to right motor (used by step helpers)
  AF_Stepper* motorL; // pointer to left motor
  Servo_ESP8266* sensorServo; // pointer to servo used for scanning
  bool* out; // pointer to optional external flag

  // step resolution used by computeSteps (kept here for convenience)
  int* stepperResolution;
  int* stepToMicrostep;
};

// No global instances are declared in this header. The main sketch
// creates the concrete MovementContext and SensorContext objects and
// passes pointers to modules that need them.

#endif // ROBOT_SHARED_H
