#ifndef ROBOT_SHARED_H
#define ROBOT_SHARED_H

#include "robot_state.h"
#include <DMotor_mod.h>
#include <Servo_ESP8266.h>

// Centralized container that points to the true globals defined in Robot_code.ino
// This lets other translation units access shared items via a single extern.
//
// Rationale: Arduino sketches often mix .ino and .cpp files. Rather than
// proliferar múltiples `extern` para variables globales en cada header, el
// struct `RobotShared` agrupa punteros (o valores) hacia los elementos
// realmente definidos en `Robot_code.ino`. Los módulos consumidores leen/escriben
// los campos de `robot` (por ejemplo `robot.desiredDistanceCm`) para acceder
// al estado compartido de forma clara y centralizada.
typedef long (*StepsFunc)();

struct RobotShared {
  StepsFunc stepsPerRevMicro; // function pointer
  const float* WHEEL_BASE_MM;
  const float* WHEEL_DIAMETER_MM;
  float* currentHeading;
  float* desiredHeading;
  float* desiredDistanceCm;
  long* targetStepsR;
  long* targetStepsL;
  long* doneStepsR;
  long* doneStepsL;
  int* dirR;
  int* dirL;

  int* stepperResolution;
  int* stepToMicrostep;

  const int* tableEntries;
  const int* mVinterval;
  float* sensorDistance; // points to first element of array
  int sensorPin; // store pin number by value (sensorPin may be a macro)

  const float* WHEEL_RADIUS_CM;
  const float* WHEELS_AXIS_CM;

  AF_Stepper* motorR;
  AF_Stepper* motorL;
  Servo_ESP8266* sensorServo;
};

// Single extern instance (defined/initialized in Robot_code.ino)
extern RobotShared robot;

#endif // ROBOT_SHARED_H
