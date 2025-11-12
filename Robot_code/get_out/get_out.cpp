#include "GetOut.h"
#include <math.h>

GetOut::GetOut(SensorContext* ctx) : ctx(ctx) {}

// Interpolate calibration table to convert sensor mV reading into
// distance in centimeters. The table stores decreasing distances for
// increasing voltage buckets; simple linear interpolation is used.
float GetOut::getSensorDistance(float mV) {
	int te = *ctx->tableEntries;
	int mvint = *ctx->mVinterval;
	// If measured mV is beyond table range, return the last entry
	if (mV > mvint * te - 1) return ctx->sensorDistance[te - 1];
	int index = mV / mvint;
	float frac = fmod(mV, (float)mvint) / (float)mvint;
	return ctx->sensorDistance[index] - ((ctx->sensorDistance[index] - ctx->sensorDistance[index + 1]) * frac);
}

// Read the analog sensor pin and convert raw ADC value to mV, then
// map to a distance using the calibration table.
float GetOut::getDistanceFromSensor() {
	const float referenceMv = 1000.0; // ADC reference used to compute mV
	float val = analogRead(ctx->sensorPin);
	float mV = (val * referenceMv) / 1023.0; // map 0..1023 to 0..referenceMv
	return getSensorDistance(mV);
}

// Compute micro-steps required to rotate the robot by `angle` degrees
// using wheel geometry parameters from the context. The formula is a
// rearrangement of the arc length and wheel circumference conversion.
long GetOut::computeSteps(float angle) {
	return (long)(angle * (*ctx->stepperResolution) * (*ctx->stepToMicrostep) * (*ctx->WHEELS_AXIS_CM) / ((*ctx->WHEEL_RADIUS_CM) * 720.0));
}

// Step the motors one micro-step in the requested direction. This
// helper centralizes the mapping between direction characters and
// the AF_Stepper API for clarity.
void GetOut::stepMotors(char dir) {
	AF_Stepper* r = ctx->motorR;
	AF_Stepper* l = ctx->motorL;
	if (dir == 'F') { r->step(1, FORWARD, SINGLE); l->step(1, FORWARD, SINGLE); }
	if (dir == 'B') { r->step(1, BACKWARD, SINGLE); l->step(1, BACKWARD, SINGLE); }
	if (dir == 'R') { r->step(1, BACKWARD, SINGLE); l->step(1, FORWARD, SINGLE); }
	if (dir == 'L') { r->step(1, FORWARD, SINGLE);  l->step(1, BACKWARD, SINGLE); }
}
