#include "Movement.h"
#include <math.h>

// Movement implementation
// This module converts high-level commands (angles, distances)
// into low-level step counts and updates the movement-related
// state stored in MovementContext. The computations use basic
// wheel geometry and the provided steps-per-revolution function.

Movement::Movement(MovementContext* ctx) : ctx(ctx) {}

// Convert an in-place rotation (degrees) into the number of micro-steps
// each wheel needs to perform. The approach is:
//  1) compute the arc length each wheel travels for the rotation
//  2) convert arc length to wheel rotations
//  3) convert rotations to micro-steps using stepsPerRevMicro()
long Movement::degreesToSteps(float angleDeg) {
	float angle = fabs(angleDeg);
	// arc length per wheel in mm
	float arc_mm = (PI * (*ctx->WHEEL_BASE_MM)) * (angle / 360.0);
	// wheel rotations required
	float rotations = arc_mm / (PI * (*ctx->WHEEL_DIAMETER_MM));
	// convert rotations to micro-steps
	float steps = rotations * (float)ctx->stepsPerRevMicro();
	return (long)(steps + 0.5);
}

// Convert a forward distance (cm) to micro-steps.
long Movement::distanceCmToSteps(float distCm) {
	float dist_mm = distCm * 10.0;
	float rotations = dist_mm / (PI * (*ctx->WHEEL_DIAMETER_MM));
	float steps = rotations * (float)ctx->stepsPerRevMicro();
	return (long)(steps + 0.5);
}

// Compute shortest signed angle difference between two headings.
float Movement::shortestAngle(float fromDeg, float toDeg) {
	float d = toDeg - fromDeg;
	while (d > 180.0) d -= 360.0;
	while (d < -180.0) d += 360.0;
	return d;
}

// Process a remote command that requests rotation and/or forward
// motion. This method writes into the MovementContext so the
// non-blocking main loop will execute the steps.
void Movement::processRemoteCommand(float distCm, float angleDeg, float out) {
	// Handle special "out" flag: request a 180-degree turn.
	// Only trigger on the rising edge of the flag and if not
	// already turning and is moving or rotating.
	if (out == 1 and currentMode != IDLE and not lastOutFlag and not getoutMode) {
		lastOutFlag = true; 
		getoutMode = true;
		Serial.println("[REMOTE] Get-out command received.");
		// Special remote flag: request a 180-degree turn.
		Serial.print(out);
		currentMode = TURN_180;
		return;
	}

	lastOutFlag = (out == 1);

	if (getoutMode) {
		// Ignore other commands while in get-out mode.
		Serial.println("[REMOTE] Ignoring command while in get-out mode.");
		return;
	}

	// Store requested forward distance
	*ctx->desiredDistanceCm = distCm;

	// Requested rotation (signed degrees). Compute target absolute
	// heading and normalize to [0,360).
	float dAng = angleDeg;
	*ctx->desiredHeading = fmod((*ctx->currentHeading) + dAng, 360.0);
	if (*ctx->desiredHeading < 0) *ctx->desiredHeading += 360.0;

	// Determine rotation steps and prepare wheel targets/directions.
	long rotSteps = degreesToSteps(dAng);
	if (rotSteps > 0) {
		// Configure an in-place rotation: wheels move in opposite
		// directions so the robot pivots around its center.
		if (dAng > 0) { *ctx->dirR = BACKWARD; *ctx->dirL = FORWARD; }
		else { *ctx->dirR = FORWARD; *ctx->dirL = BACKWARD; }
		*ctx->targetStepsR = rotSteps; *ctx->targetStepsL = rotSteps;
		*ctx->doneStepsR = 0; *ctx->doneStepsL = 0;
		currentMode = ROTATING;
		Serial.print("[REMOTE] Prepared ROTATE steps="); Serial.println(rotSteps);
		return;
	}

	// No rotation needed; prepare forward movement instead.
	long moveSteps = distanceCmToSteps(*ctx->desiredDistanceCm);
	if (moveSteps > 0) {
		*ctx->dirR = FORWARD; *ctx->dirL = FORWARD;
		*ctx->targetStepsR = moveSteps; *ctx->targetStepsL = moveSteps;
		*ctx->doneStepsR = 0; *ctx->doneStepsL = 0;
		currentMode = MOVING;
		Serial.print("[REMOTE] Prepared MOVE steps="); Serial.println(moveSteps);
	} else {
		currentMode = IDLE;
	}
}
