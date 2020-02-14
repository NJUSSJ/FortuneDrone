#include <math.h>
#include <Arduino.h>
#include "PID.h"

#define ANGLE_PITCH_KP 	8.0
#define ANGLE_PITCH_KI 	5.0
#define ANGLE_PITCH_KD 	1.5
#define ANGLE_ROLL_KP 	8.0
#define ANGLE_ROLL_KI 	5.0
#define ANGLE_ROLL_KD 	1.5
#define ANGLE_YAW_KP 	0
#define ANGLE_YAW_KI 	0
#define ANGLE_YAW_KD 	0

#define ERRORCORR_MAX	250.0
#define ERRORCORR_MAX_P	250.0
#define ERRORCORR_MAX_I	35.0
#define ERRORCORR_MAX_D	100.0

PIDController::PIDController() {
	rollAnglePID.KP = ANGLE_ROLL_KP;
	rollAnglePID.KI = ANGLE_ROLL_KI;
	rollAnglePID.KD = ANGLE_ROLL_KD;
	pitchAnglePID.KP = ANGLE_PITCH_KP;
	pitchAnglePID.KI = ANGLE_PITCH_KI;
	pitchAnglePID.KD = ANGLE_PITCH_KD;
	yawAnglePID.KI = ANGLE_YAW_KP;
	yawAnglePID.KI = ANGLE_YAW_KI;
	yawAnglePID.KD = ANGLE_YAW_KD;

	rollAnglePID.lastTimeStamp = 0;
	pitchAnglePID.lastTimeStamp = 0;
	yawAnglePID.lastTimeStamp = 0;
}

void PIDController::calCurrentPID(PID *PID, float target, float measure, float dt) {
	dt /= 1000000;
	PID->error = target - measure; // 计算误差

	if (abs(PID->error) < 20.0 && abs(PID->error) > 1.0) {
		PID->integ += PID->error; // 计算误差积分
	}

	PID->deriv = (dt != 0) ? (PID->error - PID->lastError) / dt : 0; // 计算误差微分

	if (abs(PID->KI * PID->integ) > ERRORCORR_MAX_I) {
		PID->integ = (PID->integ > 0) ? ERRORCORR_MAX_I / PID->KI : -ERRORCORR_MAX_I / PID->KI;
	}

	PID->output = (PID->KP * PID->error) + (PID->KI * PID->integ) + (PID->KD * PID->deriv);
	PID->output = minMax(PID->output, -ERRORCORR_MAX, ERRORCORR_MAX);
	PID->lastError = PID->error;
}

float PIDController::minMax(float value, float min, float max) {
	if (value < min) return min;
	if (value > max) return max;
	return value;
}


void PIDController::calCurrentRollAnglePID(float measureRoll, float targetRoll) {
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (rollAnglePID.lastTimeStamp > 0) ? (t - rollAnglePID.lastTimeStamp): 0;
	rollAnglePID.lastTimeStamp = t;

	calCurrentPID(&rollAnglePID, targetRoll, measureRoll, dt);
}

void PIDController::calCurrentPitchAnglePID(float measurePitch, float targetPitch) {
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (pitchAnglePID.lastTimeStamp > 0) ? (t - pitchAnglePID.lastTimeStamp): 0;
	pitchAnglePID.lastTimeStamp = t;

	calCurrentPID(&pitchAnglePID, targetPitch, measurePitch, dt);
}

void PIDController::calCurrentYawAnglePID(float measureYaw, float targetYaw) {
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (yawAnglePID.lastTimeStamp > 0) ? (t - yawAnglePID.lastTimeStamp): 0;
	yawAnglePID.lastTimeStamp = t;

	calCurrentPID(&yawAnglePID, targetYaw, measureYaw, dt);
}

/* ===================================================================
	* 
	* PID 修正值
	* 
	* ===================================================================
	*/

float PIDController::getRollCorrect(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return rollAnglePID.output;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	}
}

float PIDController::getPitchCorrect(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return pitchAnglePID.output;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	}
}

float PIDController::getYawCorrect(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return yawAnglePID.output;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	}
}

/* =============================================================
	* 
	* roll 相关信息
	* 
	* =============================================================
	*/
float PIDController::getRollError(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return rollAnglePID.error;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	}
}

float PIDController::getRollInteg(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return rollAnglePID.integ;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	}
}

float PIDController::getRollDeriv(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return rollAnglePID.deriv;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	}
}

/* =============================================================
	* 
	* pitch 相关信息
	* 
	* =============================================================
	*/

float PIDController::getPitchError(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return pitchAnglePID.error;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	} 
}

float PIDController::getPitchInteg(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return pitchAnglePID.integ;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	} 
}

float PIDController::getPitchDeriv(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return pitchAnglePID.deriv;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	} 
}

/* =============================================================
	* 
	* yaw 相关信息
	* 
	* =============================================================
	*/
float PIDController::getYawError(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return yawAnglePID.error;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	} 
}

float PIDController::getYawInteg(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return yawAnglePID.integ;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	} 
}

float PIDController::getYawDeriv(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return yawAnglePID.deriv;
		break;
	case RATE:
		return 0;
		break;
	default:
		return 0;
		break;
	} 
}

void PIDController::cleanData(PID *PID) {
	PID->error = 0.0;
	PID->lastError = 0.0;
	PID->integ = 0.0;
	PID->deriv = 0.0;
	PID->output = 0.0;
	PID->lastTimeStamp = micros();
}

void PIDController::cleanRollPIDData() {
	cleanData(&rollAnglePID);
}

void PIDController::cleanPitchPIDData() {
	cleanData(&pitchAnglePID);
}

void PIDController::cleanYawPIDData() {
	cleanData(&yawAnglePID);
}
