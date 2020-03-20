#include <math.h>
#include <Arduino.h>
#include "PID.h"

#define ANGLE_PITCH_KP 	0
#define ANGLE_PITCH_KI 	0
#define ANGLE_PITCH_KD 	0
#define ANGLE_ROLL_KP 	0
#define ANGLE_ROLL_KI 	0
#define ANGLE_ROLL_KD 	0
#define ANGLE_YAW_KP 	8.0
#define ANGLE_YAW_KI 	7
#define ANGLE_YAW_KD 	0.01

#define RATE_PITCH_KP 	0.6
#define RATE_PITCH_KI 	0.6
#define RATE_PITCH_KD 	0.001
#define RATE_ROLL_KP 	0.6
#define RATE_ROLL_KI  	0.6
#define RATE_ROLL_KD  	0.001


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
	
	yawRatePID.KP = ANGLE_YAW_KP;
	yawRatePID.KI = ANGLE_YAW_KI;
	yawRatePID.KD = ANGLE_YAW_KD;
	rollRatePID.KP = RATE_ROLL_KP;
	rollRatePID.KI = RATE_ROLL_KI;
	rollRatePID.KD = RATE_ROLL_KD;
	pitchRatePID.KP = RATE_PITCH_KP;
	pitchRatePID.KI = RATE_PITCH_KI;
	pitchRatePID.KD = RATE_PITCH_KD;


	rollAnglePID.lastTimeStamp = micros();
	pitchAnglePID.lastTimeStamp = micros();
	yawRatePID.lastTimeStamp = micros();
}

void PIDController::calCurrentPID(PID *PID, float target, float measure, float dt) {
	dt /= 1000000;
	PID->error = target - measure; // 计算误差


	// 积分分离
	if (abs(PID->error) < 10 && abs(PID->error) > 1) {
		PID->integ += PID->error * dt; // 计算误差积分	
	}

	
	PID->deriv = (dt != 0) ? ((PID->error - PID->lastError) / dt) : 0; // 计算误差微分

	if (abs(PID->KI * PID->integ) > ERRORCORR_MAX_I) {
		PID->integ = (PID->integ > 0) ? ERRORCORR_MAX_I / PID->KI : -ERRORCORR_MAX_I / PID->KI;
	}

	if (abs(PID->KD * PID->deriv) > ERRORCORR_MAX_D) {
		PID->deriv = (PID->deriv > 0) ? ERRORCORR_MAX_D / PID->KD : -ERRORCORR_MAX_D / PID->KD;
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

void PIDController::calCurrentYawRatePID(float measureYaw, float targetYaw) {
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (yawRatePID.lastTimeStamp > 0) ? (t - yawRatePID.lastTimeStamp): 0;
	yawRatePID.lastTimeStamp = t;

	calCurrentPID(&yawRatePID, targetYaw, measureYaw, dt);
}


void PIDController::calCurrentRollRatePID(float measureRollRate, float targetRollRate){
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (rollRatePID.lastTimeStamp > 0) ? (t - rollRatePID.lastTimeStamp) : 0;
	rollRatePID.lastTimeStamp = t;

	calCurrentPID(&rollRatePID, targetRollRate, measureRollRate, dt);
}

void PIDController::calCurrentPitchRatePID(float measurePitchRate, float targetPitchRate) {
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (pitchRatePID.lastTimeStamp > 0) ? (t - pitchRatePID.lastTimeStamp) : 0;
	pitchRatePID.lastTimeStamp = t;

	calCurrentPID(&pitchRatePID, targetPitchRate, measurePitchRate, dt);
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
		return rollRatePID.output;
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
		return pitchRatePID.output;
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
		return 0;
		break;
	case RATE:
		return yawRatePID.output;
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
		return rollRatePID.error;
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
		return rollRatePID.integ;
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
		return rollRatePID.deriv;
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
		return pitchRatePID.error;
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
		return pitchRatePID.integ;
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
		return pitchRatePID.deriv;
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
		return 0;
		break;
	case RATE:
		return yawRatePID.error;
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
		return 0;
		break;
	case RATE:
		return yawRatePID.integ;
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
		return 0;
		break;
	case RATE:
		return yawRatePID.deriv;
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
	cleanData(&rollRatePID);
}

void PIDController::cleanPitchPIDData() {
	cleanData(&pitchAnglePID);
	cleanData(&pitchRatePID);
}

void PIDController::cleanYawPIDData() {
	cleanData(&yawRatePID);
	cleanData(&yawRatePID);
}
