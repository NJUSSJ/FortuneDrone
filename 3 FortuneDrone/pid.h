typedef struct {
	float KP;
	float KI;
	float KD;
	float error;
	float lastError;
	float integ;
	float deriv;
	float output;
	long lastTimeStamp;
} PID;

class PIDController {
private:
	PID rollAnglePID;
	PID pitchAnglePID;
	PID yawAnglePID;

	void calCurrentPID(PID *PID, float target, float measure, float dt);
	void cleanData(PID *PID);
	float minMax(float value, float min, float max);

public:
	enum PIDKind {
		ANGLE,
		RATE
	};
	PIDController();
	void calCurrentRollAnglePID(float measureRoll, float targetRoll);
	void calCurrentPitchAnglePID(float measurePitch, float targetPitch);
	void calCurrentYawAnglePID(float measureYaw, float targetYaw);

	/* ===================================================================
		* 
		*  PID 修正值
		* 
		* ===================================================================
		*/
	float getRollCorrect(PIDKind pidKind);
	float getPitchCorrect(PIDKind pidKind);
	float getYawCorrect(PIDKind pidKind);


	/* =============================================================
	* 
	* roll 相关信息
	* 
	* =============================================================
	*/
	float getRollError(PIDKind kind);
	float getRollInteg(PIDKind kind);
	float getRollDeriv(PIDKind kind);

	/* =============================================================
		* 
		* pitch 相关信息
		* 
		* =============================================================
		*/
	float getPitchError(PIDKind kind);
	float getPitchInteg(PIDKind kind);
	float getPitchDeriv(PIDKind kind);

	/* =============================================================
		* 
		* yaw 相关信息
		* 
		* =============================================================
		*/
	float getYawError(PIDKind kind);
	float getYawInteg(PIDKind kind);
	float getYawDeriv(PIDKind kind);

	void cleanRollPIDData();
	void cleanPitchPIDData();
	void cleanYawPIDData();
};
