#include <Wire.h>                       // Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>      
#include "pid.h"
#include "mpu.h"
#include "irreceiver-config.h"


/* =============================================================
 *         four axis drone                    
 * 		 ↶				 ↷                     ↶		↷
 *       3                4						+   yaw   -
 *         -  x(roll)  +
 *                ↑
 *                |
 *                |             +
 *                L--------→ y(pitch)
 *                              -
 *       2                1
 *       ↻               ↺
 * ============================================================
 */
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 
// 根据 roll 和 pitch 对应的不同，修改 ESC_Ctrl
// 
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define STATE_INIT		0
#define STATE_FLYPRE	1	// 飞行准备阶段（MPU 归 0，风扇加速）
#define STATE_FLY   	2	// 悬停状态
#define STATE_STOP  	3	
#define STATE_MEAS		4	// 测量 RPY 角模式

int state = STATE_INIT;

/* ============================================================
 * 
 * 						   		ESC_Ctrl
 * 
 * ============================================================
 */
#define ESC_INIT_PWM	1000
#define ESC_MIN_PWM		1000
#define ESC_MAX_PWM		2000

int ESC_PWM[4] = { ESC_INIT_PWM, ESC_INIT_PWM, ESC_INIT_PWM, ESC_INIT_PWM };
unsigned long zero_timer, timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;


/* ============================================================
 * 
 * 						   		PID
 * 
 * ============================================================
 */
PIDController pidController;
float targetRoll = 0.0, targetPitch = 0.0, targetYaw = 0.0;
float roll_level_adjust, pitch_level_adjust;


/* ============================================================
 * 
 * 							   MPU
 * 
 * ============================================================
 */
MPU mpu;
float roll = 0.0, pitch = 0.0, roll_acc = 0.0, pitch_acc = 0.0, gz_input, gy_input, gx_input;
long ax, ay, az, acc_total;
float gx, gy, gz;
float roll_cal = 0, pitch_cal = 0;

float RAve = 0.0, PAve = 0.0;
int measureCount = 0, count = 0;
const int measureTime = 1000;



// 用于保存当前时间毫秒值
long loop_timer;

float batteryVoltage = 0;
bool setGyroAngle;

void setup() {
	Serial.begin(57600);
	Serial.println("\n===================================\n开始初始化\n");


	Wire.begin();                                                             // Start the I2C as master.
	TWBR = 24;                                                                // Set the I2C clock speed to 400kHz.


	// 初始化MPU
	int status = mpu.begin();
	if(status < 0) {
		Serial.println("MPU initialized unsuccessfully");
		while (1){}	
	}

	mpu.setGyroRange(MPU::GYRO_RANGE_500DPS);
	mpu.setAccRange(MPU::ACC_RANGE_8G);
	mpu.setDLPFBandwidth(MPU::DLPF_BANDWIDTH_41HZ);
	delay(10);

	//计算 MPU gyro 误差
	mpu.calibrateGyro();
	measureRPError();

	Serial.print("gx_offset: "); Serial.print(mpu.getGyroX_Offset()); Serial.print("  ");
	Serial.print("gy_offset: "); Serial.print(mpu.getGyroY_Offset()); Serial.print("  ");
	Serial.print("gz_offset: "); Serial.print(mpu.getGyroZ_Offset()); Serial.print("  "); Serial.println("");

	//初始化ESC
	// Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
	// port registers https://www.arduino.cc/en/Reference/PortManipulation
    // port in mega 2560 http://harperjiangnew.blogspot.com/2013/05/arduino-port-manipulation-on-mega-2560.html
	// Set pins 4, 5, 6, 7 as outputs
    DDRD |= B11110000;
	// Set digital port 12, 13 as output.
	DDRB |= B00110000;

  	init_ESC();
  
	// 初始化遥控
	initAttach_channel();        
	
	Serial.println("初始化完毕\n===================================\n");

  	delay(1000);

	//Load the battery voltage to the battery_voltage variable.
  	//65 is the voltage compensation for the diode.
  	//12.6V equals ~5V @ Analog 0.
  	//12.6V equals 1023 analogRead(0).
  	//1260 / 1023 = 1.2317.
  	//The variable battery_voltage holds 1050 if the battery voltage is 10.5V. 

	batteryVoltage = (analogRead(0) + 65) * 1.2317;
	
  	loop_timer = micros();
}

void loop() {

	mpu.readSensor();
	
	ax = mpu.getAccX();
	ay = mpu.getAccY();
	az = mpu.getAccZ();
	
	gx = mpu.getGyroX();
	gy = mpu.getGyroY();
	gz = mpu.getGyroZ();
	
	gz_input = (gz_input * 0.7) + ((gz / 65.5) * 0.3);
	gy_input = (gy_input * 0.7) + ((gy / 65.5) * 0.3);
	gx_input = (gx_input * 0.7) + ((gx / 65.5) * 0.3);


	// more info @see https://www.jianshu.com/p/1c208df3cfc8
	// MPU6050 数据寄存器是一个 16 位的，最高位是符号位，故而数据寄存器的输出范围 -7FFF~7FFF,也就是 -32767~32767。
	// 所以 gyro LBS（最小有效位）= 32767 / range = 32767 / 500 = 65.5

	// 0.0000611 = 1 / 250Hz * 65.5
	// integration of gyro
	roll += gx_input * 0.004;
	pitch += gy_input * 0.004;

	// 0.000001066 = 0.004 * (3.142(PI) / 180deg) 
	// arduino sin function is based on radius
	// pitch transfer to roll when yaw change
	roll += pitch * sin(gz_input * 0.00006981);
	pitch -= roll * sin(gz_input * 0.00006981);

	// angle will drift
	acc_total = sqrt((ax*ax)+(ay*ay)+(az*az));

	if (abs(ay) < acc_total) {
		// Calculate the roll accelerator angle.
		roll_acc = asin((float)ay / acc_total) * 57.296;
	}

	if (abs(ax) < acc_total) {
		pitch_acc = asin((float)ax / acc_total) * -57.296;
	}

	// Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
	roll_acc -= (RAve); pitch_acc -= (PAve); 

  	if (setGyroAngle) {
    // Correct the drift of the gyro roll angle with the accelerometer roll angle.
    	roll = roll * 0.9996 + roll_acc * 0.0004;   
    	pitch = pitch * 0.9996 + pitch_acc * 0.0004;
  	} else {
    	roll = roll_acc;
    	pitch = pitch_acc;
    	setGyroAngle= true; 
  	}
  
	roll_level_adjust = roll * 15;
	pitch_level_adjust = pitch * 15;
	
	if (state == STATE_INIT) {
		count++;
		if (count >= 50){
			Serial.print("roll: ");	Serial.print(roll, 2);printSpace(roll);
			Serial.print("pitch: ");Serial.print(pitch, 2);printSpace(pitch);
			Serial.println("");
			count = 0;
		}
		// 遥控器左位于中下， 右边位于中左， 开启飞行
		if (CHInMin(INDEX_PWM) && CHInCenter(INDEX_YAW) && CHInMin(INDEX_ROLL) && CHInCenter(INDEX_PITCH)) {
			state = STATE_FLY;
      		setGyroAngle = false;
			pidController.cleanRollPIDData();
			pidController.cleanPitchPIDData();
			pidController.cleanYawPIDData();
		} else if (CHInMin(INDEX_PWM) && CHInCenter(INDEX_YAW) && CHInCenter(INDEX_ROLL) && CHInMin(INDEX_PITCH) ) {
			// 遥控器左边位于中下，右边位于中下，测量 Roll、pitch 以及 gyro 误差
			state = STATE_MEAS;
			Serial.println("\n===================================\n开始测量 gyro 误差\n");
			mpu.calibrateGyro();
			Serial.print("gx_offset: "); Serial.print(mpu.getGyroX_Offset()); Serial.println("");
			Serial.print("gy_offset: "); Serial.print(mpu.getGyroY_Offset()); Serial.println("");
			Serial.print("gz_offset: "); Serial.print(mpu.getGyroZ_Offset()); Serial.println("");
			Serial.println("\n=================\n开始测量 Roll, pitch 误差\n");
		}
    	ESC_Ctrl(1000, 0, 0, 0);
	} else if (state == STATE_FLY) {
		// 遥控器左边位于中下，右边位于中右，停止飞行
		if (CHInMin(INDEX_PWM) && CHInCenter(INDEX_YAW) && CHInMax(INDEX_ROLL) && CHInCenter(INDEX_PITCH)) {
			state = STATE_INIT;
      		setGyroAngle = false;
			ESC_Ctrl(1000, 0, 0, 0);
		} else {
			// 根据 channel 值计算出人为设定的 roll、pitch、yaw
			if (channels[INDEX_ROLL] > 1508)
			{
				targetRoll = channels[INDEX_ROLL] - 1508;
			} else if (channels[INDEX_ROLL] < 1492) 
			{
				targetRoll = channels[INDEX_ROLL] - 1492;
			}
			
			if (channels[INDEX_PITCH] > 1508)
			{
				targetPitch = channels[INDEX_PITCH] - 1508;
			} else if (channels[INDEX_PITCH] < 1492) 
			{
				targetPitch = channels[INDEX_PITCH] - 1492;
			}
			
			targetRoll = (0 - roll_level_adjust) / 3;
			targetPitch = (0 - pitch_level_adjust) / 3;
			targetYaw = 0;

			cal_RATE_PID();

			//The battery voltage is needed for compensation.
  			//A complementary filter is used to reduce noise.
  			//0.09853 = 0.08 * 1.2317.
			batteryVoltage = batteryVoltage * 0.92 + (analogRead(0) + 65) * 0.09853;

			float rollCORR = pidController.getRollCorrect(PIDController::RATE);
			float pitchCORR = pidController.getPitchCorrect(PIDController::RATE);
			float yawCORR = pidController.getYawCorrect(PIDController::RATE);

			if (channels[INDEX_PWM] > 1020) {
				ESC_Ctrl(channels[INDEX_PWM], rollCORR, pitchCORR, yawCORR);
			} else {
				ESC_Ctrl(1000, 0, 0, 0);
				pidController.cleanRollPIDData();
				pidController.cleanPitchPIDData();
				pidController.cleanYawPIDData();
			}

			count++;
			if (count >= 25) {
				outputMidVal(roll, pitch, gz_input, gy_input, gx_input, rollCORR, pitchCORR, yawCORR, channels[INDEX_PWM], targetRoll, targetPitch, targetYaw);
				count = 0;
			}
		}
	} else if (state == STATE_MEAS) {
		measureRPError();
		setGyroAngle = false;
		ESC_Ctrl(1000, 0, 0, 0);
	}
	 // 控制解算采样频率为250Hz
	while (micros() - loop_timer < 4000);
    loop_timer = micros();
}


void cal_ANGLE_PID(){
	pidController.calCurrentRollAnglePID(roll, targetRoll);
	pidController.calCurrentPitchAnglePID(pitch, targetPitch);
	pidController.calCurrentYawRatePID(gz_input, targetYaw);
	
}

void cal_RATE_PID() {
	pidController.calCurrentRollRatePID(gx_input, targetRoll);
	pidController.calCurrentPitchRatePID(gy_input, targetPitch);
	pidController.calCurrentYawRatePID(gz_input, targetYaw);
}

bool CHInMin(int index){ return channels[index] < minCH2Angle_N; }
bool CHInCenter(int index){ return maxCH2Angle_N < channels[index] && channels[index] < minCH2Angle_P; }
bool CHInMax(int index){ return channels[index] > maxCH2Angle_P; }

// 输出PWM，控制电机
void ESC_Ctrl(int throttle, float rollCORR, float pitchCORR, float yawCORR) {
	if (throttle > 1500) throttle = 1500;
	ESC_PWM[0] = throttle - rollCORR + pitchCORR - yawCORR;
	ESC_PWM[1] = throttle + rollCORR + pitchCORR + yawCORR;
	ESC_PWM[2] = throttle + rollCORR - pitchCORR - yawCORR;
	ESC_PWM[3] = throttle - rollCORR - pitchCORR + yawCORR;

	if (batteryVoltage < 1240 && batteryVoltage > 800) {
		ESC_PWM[0] += ESC_PWM[0] * ((1240 - batteryVoltage) / (float)3500);
		ESC_PWM[1] += ESC_PWM[1] * ((1240 - batteryVoltage) / (float)3500);
		ESC_PWM[2] += ESC_PWM[2] * ((1240 - batteryVoltage) / (float)3500);
		ESC_PWM[3] += ESC_PWM[3] * ((1240 - batteryVoltage) / (float)3500);
	}

	zero_timer = micros();
	PORTD |= B11110000;                                            //Set port 4, 5, 6 and 7 high at once
	timer_channel_1 = ESC_PWM[0] + zero_timer;                          //Calculate the time when digital port 4 is set low.
	timer_channel_2 = ESC_PWM[1] + zero_timer;                          //Calculate the time when digital port 5 is set low.
	timer_channel_3 = ESC_PWM[2] + zero_timer;                          //Calculate the time when digital port 6 is set low.
	timer_channel_4 = ESC_PWM[3] + zero_timer;                          //Calculate the time when digital port 7 is set low.

	// ensure this code match motor of physical model
	while (PORTD >= 16) {                                            //Execute the loop until digital port 4 to 7 is low.
		esc_loop_timer = micros();                                   //Check the current time.
		if(timer_channel_1 <= esc_loop_timer) PORTD &= B11101111;     //When the delay time is expired, digital port 4 is set low.
		if(timer_channel_2 <= esc_loop_timer) PORTD &= B11011111;     //When the delay time is expired, digital port 5 is set low.
		if(timer_channel_3 <= esc_loop_timer) PORTD &= B10111111;     //When the delay time is expired, digital port 6 is set low.
		if(timer_channel_4 <= esc_loop_timer) PORTD &= B01111111;     //When the delay time is expired, digital port 7 is set low.
	}
}

void measureRPError() {
	for (measureCount; measureCount < measureTime; measureCount++)
	{
		mpu.readSensor();
    	ax = mpu.getAccX();
    	ay = mpu.getAccY();
    	az = mpu.getAccZ();

		acc_total = sqrt((ax*ax)+(ay*ay)+(az*az));

		if (abs(ay) < acc_total) {
		RAve += asin((float)ay / acc_total) * 57.296;
		}
	
		if (abs(ax) < acc_total) {
		PAve += asin((float)ax / acc_total) * -57.296;
		}
	}
	RAve /= measureTime;
	PAve /= measureTime;
	Serial.print("    RollAccError: "); Serial.println(RAve);
	Serial.print("    PitchAccError: "); Serial.println(PAve);
	Serial.println("\n测量完毕\n===================================\n");
	delay(2000);
	state = STATE_INIT;
	measureCount = 0;
}

void output_MPU(float ax, float ay, float az, float gx, float gy, float gz){
	Serial.print("ax: "); Serial.print(ax); Serial.print("  ");
	Serial.print("ay: "); Serial.print(ay); Serial.print("  ");
	Serial.print("az: "); Serial.print(az); Serial.print("  "); Serial.print(" \t");
	Serial.print("gx: "); Serial.print(gx); Serial.print("  ");
	Serial.print("gy: "); Serial.print(gy); Serial.print("  ");
	Serial.print("gz: "); Serial.print(gz); Serial.print("  "); Serial.println("");
}

// 输出中间值
void outputMidVal(float roll, float pitch, float gz, float gy, float gx, float rollCORR, float pitchCORR, float yawCORR, int PWM, float targetRoll, float targetPitch, float targetYaw){
	float RError = pidController.getRollError(PIDController::RATE);
	float RInteg = pidController.getRollInteg(PIDController::RATE);
	float RDeriv = pidController.getRollDeriv(PIDController::RATE);
	float PError = pidController.getPitchError(PIDController::RATE);
	float PInteg = pidController.getPitchInteg(PIDController::RATE);
	float PDeriv = pidController.getPitchDeriv(PIDController::RATE);
	float YError = pidController.getYawError(PIDController::RATE);
	float YInteg = pidController.getYawInteg(PIDController::RATE);
	float YDeriv = pidController.getYawDeriv(PIDController::RATE);

	Serial.print("PWM: ");		Serial.print(channels[INDEX_PWM]);
	if (channels[INDEX_PWM] < 1000){ Serial.print("   "); } else { Serial.print("  "); }

	Serial.print("R: ");		Serial.print(roll, 2);		printSpace(roll);
  	Serial.print("gx: ");    Serial.print(gx, 2);    printSpace(gx);
	Serial.print("RError: ");	Serial.print(RError, 2);	printSpace(RError);
	Serial.print("RInteg: ");	Serial.print(RInteg, 2);	printSpace(RInteg);
	Serial.print("RDeriv: ");	Serial.print(RDeriv, 2);	printSpace(RDeriv);
	Serial.print("RCORR: ");	Serial.print(rollCORR, 2);	Serial.print("   ");

	Serial.print("P: ");		Serial.print(pitch, 2);		printSpace(pitch);
  	Serial.print("gy: ");    Serial.print(gy, 2);    printSpace(gy);
	Serial.print("PError: "); 	Serial.print(PError, 2);	printSpace(PError);
	Serial.print("PInteg: ");	Serial.print(PInteg, 2);	printSpace(PInteg);
	Serial.print("PDeriv: ");	Serial.print(PDeriv, 2);	printSpace(PDeriv);
	Serial.print("PCORR: ");	Serial.print(pitchCORR, 2);	Serial.print("   ");

	Serial.print("gz: ");		Serial.print(gz, 2);		printSpace(gz);
	Serial.print("YError: ");	Serial.print(YError, 2);	printSpace(YError);
	// Serial.print("YInteg: ");	Serial.print(YInteg, 2);	printSpace(YInteg);
	// Serial.print("YDeriv: ");	Serial.print(YDeriv, 2);	printSpace(YDeriv);
	Serial.print("YCORR: ");	Serial.print(yawCORR, 2);	Serial.print("   ");
	
	Serial.println("");
}

void printSpace(float val){
	if (val < 0){
		if (abs(val) >= 100){ Serial.print(" "); }
		else if (abs(val) >= 10) { Serial.print("  "); }
		else {Serial.print("   ");}
	}
	else {
		if (abs(val) >= 100){ Serial.print("  "); }
		else if (abs(val) >= 10) { Serial.print("   "); }
		else {Serial.print("    ");}
	}
}


void init_ESC(){
  for (int i = 0; i < 1250 ; i++){
    // Set digital poort 4, 5, 6 and 7 high.
    PORTD |= B11110000;
    delayMicroseconds(1000);
    // Set digital poort 4, 5, 6 and 7 low.
    PORTD &= B00001111;
    delayMicroseconds(3000);
  }
}
