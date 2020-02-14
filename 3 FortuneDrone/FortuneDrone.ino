#include <Wire.h>                       // Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>      
#include "pid.h"
#include "mpu.h"
#include "irreceiver-config.h"
#include "func.h" 

/* =============================================================
 *         four axis drone                    
 * 		 ↶				 ↷                     ↶		↷
 *       3                4						+   yaw   -
 *         +  x(roll)  -
 *                ↑
 *                |
 *                |             -
 *                L--------→ y(pitch)
 *                              +
 *       2                1
 *       ↻               ↺
 * ============================================================
 */
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 
// 根据 roll 和 pitch 对应的不同，修改 EDF_Ctrl
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

/* ============================================================
 * 
 * 							   MPU
 * 
 * ============================================================
 */
MPU mpu;
float roll = 0.0, pitch = 0.0, roll_acc = 0.0, pitch_acc = 0.0;
float gyro_roll = 0.0, gyro_pitch = 0.0, gyro_yaw = 0.0;

// roll、pitch、yaw 误差
const float RError = 2.21f;
const float PError = 1.20f;
const float YError = 0.0f;
float RAve = 0.0, PAve = 0.0;
int measureCount = 0, count = 0;
const int measureTime = 500;

//  0,  1 : receiver_input[1] 对应的 center
//  2,  3 : receiver_input[2] 对应的 center
//  4,  5 : receiver_input[3] 对应的 center
//  6,  7 : receiver_input[4] 对应的 center
//  8,  9 : receiver_input[1] 对应的 high
// 10, 11 : receiver_input[2] 对应的 high
// 12, 13 : receiver_input[3] 对应的 high
// 14, 15 : receiver_input[4] 对应的 high
// 16, 17 : receiver_input[1] 对应的 low
// 18, 19 : receiver_input[2] 对应的 low
// 20, 21 : receiver_input[3] 对应的 low
// 22, 23 : receiver_input[4] 对应的 low
// 24 : channel 1 对应的 receiver_input 的 index，以及是否 reverse
// 25 : channel 2 对应的 receiver_input 的 index，以及是否 reverse
// 26 : channel 3 对应的 receiver_input 的 index，以及是否 reverse
// 27 : channel 4 对应的 receiver_input 的 index，以及是否 reverse
// 28 : acc_axis[4] 和 gyro_axis[4] 中 roll 值所在的 index
// 29 : acc_axis[4] 和 gyro_axis[4] 中 pitch 值所在的 index
// 30 : acc_axis[4] 和 gyro_axis[4] 中 yaw 值所在的 index
// 31 : if MPU-6050 is ready
// 32 : MPU_ADDRESS
// 33 : J
// 34 : M
// 35 : B
byte eeprom_data[36];

// 用于保存当前时间毫秒值
long start = 0, end = 0;

float batteryVoltage = 0;

void setup(){
	Serial.begin(115200);
	Serial.println("\n===================================\n开始初始化\n");

	// Serial.begin(57600);
	// Copy the EEPROM data for fast access data.
	for (int i = 0; i <= 35; i++) {
		eeprom_data[i] = EEPROM.read(i);
	}

	Wire.begin();                                                             // Start the I2C as master.
	TWBR = 12;                                                                // Set the I2C clock speed to 400kHz.

	// Check the EEPROM signature to make sure that the setup program is executed.
	while (eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B') {
		delay(10);
	}


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

	initESC();

	// 初始化遥控
	initAttach_channel();        
	
	batteryVoltage = (analogRead(0) + 65) * 1.2317;                                       // Set PCINT3 (digital input 11) to trigger an interrupt on state change.

	Serial.println("初始化完毕\n===================================\n");
	
	start = millis();
}

void loop(){

	mpu.readSensor();
	// 加速度（不用 long 求合加速度会溢出）
	long ax = mpu.getAccX();
	long ay = mpu.getAccY();
	long az = mpu.getAccZ();
	// 螺旋仪
	float gx = mpu.getGyroX();
	float gy = mpu.getGyroY();
	float gz = mpu.getGyroZ();
	// output_MPU(ax, ay, az, gx, gy, gz);

	// more info @see https://www.jianshu.com/p/1c208df3cfc8
	// MPU6050 数据寄存器是一个 16 位的，最高位是符号位，故而数据寄存器的输出范围 -7FFF~7FFF,也就是 -32767~32767。
	// 所以 gyro LBS（最小有效位）= 32767 / range = 32767 / 500 = 65.5
	// 本算法采用对 gyro 进行 PID 控制
	gyro_roll = (gyro_roll * 0.7) + (gx / 65.5 * 0.3);
	gyro_pitch = (gyro_pitch * 0.7) + (gy / 65.5 * 0.3);
	gyro_yaw = (gyro_yaw * 0.7) + (gz / 65.5 * 0.3);

	// 0.0000611 = 1 / 250Hz * 65.5
	roll += gx * 0.0000611;
	pitch += gy * 0.0000611;

	// 0.000001066 = 0.0000611 * (3.142(PI) / 180deg)
	roll += pitch * sin(gz * 0.000001066);
	pitch -= roll * sin(gz * 0.000001066);

	// angle will drift
	long acc_total = sqrt((ax*ax)+(ay*ay)+(az*az));

	// Calculate the roll angle.
	if (abs(ax) < acc_total){ roll_acc = asin((float)ax/acc_total) * -57.296; }
	// Calculate the pitch angle.
	if (abs(ay) < acc_total){ pitch_acc = asin((float)ay/acc_total) * 57.296; }

	// Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
	roll_acc -= RError; pitch_acc -= PError; 

	// Correct the drift of the gyro roll angle with the accelerometer roll angle.
	roll = roll * 0.9996 + roll_acc * 0.0004;   
	pitch = pitch * 0.9996 + pitch_acc * 0.0004;

	if (state == STATE_INIT) {
		count++;
		if (count >= 1){
			Serial.print("roll: ");	Serial.print(roll, 2);		printSpace(roll);

			Serial.print("pitch: ");Serial.print(pitch, 2);		printSpace(pitch);
			
			Serial.println("");

			count = 0;
		}
		// 遥控器左位于中下， 右边位于中左， 开启飞行
		if (CHInMin(INDEX_PWM) && CHInCenter(INDEX_YAW) && CHInMin(INDEX_ROLL) && CHInCenter(INDEX_PITCH)) {
			state = STATE_FLY;
			ESC_Ctrl(1000, 0, 0, 0);
			roll = roll_acc;
			pitch = pitch_acc;
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
			ESC_Ctrl(1000, 0, 0, 0);
			Serial.println("\n=================\n开始测量 Roll, pitch 误差\n");
		}
			
	} else if (state == STATE_FLY) {
		// 遥控器左边位于中下，右边位于中右，停止飞行
		if (CHInMin(INDEX_PWM) && CHInCenter(INDEX_YAW) && CHInMax(INDEX_ROLL) && CHInCenter(INDEX_PITCH)){
			state = STATE_INIT;
			ESC_Ctrl(1000, 0, 0, 0);
		} else {
			// 根据 channel 值计算出人为设定的 roll、pitch、yaw
			targetRoll = channel2Angle(convertReceiverChannel(channels[INDEX_ROLL]), maxAngle_RP);
			targetPitch = channel2Angle(convertReceiverChannel(channels[INDEX_PITCH]), maxAngle_RP);
			// targetRoll = 0.0;
			// targetPitch = 0.0;
			targetYaw = channel2Rate(convertReceiverChannel(channels[INDEX_YAW]), maxRate_Y, false);
			// targetYaw = 0.0;

			pidController.calCurrentPitchAnglePID(pitch, targetPitch);
			pidController.calCurrentRollAnglePID(roll, targetRoll);
			pidController.calCurrentYawAnglePID(gyro_yaw, targetYaw);

			float rollCORR = pidController.getRollCorrect(PIDController::ANGLE);
			float pitchCORR = pidController.getPitchCorrect(PIDController::ANGLE);
			float yawCORR = pidController.getYawCorrect(PIDController::RATE);

			if (convertReceiverChannel(channels[INDEX_YAW]) > 1020){
				ESC_Ctrl(convertReceiverChannel(channels[INDEX_YAW]), rollCORR, pitchCORR, yawCORR);
			} else {
				ESC_Ctrl(1000, 0, 0, 0);
				pidController.cleanRollPIDData();
				pidController.cleanPitchPIDData();
				pidController.cleanYawPIDData();
			}

			if (convertReceiverChannel(channels[INDEX_PWM]) > 1020){
				outputMidVal(roll, pitch, gyro_yaw, rollCORR, pitchCORR, yawCORR, convertReceiverChannel(channels[INDEX_PWM]), targetRoll, targetPitch, targetYaw);
			} else if(convertReceiverChannel(channels[INDEX_PWM]) > 900){
				count++;
				if (count >= 50){
					outputMidVal(roll, pitch, gyro_yaw, rollCORR, pitchCORR, yawCORR, channels[INDEX_PWM], targetRoll, targetPitch, targetYaw);
					count = 0;
				}
			}
		}
	} else if (state == STATE_MEAS) {
		measureRPError(ax, ay, acc_total);
	}
	
	
	
}


void cal_PID(){
	// Roll calculations
	pidController.calCurrentRollAnglePID(roll, targetRoll);

	// Pitch calculations
	pidController.calCurrentPitchAnglePID(pitch, targetPitch);


	// Yaw calculations
	pidController.calCurrentYawAnglePID(targetYaw, targetYaw);
	
}

// 将红外信号转化为标准的 1000 – 1500 – 2000
// The stored data in the EEPROM is used.
int convertReceiverChannel(byte function){
	byte channel;
	bool reverse = false;
	int low, center, high, actual;
	int difference;

	// 根据特定 function 找到 channel 1~4
	// 1 对应 0b00000001
	// 2 对应 0b00000010
	// 3 对应 0b00000011
	// 4 对应 0b00000100
	channel = eeprom_data[function + 23] & 0b00000111;
	// Reverse channel when most significant bit is set
	if (eeprom_data[function + 23] & 0b10000000) reverse = true;
	else reverse = false;

	// Read the actual receiver value for the corresponding function
	actual = channels[channel];
	// Store the low value for the specific receiver input channel
	low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];
	center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2];
	high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];

	if (actual < center) {
		if (actual < low) actual = low;
		// Calculate and scale the actual value to a 1000 - 2000us value
		difference = (long)(center - actual) / (center - low) * (long)500;
		// If the channel is reversed
		if (!reverse) return 1500 - difference;
		else return 1500 + difference;
	} else if (actual > center) {
		if (actual > high) actual = high;
		difference = (long)(actual - center) / (high - center) * (long)500;
		if (!reverse) return 1500 + difference;
		else return 1500 - difference;
	} else return 1500;
}

float channel2Angle(int channel, float range){
	if (channel < minCH2Angle_N){
		return -range;
	} else if (minCH2Angle_N < channel && channel < maxCH2Angle_N){
		return -myMap(channel, maxCH2Angle_N, minCH2Angle_N, 0.0, range);
	} else if (maxCH2Angle_N < channel && channel < minCH2Angle_P){
		return 0.0;
	} else if (minCH2Angle_P < channel && channel < maxCH2Angle_P){
		return myMap(channel, minCH2Angle_P, maxCH2Angle_P, 0.0, range);
	} else if (channel > maxCH2Angle_P){
		return range;
	}
}

float channel2Rate(int channel, float range, bool isExpand){
	if (isExpand){ return channel2Angle(channel, range) * 3.0; }
	else { return channel2Angle(channel, range); }
}

bool CHInMin(int index){ return channels[index] < minCH2Angle_N; }
bool CHInCenter(int index){ return maxCH2Angle_N < channels[index] && channels[index] < minCH2Angle_P; }
bool CHInMax(int index){ return channels[index] > maxCH2Angle_P; }


// 输出PWM，控制电机
void ESC_Ctrl(int throttle, float rollCORR, float pitchCORR, float yawCORR) {
	if (throttle > 1400) throttle = 1400;
	ESC_PWM[0] = throttle + rollCORR - pitchCORR - yawCORR;
	ESC_PWM[1] = throttle - rollCORR - pitchCORR + yawCORR;
	ESC_PWM[2] = throttle - rollCORR + pitchCORR - yawCORR;
	ESC_PWM[3] = throttle + rollCORR + pitchCORR + yawCORR;

	zero_timer = micros();
	PORTD |= B11110000;                                            //Set port 4, 5, 6 and 7 high at once
	timer_channel_1 = ESC_PWM[0] + zero_timer;                          //Calculate the time when digital port 4 is set low.
	timer_channel_2 = ESC_PWM[1] + zero_timer;                          //Calculate the time when digital port 5 is set low.
	timer_channel_3 = ESC_PWM[2] + zero_timer;                          //Calculate the time when digital port 6 is set low.
	timer_channel_4 = ESC_PWM[3] + zero_timer;                          //Calculate the time when digital port 7 is set low.

	// 确保和实际模型motor的对应关系
	while (PORTD >= 16) {                                            //Execute the loop until digital port 4 to 7 is low.
		esc_loop_timer = micros();                                   //Check the current time.
		if(timer_channel_1 <= esc_loop_timer) PORTD &= B11101111;     //When the delay time is expired, digital port 4 is set low.
		if(timer_channel_2 <= esc_loop_timer) PORTD &= B11011111;     //When the delay time is expired, digital port 5 is set low.
		if(timer_channel_3 <= esc_loop_timer) PORTD &= B10111111;     //When the delay time is expired, digital port 6 is set low.
		if(timer_channel_4 <= esc_loop_timer) PORTD &= B01111111;     //When the delay time is expired, digital port 7 is set low.
	}
}

void measureRPError(long ax, long ay, long acc_total) {
	if (abs(ax) < acc_total){
		RAve += asin((float)ax/acc_total) * -57.296;
	}
	if (abs(ay) < acc_total){                                     
		PAve += asin((float)ay/acc_total) * 57.296;
	}
	measureCount++;
	if (measureCount >= measureTime){
		RAve /= measureTime;
		PAve /= measureTime;
		measureCount = 0;
		Serial.print("    RollAccError: "); Serial.println(RAve);
		Serial.print("    PitchAccError: "); Serial.println(PAve);
		Serial.println("\n测量完毕\n===================================\n");
		state = STATE_INIT;
	}
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
void outputMidVal(float roll, float pitch, float gz, float rollCORR, float pitchCORR, float yawCORR, int PWM, float targetRoll, float targetPitch, float targetYaw){
	float RInteg = pidController.getRollInteg(PIDController::ANGLE);
	float RDeriv = pidController.getRollDeriv(PIDController::ANGLE);
	float PInteg = pidController.getPitchInteg(PIDController::ANGLE);
	float PDeriv = pidController.getPitchDeriv(PIDController::ANGLE);
	float YError = pidController.getYawError(PIDController::RATE);
	float YInteg = pidController.getYawInteg(PIDController::RATE);
	float YDeriv = pidController.getYawDeriv(PIDController::RATE);

	Serial.print("PWM: ");		Serial.print(channels[INDEX_PWM]);
	if (channels[INDEX_PWM] < 1000){ Serial.print("   "); } else { Serial.print("  "); }

	Serial.print("R: ");		Serial.print(roll, 2);		printSpace(roll);
	Serial.print("RError: "); 	Serial.print(targetRoll - roll, 2);	printSpace(targetRoll - roll);
	Serial.print("RInteg: ");	Serial.print(RInteg, 2);	printSpace(RInteg);
	Serial.print("RDeriv: ");	Serial.print(RDeriv, 2);	printSpace(RDeriv);
	Serial.print("RCORR: ");	Serial.print(rollCORR, 2);	Serial.print("   ");

	Serial.print("P: ");		Serial.print(pitch, 2);		printSpace(pitch);
	Serial.print("PError: "); 	Serial.print(targetPitch - pitch, 2);	printSpace(targetPitch - pitch);
	Serial.print("PInteg: ");	Serial.print(PInteg, 2);	printSpace(PInteg);
	Serial.print("PDeriv: ");	Serial.print(PDeriv, 2);	printSpace(PDeriv);
	Serial.print("PCORR: ");	Serial.print(pitchCORR, 2);	Serial.print("   ");

	// Serial.print("gz: ");		Serial.print(gz, 2);		printSpace(gz);
	// Serial.print("YError: ");	Serial.print(YError, 2);	printSpace(YError);
	// Serial.print("YInteg: ");	Serial.print(YInteg, 2);	printSpace(YInteg);
	// Serial.print("YDeriv: ");	Serial.print(YDeriv, 2);	printSpace(YDeriv);
	// Serial.print("YCORR: ");	Serial.print(yawCORR, 2);	Serial.print("   ");
	
	//for (int i = 0; i < 4; i++){ Serial.print(ESC_PWM[i]); Serial.print(",  "); }
	Serial.println("");
}

void printSpace(float val){
	if (val < 0){
		if (abs(val) > 10){ Serial.print(" "); }
		else { Serial.print("  "); }
	}
	else {
		if (abs(val) > 10){ Serial.print("  "); }
		else { Serial.print("   "); }
	}
}

float degree2Rad(float degree){
	// 180/π = 57.3
	return degree / 57.3;
}

void initESC() {
	for (int i = 0; i < 1250 ; i++){
		// Set digital port 4, 5, 6 and 7 high.
		PORTD |= B11110000;
		delayMicroseconds(ESC_INIT_PWM);
		// Set digital port 4, 5, 6 and 7 low.
		PORTD &= B00001111;
		delayMicroseconds(3000);
	}
}
