#include "mpu.h"
#include <Wire.h>                       // Include the Wire.h library so we can communicate with the gyro.

MPU mpu;
float roll = 0.0, pitch = 0.0, roll_output = 0, pitch_output = 0, roll_acc = 0.0, pitch_acc = 0.0;
float gyro_roll = 0.0, gyro_pitch = 0.0, gyro_yaw = 0.0;
long ax, ay, az, acc_total;
float gx, gy, gz;
int count = 0;
int time1 = 0;

void setup() {
    Serial.begin(57600);

    Wire.begin();

    int status = mpu.begin();
    if (status < 0) {
        Serial.println("MPU initialization failed.");
        while (1){}
    }
    mpu.setGyroRange(MPU::GYRO_RANGE_500DPS);
	  mpu.setAccRange(MPU::ACC_RANGE_8G);
	  mpu.setDLPFBandwidth(MPU::DLPF_BANDWIDTH_41HZ);
	  delay(10);

	//计算 MPU gyro 误差
	  mpu.calibrateGyro();
    time1 = micros();
}

void loop() {
  mpu.readSensor();
	// 加速度（不用 long 求合加速度会溢出）
	ax = mpu.getAccX();
	ay = mpu.getAccY();
	az = mpu.getAccZ();
	// 螺旋仪
	gx = mpu.getGyroX();
	gy = mpu.getGyroY();
	gz = mpu.getGyroZ();
	// output_MPU(ax, ay, az, gx, gy, gz);

	// more info @see https://www.jianshu.com/p/1c208df3cfc8
	// MPU6050 数据寄存器是一个 16 位的，最高位是符号位，故而数据寄存器的输出范围 -7FFF~7FFF,也就是 -32767~32767。
	// 所以 gyro LBS（最小有效位）= 32767 / range = 32767 / 500 = 65.5
	// 本算法采用对 gyro 进行 PID 控制

	// 0.0000611 = 1 / 250Hz * 65.5
	roll += gx * 0.0000611;
	pitch += gy * 0.0000611;

	// 0.000001066 = 0.0000611 * (3.142(PI) / 180deg)
	roll += pitch * sin(gz * 0.000001066);
	pitch -= roll * sin(gz * 0.000001066);

	// angle will drift
	acc_total = sqrt((ax*ax)+(ay*ay)+(az*az));

	// Calculate the roll angle.
	if (abs(ay) < acc_total){ roll_acc = asin((float)ay/acc_total) * 57.296; }
	// Calculate the pitch angle.
	if (abs(ax) < acc_total){ pitch_acc = asin((float)ax/acc_total) * -57.296; }

	// Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
	roll_acc -= 0; pitch_acc -= 0; 

  if (count == 0) {
    pitch = pitch_acc;
    roll = roll_acc;
    count++;
  } else {
    // Correct the drift of the gyro roll angle with the accelerometer roll angle.
    roll = roll * 0.9996 + roll_acc * 0.0004;   
    pitch = pitch * 0.9996 + pitch_acc * 0.0004;
  }
	roll_output = roll_output * 0.9 + roll * 0.1;
  pitch_output = pitch_output * 0.9 + pitch * 0.1;
  outputMidVal(roll, pitch);
  while (micros() - time1 < 4000);
  time1 = micros();  
}

void outputMidVal(float roll, float pitch){
	// float RInteg = pidController.getRollInteg(PIDController::ANGLE);
	// float RDeriv = pidController.getRollDeriv(PIDController::ANGLE);
	// float PInteg = pidController.getPitchInteg(PIDController::ANGLE);
	// float PDeriv = pidController.getPitchDeriv(PIDController::ANGLE);
	// float YError = pidController.getYawError(PIDController::RATE);
	// float YInteg = pidController.getYawInteg(PIDController::RATE);
	// float YDeriv = pidController.getYawDeriv(PIDController::RATE);

	// Serial.print("PWM: ");		Serial.print(channels[INDEX_PWM]);
	// if (channels[INDEX_PWM] < 1000){ Serial.print("   "); } else { Serial.print("  "); }

	Serial.print("R: ");		Serial.print(roll, 2);		printSpace(roll);
	// Serial.print("RError: "); 	Serial.print(targetRoll - roll, 2);	printSpace(targetRoll - roll);
	// Serial.print("RInteg: ");	Serial.print(RInteg, 2);	printSpace(RInteg);
	// Serial.print("RDeriv: ");	Serial.print(RDeriv, 2);	printSpace(RDeriv);
	// Serial.print("RCORR: ");	Serial.print(rollCORR, 2);	Serial.print("   ");

	Serial.print("P: ");		Serial.print(pitch, 2);		printSpace(pitch);
	// Serial.print("PError: "); 	Serial.print(targetPitch - pitch, 2);	printSpace(targetPitch - pitch);
	// Serial.print("PInteg: ");	Serial.print(PInteg, 2);	printSpace(PInteg);
	// Serial.print("PDeriv: ");	Serial.print(PDeriv, 2);	printSpace(PDeriv);
	// Serial.print("PCORR: ");	Serial.print(pitchCORR, 2);	Serial.print("   ");

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

void output_MPU(float ax, float ay, float az, float gx, float gy, float gz){
  Serial.print("ax: "); Serial.print(ax); Serial.print("  ");
  Serial.print("ay: "); Serial.print(ay); Serial.print("  ");
  Serial.print("az: "); Serial.print(az); Serial.print("  "); Serial.print(" \t");
  Serial.print("gx: "); Serial.print(gx); Serial.print("  ");
  Serial.print("gy: "); Serial.print(gy); Serial.print("  ");
  Serial.print("gz: "); Serial.print(gz); Serial.print("  "); Serial.println("");
}
