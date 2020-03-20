#include <Wire.h>

#include "mpu.h"

/* 
 * More config info @see https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */

#define MPU_ADDRESS 0x68
#define GYRO_CONFIG 0x1B
#define ACC_CONFIG  0x1C
#define DLPF_CONFIG	0x1A

#define COUNT_calibrate	500

MPU::MPU(){
	gyro_offset[INDEX_X] = 0;
	gyro_offset[INDEX_Y] = 0;
	gyro_offset[INDEX_Z] = 0;
}

int MPU::begin(){
    Wire.beginTransmission(MPU_ADDRESS);	// Start communication with the address found during search.
	Wire.write(0x6B);                   	// We want to write to the PWR_MGMT_1 register (6B hex)
	Wire.write(0x00);                   	// Set the register bits as 00000000 to activate the MPU
	Wire.endTransmission();             	// End the transmission with the MPU.

    // 默认 gyro range 为 500dps
	Wire.beginTransmission(MPU_ADDRESS);
	Wire.write(GYRO_CONFIG);                // We want to write to the GYRO_CONFIG register (1B hex)
	Wire.write(0x08);                   	// Set gyro full scale as 00001000 (500dps)
	Wire.endTransmission();

    // 默认 acc range 为 ±8g
	Wire.beginTransmission(MPU_ADDRESS);
	Wire.write(ACC_CONFIG);                 // We want to write to the ACCEL_CONFIG register (1A hex)
	Wire.write(0x10);                   	// Set acc full scale range as 00010000 (+/- 8g)
	Wire.endTransmission();

	// 测试数值是否正确写入
	Wire.beginTransmission(MPU_ADDRESS);
	Wire.write(GYRO_CONFIG);                // Start reading @ register 0x1B
	Wire.endTransmission();
	Wire.requestFrom(MPU_ADDRESS, 1);   	// Request 1 bytes from the MPU
	while (Wire.available() < 1);       	// Wait until the 6 bytes are received
	if (Wire.read() != 0x08) {
		return -1;
	}

	Wire.beginTransmission(MPU_ADDRESS);
	Wire.write(0x1A);                   	// We want to write to the CONFIG register (1A hex)
	Wire.write(0x03);                   	// Set Digital Low Pass Filter as 00000011 (~43Hz)
	Wire.endTransmission();

    return 1;
}

void MPU::setGyroRange(GyroRange range){
    Wire.beginTransmission(MPU_ADDRESS);
	Wire.write(GYRO_CONFIG);
    switch (range){
        case GYRO_RANGE_250DPS:
			Wire.write(0x00);
            break;
        case GYRO_RANGE_500DPS:
			Wire.write(0x08);
			break;
		case GYRO_RANGE_1000DPS:
			Wire.write(0x10);
			break;
		case GYRO_RANGE_2000DPS:
			Wire.write(0x18);
			break;
    }
    Wire.endTransmission();
}

void MPU::setAccRange(AccRange range){
    Wire.beginTransmission(MPU_ADDRESS);
	Wire.write(ACC_CONFIG);
    switch (range){
        case ACC_RANGE_2G:
			Wire.write(0x00);
            break;
        case ACC_RANGE_4G:
			Wire.write(0x08);
			break;
		case ACC_RANGE_8G:
			Wire.write(0x10);
			break;
		case ACC_RANGE_16G:
			Wire.write(0x18);
			break;
    }
    Wire.endTransmission();
}

void MPU::setDLPFBandwidth(DLPFBandwidth bandwidth){
    Wire.beginTransmission(MPU_ADDRESS);
	Wire.write(DLPF_CONFIG);
    switch (bandwidth){
		case DLPF_BANDWIDTH_184HZ:
			Wire.write(0x01);
			break;
		case DLPF_BANDWIDTH_92HZ:
			Wire.write(0x02);
			break;
		case DLPF_BANDWIDTH_41HZ:
			Wire.write(0x03);
			break;
		case DLPF_BANDWIDTH_20HZ:
			Wire.write(0x04);
			break;
        case DLPF_BANDWIDTH_10HZ:
			Wire.write(0x05);
			break;
    }
    Wire.endTransmission();
}

void MPU::readSensor(){
	Wire.beginTransmission(MPU_ADDRESS);
	Wire.write(0x3B);
	Wire.endTransmission();
	Wire.requestFrom(MPU_ADDRESS, 14);
	while (Wire.available() < 14);
	// Read high and low part of the angular data.
	acc_axis[INDEX_X] = Wire.read()<<8|Wire.read();
	acc_axis[INDEX_Y] = Wire.read()<<8|Wire.read();
	acc_axis[INDEX_Z] = Wire.read()<<8|Wire.read();
	temperature = Wire.read()<<8|Wire.read();
	gyro_axis[INDEX_X] = Wire.read()<<8|Wire.read();
	gyro_axis[INDEX_Y] = Wire.read()<<8|Wire.read();
	gyro_axis[INDEX_Z] = Wire.read()<<8|Wire.read();
}

void MPU::calibrateGyro(){
	gyro_offset[INDEX_X] = 0;
	gyro_offset[INDEX_Y] = 0;
	gyro_offset[INDEX_Z] = 0;
	for (int i=0; i<COUNT_calibrate; i++){
		readSensor();
		gyro_offset[INDEX_X] += gyro_axis[INDEX_X];
		gyro_offset[INDEX_Y] += gyro_axis[INDEX_Y];
		gyro_offset[INDEX_Z] += gyro_axis[INDEX_Z];
	}
	gyro_offset[INDEX_X] /= COUNT_calibrate;
	gyro_offset[INDEX_Y] /= COUNT_calibrate;
	gyro_offset[INDEX_Z] /= COUNT_calibrate;
}
