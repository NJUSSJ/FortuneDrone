#ifndef __mpu_h__
#define __mpu_h__

#define INDEX_X		0
#define INDEX_Y		1
#define INDEX_Z		2

/*
 * 读取 MPU 元件
 * 读取方式：I2C
 */
class MPU {
    public:
        enum GyroRange {
			GYRO_RANGE_250DPS,
			GYRO_RANGE_500DPS,
			GYRO_RANGE_1000DPS,
			GYRO_RANGE_2000DPS
		};
        enum AccRange {
			ACC_RANGE_2G,
			ACC_RANGE_4G,
			ACC_RANGE_8G,
			ACC_RANGE_16G
		};
		enum DLPFBandwidth {
			DLPF_BANDWIDTH_184HZ,
			DLPF_BANDWIDTH_92HZ,
			DLPF_BANDWIDTH_41HZ,
			DLPF_BANDWIDTH_20HZ,
			DLPF_BANDWIDTH_10HZ,
		};
        MPU();
        int begin();
		// 设置陀螺仪满量程范围
        void setGyroRange(GyroRange range);
		// 设置加速度满量程范围
        void setAccRange(AccRange range);
		// set the programmable Digital Low Pass Filter (DLPF) bandwidth
        void setDLPFBandwidth(DLPFBandwidth bandwidth);
		void readSensor();
		// 校准 gyro，计算得 gyro_offset
		void calibrateGyro();
		float getGyroX(){ return gyro_axis[INDEX_X] - gyro_offset[INDEX_X]; }
		float getGyroY(){ return gyro_axis[INDEX_Y] - gyro_offset[INDEX_Y]; }
		float getGyroZ(){ return gyro_axis[INDEX_Z] - gyro_offset[INDEX_Z]; }
		float getGyroX_Offset(){ return gyro_offset[INDEX_X]; }
		float getGyroY_Offset(){ return gyro_offset[INDEX_Y]; }
		float getGyroZ_Offset(){ return gyro_offset[INDEX_Z]; }
		int getAccX(){ return acc_axis[INDEX_X]; }
		int getAccY(){ return acc_axis[INDEX_Y]; }
		int getAccZ(){ return acc_axis[INDEX_Z]; }
    private:
		int acc_axis[4], gyro_axis[4];
		long gyro_offset[4];
		int temperature;
};

#endif
