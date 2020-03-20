#ifndef __irreceiver_h__
#define __irreceiver_h__

#include "Arduino.h"

// @see https://www.youtube.com/watch?v=j-kE0AMEWy4 9:37 提到该姿态解算算法，MPU 最大有效量程 60°
// roll 和 pitch 人为设置量程
#define maxAngle_RP		30.6		// 460/15 = 30.6
#define maxAngle_Y		30.6
// roll 和 pitch 人为设置角速度量程
#define maxRate_RP		153.3		// 460/3 = 153.3
#define maxRate_Y		153.3

// 用 1020～1480 表示 -20°～0°，1520～1980 表示 0°～20°
// usable range is 1480-1020 = 460
#define minCH2Angle_N	1020
#define maxCH2Angle_N	1480
#define minCH2Angle_P	1520
#define maxCH2Angle_P	1980

/* ============================================================
 *                        flysky mode 2
 * 
 *              CH3(PWM)                 CH2(Pitch)  
 *                  |                        |      
 *                  |                        |      
 *     CH4(Yaw) ----+----      CH1(Roll) ----+----  
 * 
 * ============================================================
 * flysky 查看 mode：长按 OK，按 DOWN 向下找到 Sticks mode，按 OK。
 */
int INDEX_CH1 = 0, INDEX_CH2 = 1, INDEX_CH3 = 2, INDEX_CH4 = 3;
volatile int channels[4] = {0, 0, 0, 0};
// 控制 roll、pitch、yaw 和 throttle 的 channel
volatile int INDEX_YAW = -1, INDEX_PWM = -1, INDEX_ROLL = -1, INDEX_PITCH = -1;

bool last_channel_1 = false, last_channel_2 = false;
bool last_channel_3 = false, last_channel_4 = false;
long current_time;
long time_1, time_2, time_3, time_4;

void initAttach_channel(){
	// @see https://www.arduino.cc/en/Reference/PortManipulation
	// @see https://forum.arduino.cc/index.php?topic=400719.0
	PCICR |= (1 << PCIE0);  
	PCMSK0 |= (1 << PCINT0);	// Set PCINT0 (digital input 8) to trigger an interrupt on state change.
	PCMSK0 |= (1 << PCINT1);	// Set PCINT0 (digital input 9) to trigger an interrupt on state change.
	PCMSK0 |= (1 << PCINT2);	// Set PCINT0 (digital input 10) to trigger an interrupt on state change.
	PCMSK0 |= (1 << PCINT3);	// Set PCINT0 (digital input 11) to trigger an interrupt on state change.

    // 设置控制 roll、pitch、yaw 和 throttle 的 channel
	INDEX_PWM = INDEX_CH3;
	INDEX_YAW = INDEX_CH4;
	INDEX_ROLL = INDEX_CH1;
	INDEX_PITCH = INDEX_CH2;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This routine is called every time input 8, 9, 10 or 11 changed state. This is used to read the receiver signals. 
// More information about this subroutine can be found in this video:
// https://youtu.be/bENjl1KQbvo
// @see https://www.arduino.cc/en/Reference/PortManipulation
// @see https://www.firediy.fr/article/utiliser-sa-radiocommande-avec-un-arduino-drone-ch-6
// pin change interrupt @see https://github.com/NicoHood/PinChangeInterrupt
// 中断处理程序
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){
	current_time = micros();
	// Channel 1=========================================
	if (PINB & B00000001){   								// Is input 8 high?
		if (!last_channel_1){								// Input 8 : 0 -> 1.
			last_channel_1 = true;
			time_1 = current_time;
		}
	}
	else if (last_channel_1){								// Input 8 : 1 -> 0.
		last_channel_1 = false;
		channels[INDEX_CH1] = current_time - time_1;
	}
	// Channel 2=========================================
	if (PINB & B00000010 ){									// Is input 9 high?
		if (!last_channel_2){								// Input 9 : 0 -> 1.
			last_channel_2 = true;
			time_2 = current_time;
		}
	}
	else if (last_channel_2){								// Input 9 : 1 -> 0.
		last_channel_2 = false;
		channels[INDEX_CH2] = current_time - time_2;
	}
	//Channel 3=========================================
	if (PINB & B00000100 ){  								// Is input 10 high?
		if (!last_channel_3){								// Input 10 : 0 -> 1.
			last_channel_3 = true;
			time_3 = current_time;
		}
	}
	else if (last_channel_3){								// Input 10 : 1 -> 0.
		last_channel_3 = false;
		channels[INDEX_CH3] = current_time - time_3;

	}
	//Channel 4=========================================
	if (PINB & B00001000 ){									// Is input 11 high?
		if (!last_channel_4){								// Input 11 : 0 -> 1.
			last_channel_4 = true;
			time_4 = current_time;
		}
	}
	else if (last_channel_4){								// Input 11 : 1 -> 0.
		last_channel_4 = false;
		channels[INDEX_CH4] = current_time - time_4;
	}
}

#endif
