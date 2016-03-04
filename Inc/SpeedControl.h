#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include <stdint.h>

#define MAX_TORQUE 			1000.0f		// Nmm
#define WHEEL_DIAMETER_IN_MM			133
#define MIN_CONTROL_INTERVAL_IN_MS	10 
#define PPR		1326
#define PI	3.1415927f

static const float DPP = (WHEEL_DIAMETER_IN_MM*PI/PPR);

#define P_GAIN	0.0015f
#define I_GAIN	0.00003f
#define D_GAIN	0.006f

#define EPSILON_F 0.000001f

#ifdef __cplusplus
 extern "C" {
#endif
	 
void PIDStart(int channel, float expected, uint8_t reset);
int PIDStop(int channel, uint8_t brake);
void PIDTick(uint32_t tick);
	
__weak float GetCurrentSpeed(int channel);
__weak void PWMDutyChanged(int channel, float duty);
__weak int BrakeActuator(int channel, int dir);
	 
#ifdef __cplusplus
 }
#endif
 
#endif
