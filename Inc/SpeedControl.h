#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include <stdint.h>

#define MAX_TORQUE 			1000.0f		// Nmm
#define WHEEL_DIAMETER_IN_MM			133
#define MIN_CONTROL_INTERVAL_IN_MS		1 
#define PPR		1326
#define PI	3.1415927f
#define MIN_PWM_DUTY		0.06f

static const float DPP = (WHEEL_DIAMETER_IN_MM*PI/PPR);

#define P_GAIN	0.00008f
#define D_GAIN	0.00002f
#define I_GAIN	0.00000002f

#define BRAKE_COEFF 1.f

#define EPSILON_F 0.000001f

#ifdef __cplusplus
 extern "C" {
#endif

void OutputSpeed(int channel);
int Brake(int channel, int steps);
	 
void PIDStart(int channel, float expected, uint8_t reset);
void PIDStop(int channel);
void PIDTick(uint32_t tick);
	
__weak int GetDeltaCount(uint8_t ch, uint32_t *lastCount);
__weak void PWMDutyChanged(int channel, float duty);
	 
#ifdef __cplusplus
 }
#endif
 
#endif
