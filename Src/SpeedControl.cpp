#include "SpeedControl.h"

#include <cmath>
#include <cstdlib>
#include <cstdio>

struct PIDState
{
	uint8_t Controlled;
	float ExpectedSpeed;
	float LastSpeed;
	float CurrentSpeed;
	float DState;
	float IState;
	float PWMDuty;
	uint32_t Checkpoint;
	int Interval;
	float Bi;
	float Ad;
	float Bd;
	float Br;
};

static PIDState PIDStates[2];
static uint32_t LastCount[2] = { 0, 0 };

void PIDControl(int channel);

void OutputSpeed(int channel)
{
	std::printf("CH %d Speed: %d mm/s\r\n", channel, (int)(PIDStates[channel].CurrentSpeed+0.5f));
	//fflush(stdout);
}

//static float Work[2] = { 0, 0 };
//static float CoeffK[2] = { 0, 0 };
//static float LastSpeed[2] = { 0, 0 };

//static float KS[2] = { 0, 0};
//static float K1[2] = { 0, 0};
//static float F1[2] = { 0, 0};
//     
//int32_t EstimateStopDistance(int channel, float delta, uint8_t reset)
//{
//	if (std::abs(delta)>EPSILON_F)
//	{
//		float f2 = std::abs(PIDStates[channel].PWMDuty) * MAX_TORQUE * 2 / WHEEL_DIAMETER_IN_MM;
//		float dv = PIDStates[channel].CurrentSpeed-LastSpeed[channel];
//		LastSpeed[channel] = PIDStates[channel].CurrentSpeed;
//		float k2 = 2*dv*dv/std::abs(delta);
//		if (k2>EPSILON_F)
//		{
//			float dF = F1[channel]-f2;
//			if (std::abs(dF)>EPSILON_F)
//				KS[channel] = (K1[channel]*f2-k2*F1[channel])/dF;
//			F1[channel] = f2;
//			
//		}
//	
//		Work[channel] += fs*delta;
//	}
//	if (KS[channel]<EPSILON_F)
//		return 0;
//	return (int32_t)((PIDStates[channel].CurrentSpeed*PIDStates[channel].CurrentSpeed)/KS[channel]);
//}

__weak void PWMDutyChanged(int channel, float duty)
{
}

static uint8_t BrakeTag[2] = {0, 0};

int Brake(int channel, int steps)
{
//	if (BrakeTag[channel])
//		return steps;
//	if (std::abs(steps) < PPR * BRAKE_COEFF * std::abs(PIDStates[channel].CurrentSpeed)/1000)
//	{
//		BrakeTag[channel] = 1;
//		//PIDStart(channel, PIDStates[channel].ExpectedSpeed>0?100:-100, 0);
//		PIDStart(channel, 0, 0);
//		//PIDStop(channel);
//		//return 0;
//	}
	return steps;
}

#define TF_N	2			//Normal 2-20

static const float Tf = (D_GAIN/P_GAIN)/TF_N;
static const float Tt = 1000;

void PIDStart(int channel, float expected, uint8_t reset)
{
	PIDStates[channel].ExpectedSpeed = expected;
	if (reset)
	{
		PIDStates[channel].CurrentSpeed = 0;
		PIDStates[channel].LastSpeed = 0;
		PIDStates[channel].PWMDuty = expected>0 ? MIN_PWM_DUTY : -MIN_PWM_DUTY;
		PWMDutyChanged(channel, 0);
		
		PIDStates[channel].DState=0;
		PIDStates[channel].IState=0;
		PIDStates[channel].Controlled = 0xff; 				//Started
	}
	else if (PIDStates[channel].Controlled==0)
	{
		PIDStates[channel].PWMDuty = expected>0 ? MIN_PWM_DUTY : -MIN_PWM_DUTY;
		PIDStates[channel].Controlled = 1;
	}
	
	if (std::abs(expected)<EPSILON_F)
		PIDStates[channel].Interval = 100;
	else
	{
		PIDStates[channel].Interval = (int)(DPP*10000/std::abs(PIDStates[channel].ExpectedSpeed+0.5f))*2;
		if (PIDStates[channel].Interval<MIN_CONTROL_INTERVAL_IN_MS)
			PIDStates[channel].Interval = MIN_CONTROL_INTERVAL_IN_MS;
	}
	
	PIDStates[channel].Bi = I_GAIN * PIDStates[channel].Interval;
	PIDStates[channel].Ad = Tf / (Tf + PIDStates[channel].Interval);
	PIDStates[channel].Bd = D_GAIN / (Tf + PIDStates[channel].Interval);
	PIDStates[channel].Br = PIDStates[channel].Interval/Tt;

	GetDeltaCount(channel, &LastCount[channel]); 	//Reset counter
}

void PIDStop(int channel)
{
	PIDStates[channel].Controlled = 0;
	PIDStates[channel].PWMDuty = 0;
	PWMDutyChanged(channel, 0);
	BrakeTag[channel] = 0;
}

//void PIDControl(int channel)
//{
//	static const float Windup = I_GAIN==0 ? 5000 : 1.0f/I_GAIN;
//	
//	float error = PIDStates[channel].ExpectedSpeed - PIDStates[channel].CurrentSpeed;
////	if (std::abs(error)<DPP*1000/PIDStates[channel].Interval)
////		error = 0;
//	
//	float pTerm = P_GAIN * error;
//	float dTerm = D_GAIN * (error - PIDStates[channel].DState);
//	PIDStates[channel].DState = error;
//	PIDStates[channel].IState += error;
//	if (PIDStates[channel].IState > Windup)
//		PIDStates[channel].IState = Windup;
//	else if (PIDStates[channel].IState < -Windup)
//		PIDStates[channel].IState = -Windup;
//	float iTerm = I_GAIN * PIDStates[channel].IState;
//	PIDStates[channel].PWMDuty += pTerm + dTerm + iTerm;
//	if (PIDStates[channel].PWMDuty>0)
//	{
////		if (PIDStates[channel].ExpectedSpeed<0)
////			PIDStates[channel].PWMDuty = 0;
////		else 
//		if (PIDStates[channel].PWMDuty<MIN_DUTY_CYCLE)
//			PIDStates[channel].PWMDuty = (PIDStates[channel].ExpectedSpeed<0) ? 0 : MIN_DUTY_CYCLE;
//		else if (PIDStates[channel].PWMDuty>1)
//			PIDStates[channel].PWMDuty = 1;
//	}
//	else
//	{
////		if (PIDStates[channel].ExpectedSpeed>0)
////			PIDStates[channel].PWMDuty = 0;
////		else 
//		if (PIDStates[channel].PWMDuty>-MIN_DUTY_CYCLE)
//			PIDStates[channel].PWMDuty = (PIDStates[channel].ExpectedSpeed>0) ? 0 : -MIN_DUTY_CYCLE;
//		else if (PIDStates[channel].PWMDuty<-1)
//			PIDStates[channel].PWMDuty = -1;
//	}
//	PWMDutyChanged(channel, PIDStates[channel].PWMDuty);
//}

void PIDControl(int channel)
{
	float error = PIDStates[channel].ExpectedSpeed - PIDStates[channel].CurrentSpeed;
	float P = P_GAIN * error;
	PIDStates[channel].DState = PIDStates[channel].Ad * PIDStates[channel].DState -
		PIDStates[channel].Bd * (PIDStates[channel].CurrentSpeed - PIDStates[channel].LastSpeed);
	
	float v = P + PIDStates[channel].DState + PIDStates[channel].IState;
	
	//Anti-Windup
	float rs = 0;	// u-v 		u=sat(v,low,high)
	PIDStates[channel].PWMDuty += v;
	if (PIDStates[channel].PWMDuty>1)
	{
		rs = 1 - PIDStates[channel].PWMDuty;
		PIDStates[channel].PWMDuty = 1;
	}
	else if (PIDStates[channel].PWMDuty<-1)
	{
		rs = -1 - PIDStates[channel].PWMDuty;
		PIDStates[channel].PWMDuty = -1;
	}
	
	PWMDutyChanged(channel, PIDStates[channel].PWMDuty);
	PIDStates[channel].IState += PIDStates[channel].Bi*error + PIDStates[channel].Br*rs;
	PIDStates[channel].LastSpeed = PIDStates[channel].CurrentSpeed;
}

__weak int GetDeltaCount(uint8_t ch, uint32_t *lastCount)
{
	return 0;
}

void PIDTick(uint32_t tick)
{
	for(int i=0;i<2;++i)
	{
		if (PIDStates[i].Controlled==0)
			continue;
		if (PIDStates[i].Controlled==0xff) //Just started
		{
			PIDControl(i);
			PIDStates[i].Checkpoint = tick;
			PIDStates[i].Controlled = 1;
			continue;
		}
		int duration = tick>=PIDStates[i].Checkpoint ?
			tick-PIDStates[i].Checkpoint :
			0xffffffffu - PIDStates[i].Checkpoint + tick + 1;
		if (duration >= PIDStates[i].Interval)
		{
			PIDStates[i].CurrentSpeed = GetDeltaCount(i, &LastCount[i])*DPP*1000/PIDStates[i].Interval;
			PIDControl(i);
			PIDStates[i].Checkpoint = tick;
		}
	}
}
