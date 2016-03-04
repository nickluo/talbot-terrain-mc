#include "SpeedControl.h"

#include <cmath>
#include <cstdlib>
#include <cstdio>

struct PIDState
{
	uint8_t Controlled;	//0- Idle, 1-Controlled, 2-Brake 0xff-Started
	float ExpectedSpeed;
	float LastSpeed;
	//float CurrentSpeed;
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
//static uint32_t LastCount[2] = { 0, 0 };

void PIDControl(int channel);

__weak void PWMDutyChanged(int channel, float duty)
{
}

#define TF_N	2			//Normal 2-20

static const float Tf = (D_GAIN/P_GAIN)/TF_N;
static const float Tt = 50;

void PIDStart(int channel, float expected, uint8_t reset)
{
	PIDStates[channel].ExpectedSpeed = expected;
	if (reset)
	{
		PIDStates[channel].LastSpeed = 0;
		PIDStates[channel].PWMDuty = 0;
		PWMDutyChanged(channel, 0);
		
		PIDStates[channel].DState=0;
		PIDStates[channel].IState=0;
		PIDStates[channel].Controlled = 0xff; 				//Started
	}
	else if (PIDStates[channel].Controlled==0)
	{
		PIDStates[channel].PWMDuty = 0;
		PIDStates[channel].Controlled = 1;
	}
	
	if (std::abs(expected)<EPSILON_F)
		PIDStates[channel].Interval = 100;
	else
	{
		//PIDStates[channel].Interval = (int)(DPP*10000/std::abs(PIDStates[channel].ExpectedSpeed+0.5f))*2;
		//if (PIDStates[channel].Interval<MIN_CONTROL_INTERVAL_IN_MS)
			PIDStates[channel].Interval = MIN_CONTROL_INTERVAL_IN_MS;
	}
	
	PIDStates[channel].Bi = I_GAIN * PIDStates[channel].Interval;
	PIDStates[channel].Ad = Tf / (Tf + PIDStates[channel].Interval);
	PIDStates[channel].Bd = D_GAIN / (Tf + PIDStates[channel].Interval);
	PIDStates[channel].Br = PIDStates[channel].Interval/Tt;
}

__weak int BrakeActuator(int channel, int dir)
{
	return 0;
}

int PIDStop(int channel, uint8_t brake)
{
	if (brake)
	{
		if (PIDStates[channel].Controlled == 2)
		{
			if ((int)(GetCurrentSpeed(channel))!=0)
				return 1;
		}
		else
		{
			PIDStates[channel].Controlled = 2;
			if (BrakeActuator(channel, GetCurrentSpeed(channel)>0?1:-1))
				return 1;
		}
	}
	PIDStates[channel].Controlled = 0;
	PIDStates[channel].PWMDuty = 0;
	PWMDutyChanged(channel, 0);
	return 0;
}

void PIDControl(int channel, float currentSpeed)
{
	float error = PIDStates[channel].ExpectedSpeed - currentSpeed;
	float P = P_GAIN * error;
	PIDStates[channel].DState = PIDStates[channel].Ad * PIDStates[channel].DState -
		PIDStates[channel].Bd * (currentSpeed - PIDStates[channel].LastSpeed);
	
	float v = P + PIDStates[channel].DState + PIDStates[channel].IState;
	
	//Anti-Windup
	// u-v 		u=sat(v,low,high)
	if (v>1)
		PIDStates[channel].PWMDuty = 1;
	else if (v<-1)
		PIDStates[channel].PWMDuty = -1;
	else
		PIDStates[channel].PWMDuty = v;
	
	PWMDutyChanged(channel, PIDStates[channel].PWMDuty);
	PIDStates[channel].IState += PIDStates[channel].Bi*error + PIDStates[channel].Br*(PIDStates[channel].PWMDuty-v);
	PIDStates[channel].LastSpeed = currentSpeed;
}

__weak int GetDeltaCount(uint8_t ch, uint32_t *lastCount)
{
	return 0;
}

__weak float GetCurrentSpeed(int channel)
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
			PIDControl(i,GetCurrentSpeed(i));
			PIDStates[i].Checkpoint = tick;
			PIDStates[i].Controlled = 1;
			continue;
		}
		int duration = tick>=PIDStates[i].Checkpoint ?
			tick-PIDStates[i].Checkpoint :
			0xffffffffu - PIDStates[i].Checkpoint + tick + 1;
		if (duration >= PIDStates[i].Interval)
		{
			if (PIDStates[i].Controlled==1)	//Normal PID Control
				PIDControl(i, GetCurrentSpeed(i));
			PIDStates[i].Checkpoint = tick;
		}
	}
}
