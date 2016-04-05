/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "CommandParser.h"
#include "SpeedControl.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BRAKE_ON			 0
#define RX_BUFFER_SIZE 0x200
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
int rx_index = 0;

static char Str[0x100];

struct MotorState
{
	uint8_t Move;
	int 		Steps;
	int8_t 	Dir;
	float 	CurrentSpeed;
	float 	WheelDistance;
};

struct MotorState	Motors[2];
const int8_t WheelSigns[2] = { 1, -1 };

uint8_t TestModeEnable = 0;

volatile uint32_t ObstacleDistance = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

int stdout_putchar (int ch)
{
	while (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&ch, 1) == HAL_BUSY)
		__nop();
	return ch;
}

int stdin_getchar (void)
{
	return 0;
}

int GetDeltaCount(uint8_t ch, uint32_t *lastCount)
{
	uint32_t current = ch==0 ? htim1.Instance->CNT : htim3.Instance->CNT;
	int delta = current-*lastCount;
	if (delta>0x8000)
		delta -= 0x10000; 
	else if (delta<-0x8000)
		delta += 0x10000;
	*lastCount = current;
	return delta;
}

#define BRAKE_DUTY								100
#define	DUTY_RESOLUTION						1000
#define	DUTY_ACTUAL_RESOLUTION		900

int BrakeActuator(int channel, int dir)
{
	if (channel==0)
	{
		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, dir<0 ? GPIO_PIN_SET:GPIO_PIN_RESET);
		htim15.Instance->CCR1 = BRAKE_DUTY;
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	}
	else if (channel==1)
	{
		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, dir<0 ? GPIO_PIN_SET:GPIO_PIN_RESET);
		htim15.Instance->CCR2 = BRAKE_DUTY;
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	}
	return 1;
}

void PWMDutyChanged(int channel, float duty)
{
	uint32_t ccr = (fabs(duty)<EPSILON_F) ? 0 : (uint32_t)(fabs(duty)*DUTY_ACTUAL_RESOLUTION)+BRAKE_DUTY;
	if (channel==0)
	{
		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, duty>0 ? GPIO_PIN_SET:GPIO_PIN_RESET);
		htim15.Instance->CCR1 = ccr;
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	}
	else if (channel==1)
	{
		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, duty>0 ? GPIO_PIN_SET:GPIO_PIN_RESET);
		htim15.Instance->CCR2 = ccr;
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	}
}

#define REPORT_INTERVAL	50

void OutputSpeed(int channel)
{
	printf("CH %d Speed: %d mm/s\r\n", channel, (int)(Motors[channel].CurrentSpeed+0.5f));
}

float GetCurrentSpeed(int channel)
{
	return Motors[channel].CurrentSpeed;
}

void Update()
{
	for(int i=0;i<2;++i)
	{
		if (Motors[i].Move)
		{
			if (TestModeEnable>1 && HAL_GetTick()%REPORT_INTERVAL==0)
				OutputSpeed(i);
			if (Motors[i].Dir>0)
			{
				if (Motors[i].Steps<=0)
					Motors[i].Move = PIDStop(i, BRAKE_ON);
			}
			else
			{
				if (Motors[i].Steps>=0)
					Motors[i].Move = PIDStop(i, BRAKE_ON);
			}
		}
	}
}

void HAL_SYSTICK_Callback(void)
{
	uint32_t tick = HAL_GetTick();
	if (tick%500==0)
		HAL_GPIO_TogglePin(STATUS_GPIO_Port, STATUS_Pin);
	PIDTick(HAL_GetTick());
	if (tick%100==0)
	{
		HAL_TIM_IC_Stop_IT(&htim16, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
		htim16.Instance->CNT = 0;
		HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
		
	}
}

void Move(const struct MotorConfig *config)
{
	static int8_t boot = 1;
	int max = abs(config->LW) < abs(config->RW) ? abs(config->RW) : abs(config->LW);
	
	if (max==0)
	{
		Motors[0].Steps = 0;
		Motors[0].Move = 1;
		Motors[1].Steps = 0;
		Motors[1].Move = 1;
		PIDStop(0, BRAKE_ON);
		PIDStop(1, BRAKE_ON);
		return;
	}
	
	float lw = config->LW * (WheelSigns[0]); 
	float rw = config->RW * (WheelSigns[1]); 
	float vl = config->Speed * lw / max;
	float vr = config->Speed * rw / max;
	
	Motors[0].Dir = vl>0 ? 1 : -1;
	Motors[1].Dir = vr>0 ? 1 : -1;
	Motors[0].Move = 1;
	Motors[1].Move = 1;
	
	if (fabs(vl)<EPSILON_F)
	{
		Motors[0].Steps = 0;
		PIDStop(0, BRAKE_ON);
	}
	else
	{
		Motors[0].Steps = (int)(lw / DPP);
		PIDStart(0, vl, boot);
	}
	
	if (abs(vr)<0.000001f)
	{
		Motors[1].Steps = 0;
		PIDStop(0, BRAKE_ON);
	}
	else
	{
		Motors[1].Steps = (int)(rw / DPP);
		PIDStart(1, vr, boot);
	}
	boot = 0;
}

void SetTestMode(uint8_t on)
{
	TestModeEnable = on;
	if (on)
	{
		Motors[0].WheelDistance = 0;
		Motors[1].WheelDistance = 0;
	}
}

void CommandProcess(enum CommandEnum command, const void *params)
{
	switch (command)
	{
		case SetMotorCommand:
			if (TestModeEnable)
			{
				Move((const struct MotorConfig *)params);
				printf("\x1A");
			}
			else
				printf("TestMode Off\x1A");
			fflush(stdout);
			break;
		case GetMotorCommand:
			if (TestModeEnable)
			{
				sprintf(Str,"LeftWheel_PositionInMM,%d\nLeftWheel_Speed,%d\nRightWheel_PositionInMM,%d\nRightWheel_Speed,%d\nObstacle_Distance,%d\n\x1A", 
					(int)Motors[0].WheelDistance, (int)Motors[0].CurrentSpeed,
					(int)Motors[1].WheelDistance, (int)Motors[1].CurrentSpeed, ObstacleDistance);
				uint16_t size = strlen(Str);
				while (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)Str, size)==HAL_BUSY)
					__nop();
			}
			else
			{
				printf("TestMode Off\x1A");
				fflush(stdout);
			}
			break;
		case TestModeCommand:
			SetTestMode(*(uint8_t *)params);
			printf("\x1A");
			fflush(stdout);
			break;
		case InvalidCommand:
			printf("Invalid Command\x1A");
			fflush(stdout);
			break;
	}
}

//Ultrasonic Counter Update
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t us_count_start = 0;
	static int us_count = 0;
	
	if (htim!=&htim16)
		return;
	
	if (us_count_start)
	{
		us_count_start = 0;
		us_count = htim16.Instance->CCR1 - us_count;
		if (us_count>0 && us_count<600)
			ObstacleDistance = us_count;
		else
			ObstacleDistance = 0;
		HAL_TIM_IC_Stop_IT(&htim16, TIM_CHANNEL_1);
		htim16.Instance->CNT = 0;
	}
	else
	{
		us_count_start = 1;
		us_count = htim16.Instance->CCR1;
	}
}

//Timer for speed with 10us period
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t stepCounts[2];
	static uint32_t dt[2] = { 0, 0 };
	static uint8_t boot = 1;
	if (htim != &htim14)
		return;
	if (boot)
	{
		stepCounts[0] = htim1.Instance->CNT;
		stepCounts[1] = htim3.Instance->CNT;
		boot = 0;
		return;
	}
	for(int i=0;i<2;++i)
	{
		int delta = GetDeltaCount(i, &stepCounts[i]);
		dt[i]+=10;
		if (delta==0 && dt[i]<100000)
			continue;
		if (Motors[i].Move)
			Motors[i].Steps -= delta;
		float dd = delta * DPP;
		Motors[i].WheelDistance += WheelSigns[i] * dd;
		Motors[i].CurrentSpeed = dd*1000000/dt[i];
		dt[i] = 0;
	}
	
	//Ultrasonic sensor PWM counter
	//UltrasonicCounterUpdate();
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	memset(Motors, 0, sizeof(struct MotorState)*2);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();

  /* USER CODE BEGIN 2 */
	SystemCoreClockUpdate();
	
	HAL_Delay(500);
	
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	
	HAL_TIM_Base_Start_IT(&htim14);
	
	sprintf(Str, "System Started...\r\n\x1A");
	uint16_t size = strlen(Str);
	while (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)Str, size)==HAL_BUSY)
		__nop();
	
	//memset((void *)rx_buffer, 0, RX_BUFFER_SIZE);
	//HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx_buffer, RX_BUFFER_SIZE);
	//HAL_UART_Receive_IT(&huart2, rx_buffer, 0x100);
	
//	GetDeltaCount(0, &StepCounts[0]);
//	GetDeltaCount(0, &StepCounts[1]);
	//PIDStart(0, 600, 1);
//	PWMDutyChanged(1,0.1f);
//	PWMDutyChanged(0,0.2f);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t renew = 1;
	int offset = 0;
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		Update();
		
		if (rx_index>=RX_BUFFER_SIZE || rx_buffer[rx_index] == 0)
		{
			if (renew && huart2.State!=HAL_UART_STATE_BUSY_TX && huart2.State!=HAL_UART_STATE_BUSY_TX_RX)
			{
				rx_index=0;
				offset = 0;
				HAL_UART_DMAStop(&huart2);
				memset((void *)rx_buffer, 0, RX_BUFFER_SIZE);
				HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx_buffer, RX_BUFFER_SIZE);
				renew = 0;
			}
			continue;
		}
		
		if (rx_buffer[rx_index] == '\n' || rx_index >= RX_BUFFER_SIZE-1)
		{
			int len = rx_index - offset;
			if (rx_index>0 && rx_buffer[rx_index-1] == '\r')
				--len;
			CommandParse((uint8_t *)rx_buffer+offset, len, &CommandProcess);
			offset = rx_index+1;
			renew = 1;
		}
		++rx_index;
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_RCC_EnableCSS();

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  HAL_TIM_Encoder_Init(&htim1, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  HAL_TIM_Encoder_Init(&htim3, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* TIM14 init function */
void MX_TIM14_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 9;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim14);

  HAL_TIM_OC_Init(&htim14);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1);

}

/* TIM15 init function */
void MX_TIM15_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 47;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim15);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim15);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_MspPostInit(&htim15);

}

/* TIM16 init function */
void MX_TIM16_Init(void)
{

  TIM_IC_InitTypeDef sConfigIC;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 2399;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim16);

  HAL_TIM_IC_Init(&htim16);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC14 PC15 PC0 
                           PC1 PC2 PC3 PC4 
                           PC5 PC8 PC10 PC11 
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA10 
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PF4 PF5 PF6 PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_Pin */
  GPIO_InitStruct.Pin = STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB3 
                           PB4 PB5 PB6 PB7 
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR1_Pin DIR2_Pin */
  GPIO_InitStruct.Pin = DIR1_Pin|DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIGGER_Pin */
  GPIO_InitStruct.Pin = TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(TRIGGER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIR1_Pin|DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
