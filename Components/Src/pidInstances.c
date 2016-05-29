/**
  ******************************************************************************
  * @file    pidInstances.c
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    13-Mar-2016
  * @brief   Source file of pid instances module.
  ******************************************************************************
  */

#include "pidInstances.h"

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "tempSensors.h"

#include "pid.h"

/****************************************************< pid instances */

#define PID_INST_AMOUNT 1
static PID_HandleTypeDef pid_instances[PID_INST_AMOUNT];  // The instances

/** The instance mainHotEnd.
  * This instance controls the main hot end's temperature.
  */

static TIM_HandleTypeDef htim12;
static TIM_OC_InitTypeDef sConfigOC;

#define TIM12_CLK    84     // Specifies the TIM12 clock, MHz
#define TIM12_PRSC   10     // Specifies the TIM12 prescaler value

#define OUT_PWM_FRQ  1000   // Specifies the desired pwm output frequency, Hz

#define TIM12_PER TIM12_CLK*1000000/(TIM12_PRSC*OUT_PWM_FRQ)
#define MAX_PULSE TIM12_PER

/** Initializarion of the output channel for the controller mainHotEnd_*/
static void mainHotEnd_initOutput()
{
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = TIM12_PRSC;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = TIM12_PER;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim12);
  HAL_TIM_PWM_Init(&htim12);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = MAX_PULSE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);
}

static void mainHotEnd_startOutput()
{
  HAL_TIM_Base_Start(&htim12);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}

static void mainHotEnd_stopOutput()
{
  HAL_TIM_Base_Stop(&htim12);
  HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
}

/** setOutout function for the mainHotEnd_ controller */
static void mainHotEnd_setOutput(double val)
{
  sConfigOC.Pulse = MAX_PULSE*(1 - val);
  HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}

/** Initialize the pid instance */
void mainHotEnd_initInstance()
{
  // Initialize interfaces
  pid_instances[MainHotEnd].out.init = &mainHotEnd_initOutput;
  pid_instances[MainHotEnd].out.start = &mainHotEnd_startOutput;
  pid_instances[MainHotEnd].out.stop = &mainHotEnd_stopOutput;
  pid_instances[MainHotEnd].out.setValue = &mainHotEnd_setOutput;
  pid_instances[MainHotEnd].fb.init = &tempSensorsInit;
  pid_instances[MainHotEnd].fb.start = &tempSensorsRun;
  pid_instances[MainHotEnd].fb.stop = &tempSensorsStop;
  pid_instances[MainHotEnd].fb.getValue = &tempSensorGetValue;

  // Set coefficients
  pid_instances[MainHotEnd].config.M_k = 310;
  pid_instances[MainHotEnd].config.P_k = 40;
  pid_instances[MainHotEnd].config.I_k = 0.05;
  pid_instances[MainHotEnd].config.D_k = 5;

  // Set maximum values
  pid_instances[MainHotEnd].config.I_SUM_max = 7000;
  pid_instances[MainHotEnd].config.RES_max = 1;
}

/****************************************************< End of pid instances */

/* Public functions */
void PidInitInstance(PID_instance instance)
{
  // Set initializers
  switch(instance)
  {
    case MainHotEnd: pid_instances[instance].init = &mainHotEnd_initInstance; break;
  }
  // Perform initializers
  PID_Init(&pid_instances[instance]);
}

void PidRunInstance(PID_instance instance)
{
  PID_run(&pid_instances[instance]);
}

void PidStopInstance(PID_instance instance)
{
  PID_stop(&pid_instances[instance]);
}

void PidDeInitInstance(PID_instance instance)
{
  PID_DeInit(&pid_instances[instance]);
}

void PidSetTargetValue(PID_instance instance, double val)
{
  PID_SetTargetvalue(&pid_instances[instance], val);
}

void PidIterate(PID_instance instance)
{
  osDelay(1000/pid_instances[instance].config.CALL_frq);  //TODO: 1000?
  PID_update(&pid_instances[instance], H_END1);
}
