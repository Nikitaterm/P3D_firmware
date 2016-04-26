/**
  ******************************************************************************
  * @file    stepMotor.c
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    24-Apr-2016
  * @brief   Source file of step motor drivers.
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"
#include "stepMotor.h"

#define TIM_CLK 16          // Specifies the driver's timers clock, MHz
#define MOTOR_STEP_DIV 16   // Specifies the motors step division value
#define MOTOR_STEP_DG 1.8   // Specifies the motor step, grad

#define ANGLE_IRQ_PR_PRIORITY  1   // Specifies the preempt priority for TIM IRQ
#define ANGLE_IRQ_SUB_PRIORITY 0   // Specifies the sub priority for TIM IRQ

/**
  * @brief Step motor driver's handler
  */
typedef struct
{
  TIM_HandleTypeDef htim;         /**< Timer to be used */

  TIM_OC_InitTypeDef sConfigOC;   /**< Configuration structure */

  int64_t angle;                  /**< Stores the current angle value, in steps */

  int8_t dir;                     /**< Stores the current direction */

} StMotor_HandleTypeDef;

StMotor_HandleTypeDef X_driver;   // X-axis driver 's handler
void TIM1_BRK_TIM9_IRQHandler(void)
{
  __HAL_TIM_CLEAR_FLAG(&X_driver.htim, TIM_FLAG_CC1);
  X_driver.angle += X_driver.dir;
}

static void InitIRQ(void)
{
  // X-axis
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, ANGLE_IRQ_PR_PRIORITY, ANGLE_IRQ_SUB_PRIORITY);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
}

static Error InitOutput(void)
{
  // X-axis
  X_driver.htim.Instance = TIM9;
  X_driver.htim.Init.CounterMode = TIM_COUNTERMODE_UP;
  X_driver.htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&X_driver.htim) != HAL_OK) return _HALError;

  X_driver.sConfigOC.OCMode = TIM_OCMODE_PWM1;
  X_driver.sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  X_driver.sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  return _Success;
}

static void DeInitIRQ(void)
{
  // X-axis
  HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
}

static Error DeInitOutput(void)
{
  // X-axis
  if (HAL_TIM_Base_DeInit(&X_driver.htim) != HAL_OK) return _HALError;
  if (HAL_TIM_PWM_DeInit(&X_driver.htim) != HAL_OK) return _HALError;
  return _Success;
}

/* Public functions */
Error SMotorDriversInit()
{
  InitIRQ();
  return InitOutput();
}

Error SMotorDriversDeInit()
{
  DeInitIRQ();
  return DeInitOutput();
}

void EnableMotor(Axis axis)
{
  // X-axis
  if (axis == _X)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  }
}

void DisableMotor(Axis axis)
{
  // X-axis
  if (axis == _X)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  }
}

static Error StartMotor(Axis axis)
{
  // X-axis
  if (axis == _X)
  {
    if (HAL_TIM_PWM_ConfigChannel(&X_driver.htim, &X_driver.sConfigOC, TIM_CHANNEL_1)
           != HAL_OK) return _HALError;
    if (HAL_TIM_PWM_Init(&X_driver.htim) != HAL_OK) return _HALError;
    if (HAL_TIM_PWM_Start_IT(&X_driver.htim, TIM_CHANNEL_1) != HAL_OK) return _HALError;
  }
  return _Success;
}

Error StopMotor(Axis axis)
{
  // X-axis
  if (axis == _X)
  {
    if (HAL_TIM_PWM_Stop_IT(&X_driver.htim, TIM_CHANNEL_1) != HAL_OK) return _HALError;
    __HAL_TIM_CLEAR_FLAG(&X_driver.htim, TIM_FLAG_CC1);
  }
  return _Success;
}

Error SetSpeedAndStart(Axis axis, double rpm)
{
  Error err = _Success;
  if (rpm < 0)
  {
    rpm *= -1;
    X_driver.dir = -1;
  }
  else
  {
    X_driver.dir = 1;
  }
  uint32_t frq = (uint32_t)(TIM_CLK*1000000*MOTOR_STEP_DG/(6*rpm*MOTOR_STEP_DIV));
  if ( (frq <= 0) || (frq > 0xffff) )
  {
    return _OutOfRange;
  }
  err = StopMotor(axis);
  if (err != _Success) return err;
  if (axis == _X)
  {
    X_driver.htim.Init.Period = (uint16_t)frq;
    X_driver.sConfigOC.Pulse = (uint16_t)(frq/2);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (GPIO_PinState)(X_driver.dir + 1));
  }
  err = StartMotor(axis);
  if (err != _Success) return err;
  return _Success;
}

void ZeroOutAngleCounter(Axis axis)
{
  // X-axis
  if (axis == _X)
  {
    X_driver.angle = 0;
  }
}

double GetAngle(Axis axis)
{
  // X-axis
  if (axis == _X)
  {
    return X_driver.angle * MOTOR_STEP_DG/MOTOR_STEP_DIV;
  }
}
