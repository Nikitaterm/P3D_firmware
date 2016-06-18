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

#define TIM_CLK 168          // Specifies the driver's timers clock, MHz
/** Specifies the rpm value when we have to increase the timer prescaler.
  * The value should be calculated considering the rpm point of the minimum
  * posible rotation speed with the prescaler = 1 according to the following
  * expression (rounded up to the next integer):
  * Vmin = TIM_CLK*10^6*MOTOR_STEP_DG/(65535*6*MOTOR_STEP_DIV)
  */
#define RPM_PRSC_POINT 49
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

  int64_t set_angle;              /**< Stores set angle value, in steps */

  int64_t angle;                  /**< Stores the current angle value, in steps */

  uint8_t busy;                   /**< The task is being executed */

  int8_t dir;                     /**< Stores the current direction */

} StMotor_HandleTypeDef;

static StMotor_HandleTypeDef X_driver;   // X-axis driver's handler
void TIM1_BRK_TIM9_IRQHandler(void)
{
  __HAL_TIM_CLEAR_FLAG(&X_driver.htim, TIM_FLAG_CC1);
  X_driver.angle += X_driver.dir;
  if (X_driver.angle == X_driver.set_angle)
  {
    StopMotor(_X);
  }
}

static StMotor_HandleTypeDef Y_driver;          // Y-axis driver's handler
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  __HAL_TIM_CLEAR_FLAG(&Y_driver.htim, TIM_FLAG_CC1);
  Y_driver.angle += Y_driver.dir;
  if (Y_driver.angle == Y_driver.set_angle)
  {
    StopMotor(_Y);
  }
}

static StMotor_HandleTypeDef Z_driver;          // Z-axis driver's handler
void TIM1_UP_TIM10_IRQHandler(void)
{
  __HAL_TIM_CLEAR_FLAG(&Z_driver.htim, TIM_FLAG_CC1);
  Z_driver.angle += Z_driver.dir;
  if (Z_driver.angle == Z_driver.set_angle)
  {
    StopMotor(_Z);
  }
}

static void InitIRQ(void)
{
  // X-axis
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, ANGLE_IRQ_PR_PRIORITY, ANGLE_IRQ_SUB_PRIORITY);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

  // Y-axis
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, ANGLE_IRQ_PR_PRIORITY, ANGLE_IRQ_SUB_PRIORITY);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

  // Z-axis
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, ANGLE_IRQ_PR_PRIORITY, ANGLE_IRQ_SUB_PRIORITY);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

static Error InitOutput(StMotor_HandleTypeDef *driver, TIM_TypeDef *timer)
{
  driver->htim.Instance = timer;
  driver->htim.Init.CounterMode = TIM_COUNTERMODE_UP;
  driver->htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&driver->htim) != HAL_OK) return _HALError;

  driver->sConfigOC.OCMode = TIM_OCMODE_PWM1;
  driver->sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  driver->sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&driver->htim, &driver->sConfigOC, TIM_CHANNEL_1)
      != HAL_OK) return _HALError;
  if (HAL_TIM_PWM_Init(&driver->htim) != HAL_OK) return _HALError;

  return _Success;
}

static void DeInitIRQ(void)
{
  // X-axis
  HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);

  //Y-axis
  HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);

  //Z-axis
  HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
}

static Error DeInitOutput(StMotor_HandleTypeDef *driver)
{
  if (HAL_TIM_Base_DeInit(&driver->htim) != HAL_OK) return _HALError;
  if (HAL_TIM_PWM_DeInit(&driver->htim) != HAL_OK) return _HALError;
  return _Success;
}

static Error MotorStart(StMotor_HandleTypeDef* driver)
{
  if (HAL_TIM_PWM_Start_IT(&driver->htim, TIM_CHANNEL_1) != HAL_OK) return _HALError;
  driver->busy = 1;
  return _Success;
}

static Error MotorStop(StMotor_HandleTypeDef* driver)
{
  if (HAL_TIM_PWM_Stop_IT(&driver->htim, TIM_CHANNEL_1) != HAL_OK) return _HALError;
  __HAL_TIM_CLEAR_FLAG(&driver->htim, TIM_FLAG_CC1);
  driver->busy = 0;
  return _Success;
}

static Error SetSpeedAndValue(StMotor_HandleTypeDef* driver, double rpm, double angle)
{
  if (rpm < 0)
  {
    return _OutOfRange;
  }
  Error err = _Success;
  driver->set_angle = (int32_t)(angle*MOTOR_STEP_DIV/MOTOR_STEP_DG);
  if (driver->set_angle - driver->angle < 0)
  {
    driver->dir = -1;
  }
  else if (driver->set_angle - driver->angle > 0)
  {
    driver->dir = 1;
  } else if (rpm == 0)
  {
    return _Success;  // We don't need to move
  } else
  {
    return _IncompatibleArgs;
  }
  if (rpm == 0)
  {
    return _IncompatibleArgs;
  }
  uint32_t prsc = (uint32_t)(RPM_PRSC_POINT/rpm);
  if (prsc > 0xffff) prsc = 0xffff;
  uint32_t frq = (uint32_t)(TIM_CLK*1000000*MOTOR_STEP_DG/(6*rpm*MOTOR_STEP_DIV*(prsc+1)));
  if ( (frq <= 0) || (frq > 0xffff) )
  {
    return _OutOfRange;
  }
  driver->htim.Init.Prescaler = prsc;
  driver->htim.Init.Period = (uint16_t)frq;
  driver->sConfigOC.Pulse = (uint16_t)(frq/2);
  if (driver == &X_driver)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (GPIO_PinState)(driver->dir + 1));
  }
  if (driver == &Y_driver)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (GPIO_PinState)(driver->dir + 1));
  }
  if (driver == &Z_driver)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (GPIO_PinState)(driver->dir + 1));
  }
  driver->htim.Instance->PSC = driver->htim.Init.Prescaler;
  driver->htim.Instance->ARR = driver->htim.Init.Period;
  driver->htim.Instance->CCR1 = driver->sConfigOC.Pulse;
  if (HAL_TIM_PWM_Start_IT(&driver->htim, TIM_CHANNEL_1) != HAL_OK) return _HALError;
  driver->busy = 1;
  if (err != _Success) return err;
  return _Success;
}

/* Public functions */
Error SMotorDriversInit()
{
  InitIRQ();
  Error err;
  err = InitOutput(&X_driver, TIM9);
  if (err != _Success) goto e;
  err = InitOutput(&Y_driver, TIM11);
  if (err != _Success) goto e;
  err = InitOutput(&Z_driver, TIM10);
  if (err != _Success) goto e;
  return _Success;
  e:
  return err;
}

Error SMotorDriversDeInit()
{
  DeInitIRQ();
  Error err;
  err = DeInitOutput(&X_driver);
  if (err != _Success) goto e;
  err = DeInitOutput(&Y_driver);
  if (err != _Success) goto e;
  err = DeInitOutput(&Z_driver);
  if (err != _Success) goto e;
  return _Success;
  e:
  return err;
}

Error EnableMotor(Axis axis)
{
  switch(axis)
  {
    case _X: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); break;
    case _Y: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); break;
    case _Z: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); break;
    default: return _OutOfRange;
  }
  return _Success;
}

Error DisableMotor(Axis axis)
{
  switch(axis)
  {
    case _X: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); break;
    case _Y: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); break;
    case _Z: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); break;
    default: return _OutOfRange;
  }
  return _Success;
}

Error StartMotor(Axis axis)
{
  switch(axis)
  {
    case _X: return MotorStart(&X_driver);
    case _Y: return MotorStart(&Y_driver);
    case _Z: return MotorStart(&Z_driver);
    default: return _OutOfRange;
  }
}

Error StopMotor(Axis axis)
{
  switch(axis)
  {
    case _X: return MotorStop(&X_driver);
    case _Y: return MotorStop(&Y_driver);
    case _Z: return MotorStop(&Z_driver);
    default: return _OutOfRange;
  }
}

Error MotorSetSpeedAndValue(Axis axis, double rpm, double angle)
{
  switch(axis)
  {
    case _X: return SetSpeedAndValue(&X_driver, rpm, angle);
    case _Y: return SetSpeedAndValue(&Y_driver, rpm, angle);
    case _Z: return SetSpeedAndValue(&Z_driver, rpm, angle);
    default: return _OutOfRange;
  }
}

Error ZeroOutAngleCounter(Axis axis)
{
  switch(axis)
  {
    case _X: X_driver.angle = 0; break;
    case _Y: Y_driver.angle = 0; break;
    case _Z: Z_driver.angle = 0; break;
    default: return _OutOfRange;
  }
  return _Success;
}

uint8_t IsMotorBusy(Axis axis)
{
  switch(axis)
  {
    case _X: return X_driver.busy;
    case _Y: return Y_driver.busy;
    case _Z: return Z_driver.busy;
    default: return _OutOfRange;
  }
}

double MotorGetAngle(Axis axis)
{
  switch(axis)
  {
    case _X: return X_driver.angle*MOTOR_STEP_DG/MOTOR_STEP_DIV;
    case _Y: return Y_driver.angle*MOTOR_STEP_DG/MOTOR_STEP_DIV;
    case _Z: return Z_driver.angle*MOTOR_STEP_DG/MOTOR_STEP_DIV;
    // default: return _OutOfRange;  // TODO: think!
  }
}
