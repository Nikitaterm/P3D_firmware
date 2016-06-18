/**
  ******************************************************************************
  * @file    motionController.c
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    29-May-2016
  * @brief   Source file of the motion controller module.
  ******************************************************************************
  */

// TODO: This module hasn't been tested yet!!!

#include "stm32f4xx_hal.h"

#include "motionController.h"
#include "stepMotor.h"

#include "math.h"

/** Specifies the length (in mm) of all mechanism elements.
  * Please, refer to the Kinematic.png file for the
  * graphic representation.
  */
#define OD 241.0996
#define OE 241.0996
#define DF 284.2327
#define EG 284.2327
#define OOD 82
#define OOE 82
/** Specifies the transformation values of all axises.
  */
#define Kx 72  // =360/screw_step (in mm)
#define Ky 72  // =360/screw_step (in mm)
#define Kz 120  // =360/screw_step (in mm)
/** Specifies the angle's values (in grads) of the elements
  * DF and GE in the initial position.
  */
#define Ix 90 - 2.3188
#define Iy 90 - 2.3188

double curr_X, curr_Y, curr_Z;

void InitEndStops()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  // Init the GPIO PORTE.5 for the X-axis end stop.
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Init the GPIO PORTE.6 for the Y-axis end stop.
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Init the GPIO PORTE.7 for the Z-axis end stop.
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

static double GetXAngle(double X, double Y, double Z)
{
  return X*Kx-Ix+acos((EG*EG-OE*OE+(Y+OOE)*(Y+OOE)+Z*Z)/(2*EG*sqrt((Y+OOE)*(Y+OOE)+Z*Z)))+atan(Z/Y);
}

static double GetYAngle(double X, double Y, double Z)
{
  return Y*Ky-Iy+acos((DF*DF-OD*OD+(X+OOD)*(X+OOD)+Z*Z)/(2*DF*sqrt((X+OOD)*(X+OOD)+Z*Z)))+atan(Z/X);
}

static double GetZAngle(double Z)
{
  return Z*Kz;
}

/* Public functions */

Error InitAllMotors(void)
{
  InitEndStops();
  return SMotorDriversInit();
}

Error DeInitAllMotors(void)
{
  return SMotorDriversDeInit();
}

Error EnableAllMotors(void)
{
  Error err = _Success;
  err = EnableMotor(_X);
  if (err != _Success) goto e;
  err = EnableMotor(_Y);
  if (err != _Success) goto e;
  err = EnableMotor(_Z);
  if (err != _Success) goto e;
  return _Success;
  e:
  return err;
}

Error DisabeAllMotors(void)
{
  Error err = _Success;
  err = DisableMotor(_X);
  if (err != _Success) goto e;
  err = DisableMotor(_Y);
  if (err != _Success) goto e;
  err = DisableMotor(_Z);
  if (err != _Success) goto e;
  return _Success;
  e:
  return err;
}

Error ZeroOutPosition(void)
{
  Error err = _Success;
  err = ZeroOutAngleCounter(_X);
  if (err != _Success) goto e;
  err = ZeroOutAngleCounter(_Y);
  if (err != _Success) goto e;
  err = ZeroOutAngleCounter(_Z);
  if (err != _Success) goto e;
  curr_X = 0;
  curr_Y = 0;
  curr_Z = 0;
  return _Success;
  e:
  return err;
}

Error GoToRefer(void)
{
  int8_t ready = 0;
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;
  while(ready != 0x07)
  {
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_SET) MotorSetSpeedAndValue(_X, 300, --x); else ready |= (1<<0);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_SET) MotorSetSpeedAndValue(_Y, 300, --y); else ready |= (1<<1);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7) == GPIO_PIN_SET) MotorSetSpeedAndValue(_Z, 300, --z); else ready |= (1<<2);
    while(IsMotorBusy(_X));
    while(IsMotorBusy(_Y));
    while(IsMotorBusy(_Z));
  }
  return _Success;
}

double XX, YY, ZZ;
Error GoToWithSpeed(double X, double Y, double Z, double speed)
{
  double l = sqrt((X-curr_X)*(X-curr_X)+(Y-curr_Y)*(Y-curr_Y)+(Z-curr_Z)*(Z-curr_Z));
  double cos_a = (X-curr_X)/l;
  double cos_b = (Y-curr_Y)/l;
  double cos_c = (Z-curr_Z)/l;

  XX = GetXAngle(X,Y,Z);
  YY = GetYAngle(X,Y,Z);
  ZZ = GetZAngle(Z);

  Error err = _Success;
  err = MotorSetSpeedAndValue(_X, speed*cos_a/Kx, GetXAngle(X,Y,Z));
  if (err != _Success) goto e;
  err = MotorSetSpeedAndValue(_Y, speed*cos_b/Ky, GetYAngle(X,Y,Z));
  if (err != _Success) goto e;
  err = MotorSetSpeedAndValue(_Z, speed*cos_c/Kz, GetZAngle(Z));
  if (err != _Success) goto e;

  while(IsMotorBusy(_X));
  while(IsMotorBusy(_Y));
  while(IsMotorBusy(_Z));

  return _Success;
  e:
  return err;
}
