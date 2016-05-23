/**
  ******************************************************************************
  * @file    stepMotor_test.c
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    24-Apr-2016
  * @brief   Source file of step motor drivers.
  ******************************************************************************
  */

#include "stepMotor_test.h"

#include "cmsis_os.h"

#include "error.h"
#include "stepMotor.h"

static Error assertTrue(uint8_t exp)
{
  return exp == 1 ? _Success: _UnitTestError;
}

static Error PrepareForTests()
{
  if (SMotorDriversInit() != _Success) return _UnitTestError;
  return _Success;
}

static Error FinishTests()
{
  if (SMotorDriversDeInit() != _Success) return _UnitTestError;
  return _Success;
}

/**
  * @brief This test checks correctness of the angle feed back implementation.
  * In case of the assertion error, check accuracy of the vTaskDelay() function first.
  */
static Error TestAngleFeedBack()
{
  Error err;
  EnableMotor(_X);
  err = SetSpeedAndValue(_X, 100, 360);
  if (err != _Success) goto e;
  while(IsMotorBusy(_X));
  err = assertTrue(MotorGetAngle(_X) == 360);
  if (err != _Success) goto e;
  err = SetSpeedAndValue(_X, 220, 720);
  if (err != _Success) goto e;
  while(IsMotorBusy(_X));
  err = assertTrue(MotorGetAngle(_X) == 720);
  if (err != _Success) goto e;
  err = SetSpeedAndValue(_X, 220, -720);
  if (err != _Success) goto e;
  while(IsMotorBusy(_X));
  err = assertTrue(MotorGetAngle(_X) == -720);
  if (err != _Success) goto e;
  err = SetSpeedAndValue(_X, 220, 0);
  if (err != _Success) goto e;
  while(IsMotorBusy(_X));
  err = StopMotor(_X);
  if (err != _Success) goto e;
  DisableMotor(_X);
  return assertTrue(MotorGetAngle(_X) == 0);
  e:
  return err;
}

// TODO: more tests!
Error MotorTestAll(void)
{
  if (PrepareForTests() != _Success) return _UnitTestError;
  if (TestAngleFeedBack() != _Success) return _UnitTestError;
  if (FinishTests() != _Success) return _UnitTestError;
  return _Success;  
}
