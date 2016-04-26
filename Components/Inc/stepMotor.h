/**
  ******************************************************************************
  * @file    stepMotor.h
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    24-Apr-2016
  * @brief   Header file of step motor drivers.
  ******************************************************************************
  * Project: P3D_firmware
  * Description:
  * Step motor driver implements low-level control interfaces suitable
  * for middl- and high- level layers.
  */

#ifndef STEP_MOTOR_H_
#define STEP_MOTOR_H_

#include "error.h"

/**
  * @brief Possible axises
  */
typedef enum
{
  _X
} Axis;

/**
  * @brief Init all drivers
  */
Error SMotorDriversInit(void);

/**
  * @brief Deinit all drivers
  */
Error SMotorDriversDeInit(void);

/**
  * @brief Enable a motor
  */
void EnableMotor(Axis axis);

/**
  * @brief Disable a motor
  */
void DisableMotor(Axis axis);

/**
  * @brief Enable a motor
  */
Error StopMotor(Axis axis);

/**
  * @brief Set speed for the specified axis and start it
  */
Error SetSpeedAndStart(Axis axis, double rpm);

/**
  * @brief Zero out the current angle counter value
  */
void ZeroOutAngleCounter(Axis axis);

/**
  * @brief Get current angle
  */
double GetAngle(Axis axis);

#endif
