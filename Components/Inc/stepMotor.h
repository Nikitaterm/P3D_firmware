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
  _X,
  _Y
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
Error EnableMotor(Axis axis);

/**
  * @brief Disable a motor
  */
Error DisableMotor(Axis axis);

/**
  * @brief Set speed and position for the specified axis.
  * This function also launches the motor
  */
Error MotorSetSpeedAndValue(Axis axis, double rpm, double angle);

/**
  * @brief Stop a motor
  */
Error StopMotor(Axis axis);

/**
  * @brief Zero out current angle counter value.
  * Call this function by the end-stop signal
  */
Error ZeroOutAngleCounter(Axis axis);

/**
  * @brief Check whether a motor is still working.
  * Use this function before giving the next task to the motor
  */
uint8_t IsMotorBusy(Axis axis);

/**
  * @brief Get current angle of the specified motor
  */
double MotorGetAngle(Axis axis);

#endif
