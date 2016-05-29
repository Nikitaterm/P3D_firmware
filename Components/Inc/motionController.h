/**
  ******************************************************************************
  * @file    motionController.h
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    29-May-2016
  * @brief   Header file of the motion controller module.
  ******************************************************************************
  * Project: P3D_firmware
  * Description:
  * This module provides all algorithms for track planning and
  * control.
  */

#ifndef _MOTION_CONTROLLER_H_
#define _MOTION_CONTROLLER_H_

#include "error.h"

/**
  * @brief Init all motors
  */
Error InitAllMotors(void);

/**
  * @brief Deinit all motors
  */
Error DeInitAllMotors(void);

/**
  * @brief Enable all motors
  */
Error EnableAllMotors(void);

/**
  * @brief Disable all motors
  */
Error DisableAllMotors(void);

/**
  * @brief Zero out all coordinates.
  * Call this function by the end-stop signal
  */
Error ZeroOutPosition(void);

/**
  * @brief Move to the specified point with the specified speed
  */
Error GoToWithSpeed(double X, double Y, double Z, double speed);

#endif
