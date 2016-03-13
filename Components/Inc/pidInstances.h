/**
  ******************************************************************************
  * @file    pidInstances.h
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    13-Mar-2016
  * @brief   Header file of pid instances module.
  ******************************************************************************
  * Project: P3D_firmware
  * Description:
  * This module contains interfaces and implementations for all pid
  * controllers in the system.
  */

#ifndef PID_INSTANCES_H_
#define PID_INSTANCES_H_

#include "pid.h"

/* PID instances */
typedef enum {
  MainHotEnd

} PID_instance;

/**
  * @brief Init the pid instance
  */
void PidInitInstance(PID_instance instance);

/**
  * @brief Run the pid instance
  */
void PidRunInstance(PID_instance instance);

/**
  * @brief Stop the pid instance
  */
void PidStopInstance(PID_instance instance);

/**
  * @brief Deinit the pid instance
  */
void PidDeInitInstance(PID_instance instance);

/**
  * @brief Set the target value for the pid instance specified
  */
void PidSetTargetValue(PID_instance instance, double val);

/**
  * @brief Iterate the pid instance
  * IMPORTANT!
  * Call this function inside an infinite loop. The required pid call
  * frequency is achieved by this method. It isn't necessary to do
  * delay when calling the method.
  */
void PidIterate(PID_instance instance);

#endif
