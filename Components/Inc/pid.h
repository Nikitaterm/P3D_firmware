/**
  ******************************************************************************
  * @file    pid.h
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    09-Feb-2016
  * @brief   Header file of pid controller.
  ******************************************************************************
  * Project: P3D_firmware
  * Description:
  * The pid controller is designed to implement high-precision control of various
  * parameters. This module contains an abstract interface for different types of
  * input (feed back) and output channels.
  */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

/**
  * @brief PID IO interface
  */
typedef struct
{
  void(*init)(void);

  void(*start)(void);

  void(*stop)(void);

  void(*setValue)(double);

  double(*getValue)(uint8_t);

  void(*deInit)(void);

} PID_IOInterface;

/**
  * @brief PID configuration structure definition
  */
typedef struct
{
  double target_val;    /**< Specifies the output value to achieve */

  uint32_t CALL_frq;    /**< Specifies the pid controller call frequency, Hz */

  double M_k;           /**< Pid main coefficient */

  double P_k;           /**< Pid P-coefficient */

  double I_k;           /**< Pid I-coefficient */

  double D_k;           /**< Pid D-coefficient */

  double I_SUM_max;     /**< Specifies the max value of the integral sum */

  double RES_max;       /**< Maximum result */

} PID_ConfigTypeDef;

/**
  * @brief PID handler structure definition
  */
typedef struct
{
  PID_ConfigTypeDef config;  /**< Config strucutre */

  PID_IOInterface out;       /**< Output interface */

  PID_IOInterface fb;        /**< Input interface */

  void(*init)(void);         /**< Initializer functon */

  float S_i;   /**< Integral sum */
  float e_pr;  /**< Previous error value */
} PID_HandleTypeDef;

/**
  * @brief Init the pid controller
  */
void PID_Init(PID_HandleTypeDef* handle);

/**
  * @brief Run the pid controller
  */
void PID_run(PID_HandleTypeDef* handle);

/**
  * @brief Stop the pid controller
  */
void PID_stop(PID_HandleTypeDef* handle);

/**
  * @brief Deinit the pid controller
  */
void PID_DeInit(PID_HandleTypeDef* handle);

/**
  * @brief Set the target value
  */
void PID_SetTargetvalue(PID_HandleTypeDef* handle, double val);

/**
  * @brief Iterate the pid controller
  */
void PID_update(PID_HandleTypeDef* handle, uint8_t fb_channel);

#endif
