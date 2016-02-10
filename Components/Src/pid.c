/**
  ******************************************************************************
  * @file    pid.c
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    09-Feb-2016
  * @brief   Source file of pid controller.
  ******************************************************************************
  */

#include "pid.h"
#include "termocoupleLookUp.h"

/* Private constant variables */

/* Private configurable variables */
static PID_InitTypeDef config;

/* Private variables */

/* Private functions */

/* Public functions */
void PID_Init(PID_InitTypeDef* init_config)
{
  config = *init_config;
}
