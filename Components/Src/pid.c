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

/* Private functions */
void PID_Init(PID_HandleTypeDef* handle)
{
  handle->init();
  handle->out.init();
  handle->fb.init();
}

void PID_run(PID_HandleTypeDef* handle) {
  handle->fb.start();
  handle->out.start();
}

void PID_stop(PID_HandleTypeDef* handle) {
  handle->fb.stop();
  handle->out.stop();
}

void PID_DeInit(PID_HandleTypeDef* handle)
{
  handle->out.deInit();
  handle->fb.deInit();
}

void PID_SetTargetvalue(PID_HandleTypeDef* handle, double val) {
  handle->config.target_val = val;
}

double cur_temp, e;
double result;
void PID_update(PID_HandleTypeDef* handle, uint8_t fb_channel) {
  cur_temp = handle->fb.getValue(0);
	e = handle->config.target_val - handle->fb.getValue(fb_channel);
	double Max_S_i = 25*handle->config.target_val;
	if ( (!( (handle->S_i >= Max_S_i) && (e >= 0) ) ) &&
                        (!( (handle->S_i <= 0) && (e <= 0) ) ) )
    handle->S_i += e;
	result = (1/handle->config.M_k)*(handle->config.P_k*e +
                        handle->config.I_k*handle->S_i + handle->config.D_k*(e - handle->e_pr));
	if (result >= handle->config.RES_max)
		result = handle->config.RES_max;
	if (result <= 0)
		result = 0;
	handle->e_pr = e;
	handle->out.setValue(result);
}
