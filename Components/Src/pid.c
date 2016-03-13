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

/** Possible statuses */
typedef enum
{
  NotInitialized,
  Stopped,
  Run

} Status;

static Status status = NotInitialized;

/* Private functions */
void PID_Init(PID_HandleTypeDef* handle)
{
  if (status == NotInitialized)
  { 
    handle->init();
    handle->out.init();
    handle->fb.init();

    status = Stopped;
  }
}

void PID_run(PID_HandleTypeDef* handle)
{
  if (status == Stopped)
  {
    handle->fb.start();
    handle->out.start();

    status = Run;
  }
}

void PID_stop(PID_HandleTypeDef* handle)
{
  if (status == Run)
  {
    handle->fb.stop();
    handle->out.stop();

    status = Stopped;
  }
}

void PID_DeInit(PID_HandleTypeDef* handle)
{
  if (status == Stopped)
  {
    handle->out.deInit();
    handle->fb.deInit();

    status = NotInitialized;
  }
}

void PID_SetTargetvalue(PID_HandleTypeDef* handle, double val)
{
  if (status == Run)
  {
    handle->config.target_val = val;
  }
}

double cur_temp, e;
double result;
void PID_update(PID_HandleTypeDef* handle, uint8_t fb_channel)
{
  if (status == Run)
  {
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
}
