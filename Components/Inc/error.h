/**
  ******************************************************************************
  * @file    error.h
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    24-Apr-2016
  * @brief   All errors specified for the project.
  ******************************************************************************
  * Project: P3D_firmware
  * Description:
  * This file contains all error codes, messages and tools to handle the
  * errors.
  */

#ifndef ERROR_H_
#define ERROR_H_

typedef enum
{
  _Success,
  _HALError,
  _OutOfRange,
  _UnitTestError,
  _IncompatibleArgs

} Error;

#endif
