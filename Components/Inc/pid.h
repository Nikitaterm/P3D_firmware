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
  * The pid controller is designed to implement high-precision temperature control.
  * The code contains both the controller and a temperature measurement init. It's
  * designed to operate with k-type thermocouples.
  */

#include <stdint.h>

/**
  * @brief PID Init structure definition
  */
typedef struct
{
  uint32_t OUTPUT_Pin;	/**< Specifies the output GPIO pin (heat element PWM output)
                          */

  uint32_t FB_Pin;      /**< Specifies the temperature feed back input GPIO pin
                             (analog thermocouple pin) */

  uint32_t CALL_frq;    /**< Specifies the pid controller call frequency, Hz */

  double AMPL_k;        /**< Specifies the thermocouple amplifier gain coefficient */

  double M_AVG_k;       /**< Specifies the thermocouple feed back mooving average
                             filter coefficient	*/

  double P_k;           /**< Pid P-coefficient */

  double I_k;           /**< Pid I-coefficient */

  double D_k;           /**< Pid D-coefficient */

  double I_SUM_max;     /**< Specifies the max value of the integral sum */

  uint32_t OUTPUT_PWM_frq;  /**< Specifies the output PWM frequency, Hz */

} PID_InitTypeDef;

/* Functions */

/**
  * @brief Configure the pid controller.
  */
void PID_Init(PID_InitTypeDef* init_config);
