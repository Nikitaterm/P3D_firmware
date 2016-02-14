/**
  ******************************************************************************
  * @file    tempSensors.h
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    14-Feb-2016
  * @brief   Header file of temperature sensors driver.
  ******************************************************************************
  * Project: P3D_firmware
  * Description:
  * This module support temperature sensors connected to the board.
  * PARAMETERS:
  *   sensor type: k-type thermocouple
  * REQUIRED RESOURCES:
  *   ADC1
  *   DMA2: stream0, channel 0
  */

/**
  * @brief Define supported channels
  * Add new items to support more channels
  */
typedef enum
{
  H_END1  /// channel 0: the first hot end
  /* Insert a new channel here */

} Channel;

/**
  * @brief Configure the driver
  */
void tempSensorsInit(void);

/**
  * @brief De-init the driver
  */
void tempSensorsDeInit(void);

/**
  * @brief Run the driver
  */
void tempSensorsRun(void);

/**
  * @brief Run the driver
  */
double tempSensorsGet(Channel ch);
