/**
  ******************************************************************************
  * @file    thermocoupleLookUp.h
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    09-Feb-2016
  * @brief   Header file with look-up tables for supported termocouples.
  ******************************************************************************
  */

/**
  * @brief Look-up table for k-type thermocouples.
  */
#define kTYPE_DELTA 10
static double k_type[33] =
  {0,	0.397, 0.798,	1.203, 1.612, 2.023, 2.436,	2.851, 3.267,	3.682,
   4.096, 4.509, 4.920,	5.328, 5.735,	6.138, 6.540,	6.941, 7.340,	7.739,
   8.138,	8.539, 8.940, 9.343, 9.747, 10.153, 10.561, 10.971, 11.382, 11.795,
   12.209, 12.624, 13.040};
