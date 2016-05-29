/**
  ******************************************************************************
  * @file    motionController.c
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    29-May-2016
  * @brief   Source file of the motion controller module.
  ******************************************************************************
  */

// TODO: This module hasn't been tested yet!!!

#include "motionController.h"

#include "math.h"

#include "stepMotor.h"

/** Specifies the length of all mechanism elements.
  * Please, refer to the Kinematic.png file for the
  * graphic representation.
  */
#define OD 16
#define OE 17
#define DF 18
#define EG 19
/** Specifies the transformation values of all axises.
  */
#define Kx 10  // =360/screw_step
#define Ky 10  // =360/screw_step
#define Kz 10  // =360/screw_step
/** Specifies the angle's values (in grads) of the elements
  * DF and GE in the initial position.
  */
#define Ix 120
#define Iy 120

double curr_X, curr_Y, curr_Z;

static double GetXAngle(double X, double Y, double Z)
{
  return X/Kx-Ix+acos((EG*EG-OE*OE+Y*Y+Z*Z)/(2*EG*sqrt(Y*Y+Z*Z)))+atan(Z/X);
}

static double GetYAngle(double X, double Y, double Z)
{
  return Y/Ky-Iy+acos((DF*DF-OD*OD+X*X+Z*Z)/(2*DF*sqrt(X*X+Y*Y)))+atan(Z/Y);
}

static double GetZAngle(double Z)
{
  return Z/Kz;
}

/* Public functions */

Error InitAllMotors(void)
{
  return SMotorDriversInit();
}

Error DeInitAllMotors(void)
{
  return SMotorDriversDeInit();
}

Error EnableAllMotors(void)
{
  Error err = _Success;
  err = EnableMotor(_X);
  if (err != _Success) goto e;
  err = EnableMotor(_Y);
  if (err != _Success) goto e;
  err = EnableMotor(_Z);
  if (err != _Success) goto e;
  return _Success;
  e:
  return err;
}

Error DisabeAllMotors(void)
{
  Error err = _Success;
  err = DisableMotor(_X);
  if (err != _Success) goto e;
  err = DisableMotor(_Y);
  if (err != _Success) goto e;
  err = DisableMotor(_Z);
  if (err != _Success) goto e;
  return _Success;
  e:
  return err;
}

Error ZeroOutPosition(void)
{
  Error err = _Success;
  err = ZeroOutAngleCounter(_X);
  if (err != _Success) goto e;
  err = ZeroOutAngleCounter(_Y);
  if (err != _Success) goto e;
  err = ZeroOutAngleCounter(_Z);
  if (err != _Success) goto e;
  curr_X = 0;
  curr_Y = 0;
  curr_Z = 0;
  return _Success;
  e:
  return err;
}

Error GoToWithSpeed(double X, double Y, double Z, double speed)
{
  double l = sqrt((X-curr_X)*(X-curr_X)+(Y-curr_Y)*(Y-curr_Y)+(Z-curr_Z)*(Z-curr_Z));
  double cos_a = (X-curr_X)/l;
  double cos_b = (Y-curr_Y)/l;
  double cos_c = (Z-curr_Z)/l;

  Error err = _Success;
  err = MotorSetSpeedAndValue(_X, speed*cos_a/Kx, GetXAngle(X,Y,Z));
  if (err != _Success) goto e;
  err = MotorSetSpeedAndValue(_Y, speed*cos_b/Ky, GetYAngle(X,Y,Z));
  if (err != _Success) goto e;
  err = MotorSetSpeedAndValue(_Z, speed*cos_c/Kz, GetZAngle(Z));
  if (err != _Success) goto e;

  while(IsMotorBusy(_X));
  while(IsMotorBusy(_Y));
  while(IsMotorBusy(_Z));

  return _Success;
  e:
  return err;
}
