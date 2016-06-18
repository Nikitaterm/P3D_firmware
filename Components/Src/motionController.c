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

/** Specifies the length (in mm) of all mechanism elements.
  * Please, refer to the Kinematic.png file for the
  * graphic representation.
  */
#define OD 241.0996
#define OE 241.0996
#define DF 284.2327
#define EG 284.2327
#define OOD 82
#define OOE 82
/** Specifies the transformation values of all axises.
  */
#define Kx 72  // =360/screw_step (in mm)
#define Ky 72  // =360/screw_step (in mm)
#define Kz 120  // =360/screw_step (in mm)
/** Specifies the angle's values (in grads) of the elements
  * DF and GE in the initial position.
  */
#define Ix 90 - 2.3188
#define Iy 90 - 2.3188

double curr_X, curr_Y, curr_Z;

static double GetXAngle(double X, double Y, double Z)
{
  return X*Kx-Ix+acos((EG*EG-OE*OE+(Y+OOE)*(Y+OOE)+Z*Z)/(2*EG*sqrt((Y+OOE)*(Y+OOE)+Z*Z)))+atan(Z/Y);
}

static double GetYAngle(double X, double Y, double Z)
{
  return Y*Ky-Iy+acos((DF*DF-OD*OD+(X+OOD)*(X+OOD)+Z*Z)/(2*DF*sqrt((X+OOD)*(X+OOD)+Z*Z)))+atan(Z/X);
}

static double GetZAngle(double Z)
{
  return Z*Kz;
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

double XX, YY, ZZ;
 Error errR;
Error GoToWithSpeed(double X, double Y, double Z, double speed)
{
  double l = sqrt((X-curr_X)*(X-curr_X)+(Y-curr_Y)*(Y-curr_Y)+(Z-curr_Z)*(Z-curr_Z));
  double cos_a = (X-curr_X)/l;
  double cos_b = (Y-curr_Y)/l;
  double cos_c = (Z-curr_Z)/l;

  XX = GetXAngle(X,Y,Z);
  YY = GetYAngle(X,Y,Z);
  ZZ = GetZAngle(Z);

  errR = _Success;
  errR = MotorSetSpeedAndValue(_X, speed*cos_a/Kx, GetXAngle(X,Y,Z));
  if (errR != _Success) goto e;
  errR = MotorSetSpeedAndValue(_Y, speed*cos_b/Ky, GetYAngle(X,Y,Z));
  if (errR != _Success) goto e;
  errR = MotorSetSpeedAndValue(_Z, speed*cos_c/Kz, GetZAngle(Z));
  if (errR != _Success) goto e;

  while(IsMotorBusy(_X));
  while(IsMotorBusy(_Y));
  while(IsMotorBusy(_Z));

  return _Success;
  e:
  return errR;
}
