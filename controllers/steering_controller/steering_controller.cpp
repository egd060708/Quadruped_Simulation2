// File:          steering_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <iostream>
#include <math.h>

using namespace webots;

int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // get devices
  Accelerometer *acc = robot->getAccelerometer("ACC");
  Gyro *gyro = robot->getGyro("GYRO");
  InertialUnit *imu = robot->getInertialUnit("IMU");
  Keyboard *keyboard = robot->getKeyboard();
  Motor *steerMotor[4];
  Motor *wheelMotor[4];
  Motor *gimbalMotor[2];
  steerMotor[0] = robot->getMotor("LF_STEER_MOTOR");
  steerMotor[1] = robot->getMotor("RF_STEER_MOTOR");
  steerMotor[2] = robot->getMotor("LB_STEER_MOTOR");
  steerMotor[3] = robot->getMotor("RB_STEER_MOTOR");
  wheelMotor[0] = robot->getMotor("LF_WHEEL_MOTOR");
  wheelMotor[1] = robot->getMotor("RF_WHEEL_MOTOR");
  wheelMotor[2] = robot->getMotor("LB_WHEEL_MOTOR");
  wheelMotor[3] = robot->getMotor("RB_WHEEL_MOTOR");
  gimbalMotor[0] = robot->getMotor("GIMBAL_YAW_MOTOR");
  gimbalMotor[1] = robot->getMotor("GIMBAL_PITCH_MOTOR");
  PositionSensor *steerEncoder[4];
  PositionSensor *wheelEncoder[4];
  PositionSensor *gimbalEncoder[4];
  steerEncoder[0] = robot->getPositionSensor("LF_STEER_ENCODER");
  steerEncoder[1] = robot->getPositionSensor("RF_STEER_ENCODER");
  steerEncoder[2] = robot->getPositionSensor("LB_STEER_ENCODER");
  steerEncoder[3] = robot->getPositionSensor("RB_STEER_ENCODER");
  wheelEncoder[0] = robot->getPositionSensor("LF_WHEEL_ENCODER");
  wheelEncoder[1] = robot->getPositionSensor("RF_WHEEL_ENCODER");
  wheelEncoder[2] = robot->getPositionSensor("LB_WHEEL_ENCODER");
  wheelEncoder[3] = robot->getPositionSensor("RB_WHEEL_ENCODER");
  gimbalEncoder[0] = robot->getPositionSensor("GIMBAL_YAW_ENCODER");
  gimbalEncoder[1] = robot->getPositionSensor("GIMBAL_PITCH_ENCODER");

  // enable devices
  acc->enable(timeStep);
  gyro->enable(timeStep);
  imu->enable(timeStep);
  keyboard->enable(timeStep);
  for (int i(0); i < 4; i++)
  {
    steerMotor[i]->setPosition(0);
    wheelMotor[i]->setPosition(INFINITY);
    // wheelMotor[i]->setControlPID(10, 10, 0);
    steerEncoder[i]->enable(timeStep);
    wheelEncoder[i]->enable(timeStep);
  }
  for (int i(0); i < 2; i++)
  {
    gimbalMotor[i]->setPosition(0);
    gimbalEncoder[i]->enable(timeStep);
  }

  double Vx_t = 0, Vy_t = 0;
  double yaw_t = 0, pitch_t = 0;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1)
  {
    Vx_t = 0;
    Vy_t = 0;
    int key = keyboard->getKey();
    while (key > 0)
    {
      switch (key)
      {
      case keyboard->UP:
        Vx_t = 20;
        break;
      case keyboard->DOWN:
        Vx_t = -20;
        break;
      case keyboard->RIGHT:
        Vy_t = -20;
        break;
      case keyboard->LEFT:
        Vy_t = 20;
        break;
      case 'W':
        pitch_t -= 0.01;
        break;
      case 'S':
        pitch_t += 0.01;
        break;
      case 'A':
        yaw_t += 0.01;
        break;
      case 'D':
        yaw_t -= 0.01;
        break;
      default:
        break;
      }
      key = keyboard->getKey();
    }

    double directionAngle = 0, speed = 0;
    if (Vx_t == 0 && Vy_t == 0)
    {
      directionAngle = 0;
    }
    else if (Vx_t == 0 && Vy_t != 0)
    {
      if (Vy_t > 0)
      {
        directionAngle = M_PI_2;
      }
      else
      {
        directionAngle = -M_PI_2;
      }
    }
    else if (Vx_t != 0 && Vy_t == 0)
    {
      if (Vx_t > 0)
      {
        directionAngle = 0;
      }
      else
      {
        directionAngle = M_PI;
      }
    }
    else
    {
      directionAngle = std::atan2(Vy_t,Vx_t);
    }
    speed = std::sqrt(Vx_t * Vx_t + Vy_t * Vy_t);
    for (int i(0); i < 4; i++)
    {
      steerMotor[i]->setPosition(-directionAngle);
      if (i == 0 || i == 2)
      {
        wheelMotor[i]->setVelocity(-speed);
      }
      else
      {
        wheelMotor[i]->setVelocity(speed);
      }
    }
    gimbalMotor[0]->setPosition(yaw_t);
    gimbalMotor[1]->setPosition(pitch_t);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
