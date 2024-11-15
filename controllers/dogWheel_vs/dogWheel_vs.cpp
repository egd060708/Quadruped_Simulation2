#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Compass.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>
#include <webots/Display.hpp>
#include "LegCtrl.h"
#include "RobotParams.hpp"
#include "Body.h"
#include "kelmanFilter.h"
#include "BodyCtrl.h"
#include "vofaTransmit.h"
#include "GaitCtrl.h"
#include "SecondButterworthLPF.h"
#include "virtualLegCtrl.h"
#include "dataDisplay.h"

using namespace webots;
using namespace Quadruped;
using namespace std;

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // get keyboard
  Keyboard* keyboard = robot->getKeyboard();
  keyboard->enable(timeStep);

  // get imu
  InertialUnit* imu = robot->getInertialUnit("trunk_imu inertial");
  imu->enable(timeStep);
  Gyro* gyro = robot->getGyro("trunk_imu gyro");
  gyro->enable(timeStep);
  Accelerometer* acc = robot->getAccelerometer("trunk_imu accelerometer");
  acc->enable(timeStep);

  // get motor
  Motor* motors[4][3];

  motors[Quadruped::LF][0] = robot->getMotor("FL_hip_joint");
  motors[Quadruped::LF][1] = robot->getMotor("FL_thigh_joint");
  motors[Quadruped::LF][2] = robot->getMotor("FL_calf_joint");
  for (int i = 0; i < 3; i++)
  {
      motors[Quadruped::LF][i]->setPosition(INFINITY);
      motors[Quadruped::LF][i]->setVelocity(0);
  }

  motors[Quadruped::RF][0] = robot->getMotor("FR_hip_joint");
  motors[Quadruped::RF][1] = robot->getMotor("FR_thigh_joint");
  motors[Quadruped::RF][2] = robot->getMotor("FR_calf_joint");
  for (int i = 0; i < 3; i++)
  {
      motors[Quadruped::RF][i]->setPosition(INFINITY);
      motors[Quadruped::RF][i]->setVelocity(0);
  }

  motors[Quadruped::LB][0] = robot->getMotor("RL_hip_joint");
  motors[Quadruped::LB][1] = robot->getMotor("RL_thigh_joint");
  motors[Quadruped::LB][2] = robot->getMotor("RL_calf_joint");
  for (int i = 0; i < 3; i++)
  {
      motors[Quadruped::LB][i]->setPosition(INFINITY);
      motors[Quadruped::LB][i]->setVelocity(0);
  }

  motors[Quadruped::RB][0] = robot->getMotor("RR_hip_joint");
  motors[Quadruped::RB][1] = robot->getMotor("RR_thigh_joint");
  motors[Quadruped::RB][2] = robot->getMotor("RR_calf_joint");
  for (int i = 0; i < 3; i++)
  {
      motors[Quadruped::RB][i]->setPosition(INFINITY);
      motors[Quadruped::RB][i]->setVelocity(0);
  }

  // get position sensor
  PositionSensor* encoder[4][3];

  encoder[Quadruped::LF][0] = robot->getPositionSensor("FL_hip_joint_sensor");
  encoder[Quadruped::LF][1] = robot->getPositionSensor("FL_thigh_joint_sensor");
  encoder[Quadruped::LF][2] = robot->getPositionSensor("FL_calf_joint_sensor");
  for (int i = 0; i < 3; i++)
  {
      encoder[Quadruped::LF][i]->enable(timeStep);
  }

  encoder[Quadruped::RF][0] = robot->getPositionSensor("FR_hip_joint_sensor");
  encoder[Quadruped::RF][1] = robot->getPositionSensor("FR_thigh_joint_sensor");
  encoder[Quadruped::RF][2] = robot->getPositionSensor("FR_calf_joint_sensor");
  for (int i = 0; i < 3; i++)
  {
      encoder[Quadruped::RF][i]->enable(timeStep);
  }

  encoder[Quadruped::LB][0] = robot->getPositionSensor("RL_hip_joint_sensor");
  encoder[Quadruped::LB][1] = robot->getPositionSensor("RL_thigh_joint_sensor");
  encoder[Quadruped::LB][2] = robot->getPositionSensor("RL_calf_joint_sensor");
  for (int i = 0; i < 3; i++)
  {
      encoder[Quadruped::LB][i]->enable(timeStep);
  }

  encoder[Quadruped::RB][0] = robot->getPositionSensor("RR_hip_joint_sensor");
  encoder[Quadruped::RB][1] = robot->getPositionSensor("RR_thigh_joint_sensor");
  encoder[Quadruped::RB][2] = robot->getPositionSensor("RR_calf_joint_sensor");
  for (int i = 0; i < 3; i++)
  {
      encoder[Quadruped::RB][i]->enable(timeStep);
  }

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
