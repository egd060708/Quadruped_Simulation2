#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Compass.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>
#include "Leg.h"
#include "LegCtrl.h"
#include "Body.h"
#include "mathPrint.h"
#include "GaitCtrl.h"
#include "water_gait1.h"

#define MOTOR_TORQUE 0
#define BACK_INV 1 // 是否使用后腿反屈膝
#define STAND_BIAS 0.023 // 中性落足点的偏移，相对于四个髋关节的位置，正为向外，负为向内
#define HEIGHT 0.15
#define IS_WATER_GAIT 1

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // get keyboard
  Keyboard* keyboard = robot->getKeyboard();
  keyboard->enable(timeStep);

  // get imu
  InertialUnit* imu = robot->getInertialUnit("imu");
  imu->enable(timeStep);
  Gyro* gyro = robot->getGyro("gyro");
  gyro->enable(timeStep);
  Accelerometer* acc = robot->getAccelerometer("acc");
  acc->enable(timeStep);

  // get motor
  Motor* motors[4][2];

  motors[LF][0] = robot->getMotor("lf_hip_motor");
  motors[LF][1] = robot->getMotor("lf_knee_motor");

  for (int i = 0; i < 2; i++)
  {
      motors[LF][i]->setControlPID(20, 1, 0.1);
#if MOTOR_TORQUE == 1
      motors[LF][i]->setPosition(INFINITY);
      motors[LF][i]->setVelocity(0);
#endif
  }


  motors[RF][0] = robot->getMotor("rf_hip_motor");
  motors[RF][1] = robot->getMotor("rf_knee_motor");

  for (int i = 0; i < 2; i++)
  {
      motors[RF][i]->setControlPID(20, 1, 0.1);
#if MOTOR_TORQUE == 1
      motors[RF][i]->setPosition(INFINITY);
      motors[RF][i]->setVelocity(0);
#endif
  }

  motors[LB][0] = robot->getMotor("lb_hip_motor");
  motors[LB][1] = robot->getMotor("lb_knee_motor");

  for (int i = 0; i < 2; i++)
  {
      motors[LB][i]->setControlPID(20, 1, 0.1);
#if MOTOR_TORQUE == 1
      motors[LB][i]->setPosition(INFINITY);
      motors[LB][i]->setVelocity(0);
#endif
  }


  motors[RB][0] = robot->getMotor("rb_hip_motor");
  motors[RB][1] = robot->getMotor("rb_knee_motor");

  for (int i = 0; i < 2; i++)
  {
      motors[RB][i]->setControlPID(20, 1, 0.1);
#if MOTOR_TORQUE == 1
      motors[RB][i]->setPosition(INFINITY);
      motors[RB][i]->setVelocity(0);
#endif
  }

  /* 自定义类型 */
  LegS legsObj[4];
  Leg_Ctrl_Param legsCtrlParam[4];
  Body body;
  double links[2] = { 0.09, 0.1225};
  InitLeg(&legsObj[LF], links, 1, 1);
  InitLeg(&legsObj[RF], links, -1, 1);
#if BACK_INV==1
  InitLeg(&legsObj[LB], links, 1, -1);
  InitLeg(&legsObj[RB], links, -1, -1);
#else
  InitLeg(&legsObj[LB], links, 1, 1);
  InitLeg(&legsObj[RB], links, -1, 1);
#endif
  double kp_p[3] = { 10,10,-5 };
  //double kd_p[3] = { 0.5,0.5,-0.25 };
  double kd_p[3] = { 0.,0.,-0. };
  double kp_t[3] = { 1,1,1 };
  double init_p[4][3] = {
      {STAND_BIAS, 0, -HEIGHT},
      {STAND_BIAS, 0, -HEIGHT},
      {-STAND_BIAS, 0, -HEIGHT},
      {-STAND_BIAS, 0, -HEIGHT}
  };
  for (int i = 0; i < 4; i++)
  {
      InitLegCtrl(&legsCtrlParam[i], kp_p, kd_p, kp_t, (double)timeStep * 0.001);
      StartVirtualVMC(init_p[i], &legsObj[i]);
  }
  LegS* lobj[4] = { &legsObj[LF],&legsObj[RF],&legsObj[LB],&legsObj[RB] };
  InitBody(&body, lobj, (double)timeStep * 0.001);
  double leg2Body[3] = { 0.12, 0.0826, 0 };
  SetBodyParam(&body, leg2Body);
  double init_Ep[4][3] = {
      {0.12 + STAND_BIAS, 0.0826, 0.},
      {0.12 + STAND_BIAS, -0.0826, 0.},
      {-0.12 - STAND_BIAS, 0.0826, 0.},
      {-0.12 - STAND_BIAS, -0.0826, 0.},
  };
  UpdateFootPoint(&body.targetWorldState, init_Ep);
  UpdateFootPointBase(&body.targetWorldState, init_Ep);
  double angle_t[3] = { 0,0,0 };
  double p_t[3] = { 0,0,HEIGHT };
  UpdateBodyPos(&body.targetWorldState, angle_t, p_t);
  GaitS gaitData;
  double bias[4] = { 0.5, 0, 0, 0.5 };
  InitGaitCtrl(&gaitData, &body, 0.6, 0.5, bias);
  double cmd_slope[3] = { 0.01,0.01,0.01 };
  bool is_gait = false;

  // 水下步态参数
  double water_Ts = 4.;
  double water_start_t = 0;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
      // 步态速度指令
      double gait_cmd[4] = { 0,0,0,0.03 };
      // 获取系统时间
      double t = robot->getTime();
      // 键盘控制
      int key = keyboard->getKey();
      while (key > 0)
      {
          switch (key)
          {
          case keyboard->UP:
              //p_t[0] += 0.0002;
              gait_cmd[0] = 0.2;
              break;
          case keyboard->DOWN:
              //p_t[0] -= 0.0002;
              gait_cmd[0] = -0.2;
              break;
          case keyboard->RIGHT:
              //p_t[1] -= 0.0002;
              gait_cmd[1] = -0.3;
              break;
          case keyboard->LEFT:
              //p_t[1] += 0.0002;
              gait_cmd[1] = 0.3;
              break;
          case (keyboard->SHIFT + keyboard->RIGHT):
              angle_t[0] += 0.0005;
              break;
          case (keyboard->SHIFT + keyboard->LEFT):
              angle_t[0] -= 0.0005;
              break;
          case (keyboard->SHIFT + keyboard->UP):
              p_t[2] += 0.0001;
              break;
          case (keyboard->SHIFT + keyboard->DOWN):
              p_t[2] -= 0.0001;
              break;
          case 'O':
              break;
          case 'W':
              angle_t[1] += 0.0005;
              break;
          case 'S':
              angle_t[1] -= 0.0005;
              break;
          case 'A':
              //angle_t[2] += 0.0005;
              gait_cmd[2] = 1.5;
              break;
          case 'D':
              //angle_t[2] -= 0.0005;
              gait_cmd[2] = -1.5;
              break;
          case 'U':
              is_gait = true;
              GaitRestart(&gaitData, t);
              water_start_t = t;
              break;
          case 'I':
              is_gait = false;
              water_start_t = 0;
              break;
          }
          key = keyboard->getKey();
      }
      if (IS_WATER_GAIT)
      {
          double get_gait[4][2];
          getGait(get_gait, water_Ts, t, water_start_t);
          for (int i = 0; i < 4; i++)
          {
              for (int j = 0; j < 2; j++)
              {
                  double out_angle = 0;
                  if (i == 1 || i == 3)
                  {
                      out_angle = -(get_gait[i][j]);
                      if (j == 0)
                      {
                          out_angle -= 3.1415926 / 2.;
                      }
                  }
                  else
                  {
                      out_angle = get_gait[i][j];
                      if (j == 0)
                      {
                          out_angle += 3.1415926 / 2.;
                      }
                  }
                  
                  motors[i][j]->setPosition(out_angle);
              }
          }
      }
      else
      {
          // 更新机身姿态目标
          UpdateBodyPos(&body.targetWorldState, angle_t, p_t);
          UpdateRsb(body.Rsb_t, body.targetWorldState.Ang_xyz);
          UpdateRsbI_R(body.RsbI_t, body.Rsb_t);
          UpdatePsb(body.Psb_t, body.targetWorldState.dist);
          // 更新伪反馈
          EndS* leg4c[4] = { &body.legs[LF]->currentEnd,
                            &body.legs[RF]->currentEnd,
                            &body.legs[LB]->currentEnd,
                            &body.legs[RB]->currentEnd };
          EndS* bleg4c[4] = { &body.currentBodyState.leg_b[LF],
                             &body.currentBodyState.leg_b[RF],
                             &body.currentBodyState.leg_b[LB],
                             &body.currentBodyState.leg_b[RB] };
          Leg2BodyAll(bleg4c, leg4c, body.leg2body);
          Body2WorldP(&body.currentWorldState, &body.currentBodyState, body.Rsb_t, body.Psb_t);
          // 步态生成
          SetGaitCmd(&gaitData, gait_cmd, cmd_slope);
          PhaseGen(&gaitData, t);
          GaitGen(&gaitData);
          /*printV("R", gaitData.R, 4);
          printV("t", gaitData.theta, 4);*/
          if (is_gait == true)
          {
              UpdateFootPoint(&body.targetWorldState, gaitData.End_Pos);
              UpdateFootVel(&body.targetWorldState, gaitData.End_Vel);
          }
          else
          {
              UpdateFootPoint(&body.targetWorldState, init_Ep);
              double end_v[4][3] = { 0 };
              UpdateFootVel(&body.targetWorldState, end_v);
          }
          // 输出转换坐标系
          //printV("WorldP", body.targetWorldState.leg_s[0].Position, 3);
          World2BodyP(&body.targetBodyState, &body.targetWorldState, body.RsbI_t, body.Psb_t);
          //printV("BodyP", body.targetBodyState.leg_b[0].Position, 3);
          EndS* leg4t[4] = { &body.legs[LF]->targetEnd,
                            &body.legs[RF]->targetEnd,
                            &body.legs[LB]->targetEnd,
                            &body.legs[RB]->targetEnd };
          EndS* bleg4t[4] = { &body.targetBodyState.leg_b[LF],
                             &body.targetBodyState.leg_b[RF],
                             &body.targetBodyState.leg_b[LB],
                             &body.targetBodyState.leg_b[RB] };
          Body2LegAll(leg4t, bleg4t, body.leg2body);
          /*printV("LegP", legsObj[0].targetEnd.Position, 3);*/
          //LegVirtualVMC(&legsObj[0], &legsCtrlParam[0]);
          // 接入控制器
          /*std::cout << legsObj[0].targetJoint.Angle[0] << std::endl;
          std::cout << legsObj[0].targetJoint.Angle[1] << std::endl;*/
          for (int i = 0; i < 4; i++)
          {
              LegVirtualVMC(&legsObj[i], &legsCtrlParam[i]);
              for (int j = 0; j < 2; j++)
              {
                  if (i == 1 || i == 3)
                  {
                      motors[i][j]->setPosition(-legsObj[i].targetJoint.Angle[j]);
                  }
                  else
                  {
                      motors[i][j]->setPosition(legsObj[i].targetJoint.Angle[j]);
                  }
              }
          }
      }
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
