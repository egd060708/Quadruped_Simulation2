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
//#include "BodyCtrl.h"
#include "vofaTransmit.h"
#include "GaitCtrl.h"
//#include "SecondButterworthLPF.h"
#include "virtualLegCtrl.h"
#include "dataDisplay.h"
#include "RobotEst.h"
#include "BodyCtrlNew.h"
#include "mathTool.h"
#include "SlopeEst.h"
#include "TraPlan.h"



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
  Motor* motors[4][4];

  motors[Quadruped::LF][0] = robot->getMotor("FL_hip_joint");
  motors[Quadruped::LF][1] = robot->getMotor("FL_thigh_joint");
  motors[Quadruped::LF][2] = robot->getMotor("FL_calf_joint");
  motors[Quadruped::LF][3] = robot->getMotor("FL_foot_joint");
  for (int i = 0; i < 4; i++)
  {
      motors[Quadruped::LF][i]->setPosition(INFINITY);
      motors[Quadruped::LF][i]->setVelocity(0);
  }

  motors[Quadruped::RF][0] = robot->getMotor("FR_hip_joint");
  motors[Quadruped::RF][1] = robot->getMotor("FR_thigh_joint");
  motors[Quadruped::RF][2] = robot->getMotor("FR_calf_joint");
  motors[Quadruped::RF][3] = robot->getMotor("FR_foot_joint");
  for (int i = 0; i < 4; i++)
  {
      motors[Quadruped::RF][i]->setPosition(INFINITY);
      motors[Quadruped::RF][i]->setVelocity(0);
  }

  motors[Quadruped::LB][0] = robot->getMotor("RL_hip_joint");
  motors[Quadruped::LB][1] = robot->getMotor("RL_thigh_joint");
  motors[Quadruped::LB][2] = robot->getMotor("RL_calf_joint");
  motors[Quadruped::LB][3] = robot->getMotor("RL_foot_joint");
  for (int i = 0; i < 4; i++)
  {
      motors[Quadruped::LB][i]->setPosition(INFINITY);
      motors[Quadruped::LB][i]->setVelocity(0);
  }

  motors[Quadruped::RB][0] = robot->getMotor("RR_hip_joint");
  motors[Quadruped::RB][1] = robot->getMotor("RR_thigh_joint");
  motors[Quadruped::RB][2] = robot->getMotor("RR_calf_joint");
  motors[Quadruped::RB][3] = robot->getMotor("RR_foot_joint");
  for (int i = 0; i < 4; i++)
  {
      motors[Quadruped::RB][i]->setPosition(INFINITY);
      motors[Quadruped::RB][i]->setVelocity(0);
  }

  // get position sensor
  PositionSensor* encoder[4][4];

  encoder[Quadruped::LF][0] = robot->getPositionSensor("FL_hip_joint_sensor");
  encoder[Quadruped::LF][1] = robot->getPositionSensor("FL_thigh_joint_sensor");
  encoder[Quadruped::LF][2] = robot->getPositionSensor("FL_calf_joint_sensor");
  encoder[Quadruped::LF][3] = robot->getPositionSensor("FL_foot_joint_sensor");
  for (int i = 0; i < 4; i++)
  {
      encoder[Quadruped::LF][i]->enable(timeStep);
  }

  encoder[Quadruped::RF][0] = robot->getPositionSensor("FR_hip_joint_sensor");
  encoder[Quadruped::RF][1] = robot->getPositionSensor("FR_thigh_joint_sensor");
  encoder[Quadruped::RF][2] = robot->getPositionSensor("FR_calf_joint_sensor");
  encoder[Quadruped::RF][3] = robot->getPositionSensor("FR_foot_joint_sensor");
  for (int i = 0; i < 4; i++)
  {
      encoder[Quadruped::RF][i]->enable(timeStep);
  }

  encoder[Quadruped::LB][0] = robot->getPositionSensor("RL_hip_joint_sensor");
  encoder[Quadruped::LB][1] = robot->getPositionSensor("RL_thigh_joint_sensor");
  encoder[Quadruped::LB][2] = robot->getPositionSensor("RL_calf_joint_sensor");
  encoder[Quadruped::LB][3] = robot->getPositionSensor("RL_foot_joint_sensor");
  for (int i = 0; i < 4; i++)
  {
      encoder[Quadruped::LB][i]->enable(timeStep);
  }

  encoder[Quadruped::RB][0] = robot->getPositionSensor("RR_hip_joint_sensor");
  encoder[Quadruped::RB][1] = robot->getPositionSensor("RR_thigh_joint_sensor");
  encoder[Quadruped::RB][2] = robot->getPositionSensor("RR_calf_joint_sensor");
  encoder[Quadruped::RB][3] = robot->getPositionSensor("RR_foot_joint_sensor");
  for (int i = 0; i < 4; i++)
  {
      encoder[Quadruped::RB][i]->enable(timeStep);
  }

  // self controll classes
  double links[7] = { Quadruped::L1, Quadruped::L2, Quadruped::L3, Quadruped::L4, Quadruped::L5, Quadruped::L6, Quadruped::L7 };
  Vector<double, 6> iner[4] = { Quadruped::Ihip, Quadruped::Ithigh, Quadruped::Icalf, Quadruped::Ifoot };
  double mass[4] = { Quadruped::Mhip, Quadruped::Mthigh, Quadruped::Mcalf, Quadruped::Mfoot };
  Vector3d p_mass[4] = { Quadruped::Phip, Quadruped::Pthigh, Quadruped::Pcalf, Quadruped::Pfoot };
  Leg lf_leg_obj(links, p_mass, mass, iner, Quadruped::LF);
  LegCtrl lf_leg_ctrl(&lf_leg_obj, timeStep);
  lf_leg_ctrl.setEndPositionTar(Eigen::Vector3d(0.0130447 + XMOVE + XSPANF, 0.234832 + YSPAN, -0.5937));

  Leg rf_leg_obj(links, p_mass, mass, iner, Quadruped::RF);
  LegCtrl rf_leg_ctrl(&rf_leg_obj, timeStep);
  rf_leg_ctrl.setEndPositionTar(Eigen::Vector3d(0.0130447 + XMOVE + XSPANF, -0.234832 - YSPAN, -0.5937));

  Leg lb_leg_obj(links, p_mass, mass, iner, Quadruped::LB);
  LegCtrl lb_leg_ctrl(&lb_leg_obj, timeStep);
  lb_leg_ctrl.setEndPositionTar(Eigen::Vector3d(0.0130447 + XMOVE - XSPANB, 0.234832 + YSPAN, -0.5937));

  Leg rb_leg_obj(links, p_mass, mass, iner, Quadruped::RB);
  LegCtrl rb_leg_ctrl(&rb_leg_obj, timeStep);
  rb_leg_ctrl.setEndPositionTar(Eigen::Vector3d(0.0130447 + XMOVE - XSPANB, -0.234832 - YSPAN, -0.5937));

//#if USE_WHEEL == 1
  QpwEst qpest(static_cast<double>(timeStep) * 0.001f);
//#else
//  QpEst qpest(static_cast<double>(timeStep) * 0.001f);
//#endif
  Leg* legsObj[4] = { &lf_leg_obj, &rf_leg_obj, &lb_leg_obj, &rb_leg_obj };
  Body qp_body(&qpest,legsObj, static_cast<double>(timeStep) * 0.001f);
  double mb[3] = { Quadruped::Mmid, Quadruped::Mhead, Quadruped::Mtail };
  Vector<double, 6> ib[3] = { Quadruped::Imid, Quadruped::Ihead, Quadruped::Itail };
  Vector3d pb[3] = { Quadruped::Pmid, Quadruped::Phead, Quadruped::Ptail };
  Eigen::Matrix<double, 3, 4> footPoint;
  footPoint.col(LF) = Vector3d(0.3415447 + XMOVE + XSPANF, 0.234832 + YSPAN, 0);
  footPoint.col(RF) = Vector3d(0.3415447 + XMOVE + XSPANF, -0.234832 - YSPAN, 0);
  footPoint.col(LB) = Vector3d(-0.3154553 + XMOVE - XSPANB, 0.234832 + YSPAN, 0);
  footPoint.col(RB) = Vector3d(-0.3154553 + XMOVE - XSPANB, -0.234832 - YSPAN, 0);
  qp_body.initParams(leg2bodyFrame, footPoint, mb, ib, pb);
  qp_body.updateTargetFootPoint(footPoint);
  Vector3d angle_t(0, 0, 0);
  Vector3d p_t(0, 0, 0.5937);
  double x_t = 0;
  double y_t = 0;
  double z_t = 0.5937;
  double roll_t = 0;
  double pitch_t = 0;
  double yaw_t = 0;
  double test = 0;

  LegCtrl* legsCtrl[4] = { &lf_leg_ctrl,&rf_leg_ctrl,&lb_leg_ctrl,&rb_leg_ctrl };

  //BodyCtrl qp_ctrl(&qp_body, legsCtrl, timeStep);
#if USE_WHEEL == 1
  QpwPVCtrl qp_ctrl(&qp_body, legsCtrl, timeStep);
#else
  QpPVCtrl qp_ctrl(&qp_body, legsCtrl, timeStep);
#endif
  qp_ctrl.importWeight(Q, F, R, W);
//#if USE_WHEEL == 1
//  qp_ctrl.importPDparam(linPD, angPD,wheelPID);
//#else
//  qp_ctrl.importPDparam(linPD, angPD);
//#endif
  bool use_mpc = false;

  Vector4d phaseResult;
  Vector4i contactResult;
  phaseResult.setZero();
  contactResult.setOnes();
  GaitCtrl gaitCtrl(&qp_ctrl, legsCtrl, timeStep, &phaseResult, &contactResult);
  gaitCtrl.initSwingParams(0.6, 1.0, Eigen::Vector4d(0.5, 0, 0, 0.5), robot->getTime());
  gaitCtrl.initExpectK(gaitK);
  Eigen::Matrix<double, 3, 4> feetPos;
  Eigen::Matrix<double, 3, 4> feetVel;

  SlopeEst slope;
  slope.init(footPoint, Eigen::Vector4i::Ones());

  BalanceDividePlanning bdp;
  bdp.setInitBodyPlanPosition(Eigen::Vector3d(x_t, y_t, z_t));
  bdp.setInitFootPlanPosition(footPoint);

  BalanceJointPlanning bjp;
  bjp.setInitBodyPlanPosition(Eigen::Vector3d(x_t, y_t, z_t));
  bjp.setInitFootPlanPosition(footPoint);

  //VOFA vofa("vjs.exe");

  Eigen::Vector4d last_encoderValue[4];
  for (auto p : last_encoderValue)
  {
      p.setZero();
  }

  uint8_t initCount = 0;
  bool is_sys_init = false;
  Vector3d imuRPYd;
  Vector4d imuQd;
  Vector3d gyrod;
  Vector3d accd;
  Eigen::Matrix<double, 3, 4> legF = Eigen::Matrix<double,3,4>::Zero();

  // 重新配置腿部曲线跟踪参数
  Vector2d lPid_pvParams[6];
  lPid_pvParams[0] << 8000, 350;
  lPid_pvParams[1] << 150, 175;
  lPid_pvParams[2] << 8000, 350;
  lPid_pvParams[3] << 150, 175;
  lPid_pvParams[4] << 8000, 500;
  lPid_pvParams[5] << 150, 250;
  for (auto p : legsCtrl)
  {
      p->loadPid_pvParams(p->lPid_pv, lPid_pvParams);
  }

  //LPF_SecondOrder_Classdef velFilterN[3] = { LPF_SecondOrder_Classdef(5,500),LPF_SecondOrder_Classdef(5,500) ,LPF_SecondOrder_Classdef(5,500) };
  //MeanFilter<100> velFilterN[3];
  MeanFilter<100> velFilter[4];
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
      // 使用gps计算加速度
      double t = robot->getTime();

      double vx_t = 0;
      double vy_t = 0;
      double vz_t = 0;
      double vyaw_t = 0;
      double vtest = 0;

      if (is_sys_init == false)
      {
          initCount++;
          if (initCount == 5)
          {
              is_sys_init = true;
          }
      }
      else
      {
          // 键盘控制
          int key = keyboard->getKey();
          while (key > 0)
          {
              switch (key)
              {
              case keyboard->UP:
                  //x_t += 0.0002;
                  vx_t = 2.2;
                  break;
              case keyboard->DOWN:
                  //x_t -= 0.0002;
                  vx_t = -1.7;
                  break;
              case keyboard->RIGHT:
                  //y_t -= 0.0002;
                  vy_t = -0.5;
                  break;
              case keyboard->LEFT:
                  //y_t += 0.0002;
                  vy_t = 0.5;
                  break;
              case (keyboard->SHIFT + keyboard->RIGHT):
                  roll_t += 0.0005;
                  break;
              case (keyboard->SHIFT + keyboard->LEFT):
                  roll_t -= 0.0005;
                  break;
              case (keyboard->SHIFT + keyboard->UP):
                  vz_t = 0.3;
                  //z_t += 0.0002;
                  break;
              case (keyboard->SHIFT + keyboard->DOWN):
                  vz_t = -0.3;
                  //z_t -= 0.0002;
                  break;
              case 'O':
                  break;
              case 'W':
                  pitch_t += 0.0005;
                  break;
              case 'S':
                  pitch_t -= 0.0005;
                  break;
              case 'A':
                  //yaw_t += 0.0005;
                  vyaw_t = 0.7;
                  break;
              case 'D':
                  //yaw_t -= 0.0005;
                  vyaw_t = -0.7;
                  break;
              case 'U':
                  use_mpc = true;
                  gaitCtrl.restart();
                  yaw_t = imuRPYd(2);
                  gaitCtrl.initSwingParams(0.6, 0.6, Eigen::Vector4d(0.5, 0, 0, 0.5), robot->getTime());
                  break;
              case 'I':
                  gaitCtrl.initSwingParams(0.6, 1.0, Eigen::Vector4d(0.5, 0, 0, 0.5), robot->getTime());
                  break;
              }
              key = keyboard->getKey();
          }
          Eigen::AngleAxisd rotationz_t(yaw_t, Eigen::Vector3d::UnitZ());
          /*vx_t = velFilter[0].f(slopeConstrain(vx_t, qp_body.est->getEstBodyVelB()(0), 0.5, -0.5));
          vy_t = velFilter[1].f(slopeConstrain(vy_t, qp_body.est->getEstBodyVelB()(1), 0.3, -0.3));
          vz_t = velFilter[2].f(slopeConstrain(vz_t, qp_body.est->getEstBodyVelB()(2), 0.2, -0.2));*/
          /*vx_t = velFilter[0].f(vx_t);
          vy_t = velFilter[1].f(vy_t);
          vz_t = velFilter[2].f(vz_t);*/
          vyaw_t = velFilter[3].f(slopeConstrain(vyaw_t, qp_ctrl.currentBalanceState.r_dot(2), 0.3, -0.3));
          Eigen::Vector3d real_vt(vx_t, vy_t, vz_t);
          static Vector3d last_real_vt = Eigen::Vector3d::Zero();
          static double last_vyaw_t = vyaw_t;
          real_vt = rotationz_t.toRotationMatrix() * real_vt;
          x_t += 0.5 * (last_real_vt(0) + real_vt(0)) * 0.001 * timeStep;
          y_t += 0.5 * (last_real_vt(1) + real_vt(1)) * 0.001 * timeStep;
          z_t += 0.5 * (last_real_vt(2) + real_vt(2)) * 0.001 * timeStep;
          yaw_t += 0.5 * (last_vyaw_t + vyaw_t) * 0.001 * timeStep;
          last_real_vt = real_vt;
          last_vyaw_t = vyaw_t;

          IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");
          IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");

          const double* imuRPY_data = imu->getRollPitchYaw();
          const double* imuQ_data = imu->getQuaternion();
          const double* gyro_data = gyro->getValues();
          const double* acc_data = acc->getValues();

          imuRPYd << static_cast<double>(imuRPY_data[0]), static_cast<double>(imuRPY_data[1]), static_cast<double>(imuRPY_data[2]);
          imuQd << static_cast<double>(imuQ_data[3]), static_cast<double>(imuQ_data[0]), static_cast<double>(imuQ_data[1]), static_cast<double>(imuQ_data[2]);
          gyrod << static_cast<double>(gyro_data[0]), static_cast<double>(gyro_data[1]), static_cast<double>(gyro_data[2]);
          accd << static_cast<double>(acc_data[0]), static_cast<double>(acc_data[1]), static_cast<double>(acc_data[2]);
          qp_body.updateBodyImu(imuQd);
          qp_body.updateBodyGyro(gyrod);
          qp_body.updateBodyAcc(accd);
          qp_body.calTbs(1);
          qp_body.bodyAndWorldFramePosition(1);
          qp_body.legAndBodyPosition(1);
          qp_body.legVelocityInWorldFrame();
          if (t > 0.2)
          {
//#if USE_WHEEL == 0
//              // 使用宇树教程观测器
//              Eigen::Matrix<double, 3, 1> estInput = qp_body.currentWorldState.linAcc_xyz + qp_body.g;
//              Eigen::Matrix<double, 28, 1> estObserve;
//              for (int i(0); i < 4; i++)
//              {
//                  estObserve.block<3, 1>(i * 3, 0) = qp_body.Rsb_c * qp_body.currentBodyState.leg_b[i].Position;
//                  estObserve.block<3, 1>(12 + i * 3, 0) = qp_body.currentWorldState.leg_s[i].Velocity;
//                  estObserve(24 + i, 0) = 0;
//              }
//              qp_body.est->estimatorRun(estInput, estObserve, contactResult, phaseResult);
//#else

              // 使用四轮足论文观测器
              Eigen::Matrix<double, 3, 1> estInput = qp_body.currentWorldState.linAcc_xyz + qp_body.g;
              Eigen::Matrix<double, 44, 1> estObserve;
              estObserve.setZero();
              for (int i(0); i < 4; i++)
              {
                  estObserve.block<3, 1>(i * 3, 0) = qp_body.Rsb_c * qp_body.currentBodyState.leg_b[i].Position;
                  estObserve.block<3, 1>(12 + i * 3, 0) = qp_body.currentWorldState.leg_s[i].Velocity;
                  estObserve.block<3, 1>(24 + i * 3, 0) = qp_body.currentWorldState.leg_s[i].VelocityW;
              }
              qp_body.est->estimatorRun(estInput, estObserve, contactResult, phaseResult);
//#endif
              ///*std::cout << "estState: \n" << qpest.estimatorState.transpose() << std::endl;
              //std::cout << "estOut:   \n" << qpest.estimatorOut.transpose() << std::endl;*/
              qp_body.currentWorldState.dist = qp_body.est->getEstBodyPosS();
              qp_body.currentWorldState.linVel_xyz = qp_body.est->getEstBodyVelS();
              qp_body.updateEqBody();
              qp_ctrl.updateDynamic();
          }
          /*std::cout << "mass:" << qp_body.M << std::endl;
          std::cout << "inerM:" << qp_body.I << std::endl;
          std::cout << "massP:" << qp_body.P << std::endl;
          std::cout << "estPos:" << qp_body.getEstFeetPos() << std::endl;*/
          /*std::cout << "tar:" << lf_leg_obj.targetLeg.Position << std::endl;
          std::cout << "cur:" << lf_leg_obj.currentLeg.Position << std::endl;
          std::cout << "f:" << lf_leg_obj.targetLeg.Force << std::endl;*/

          // 不适用平衡控制器和步态
          if (use_mpc == false)
          {
              p_t(0) = x_t;
              p_t(1) = y_t;
              p_t(2) = z_t;
              angle_t(0) = roll_t;
              angle_t(1) = pitch_t;
              angle_t(2) = yaw_t;
              qp_body.updateBodyTargetPos(angle_t, p_t);
              qp_body.calTbs(-1);
              qp_body.bodyAndWorldFramePosition(-1);
              qp_body.legAndBodyPosition(-1);
              /*double w = 10;
              double scale = 0.02;
              legsCtrl[0]->legObject->targetLeg.Position(0) += scale * sin(w*t);
              legsCtrl[0]->legObject->targetLeg.Position(1) += scale * sin(w*t);
              legsCtrl[0]->legObject->targetLeg.Position(2) += scale * sin(w*t);
              legsCtrl[0]->legObject->targetLeg.Velocity(0) = scale * w * cos(w*t);
              legsCtrl[0]->legObject->targetLeg.Velocity(1) = scale * w * cos(w*t);
              legsCtrl[0]->legObject->targetLeg.Velocity(2) = scale * w * cos(w*t);*/
          }

          Eigen::Vector4d encoderValue[4];
          Eigen::Vector4d motorSpeed[4];
          for (int i = 0; i < 4; i++)
          {
              for (int j = 0; j < 4; j++)
              {
                  encoderValue[i](j) = encoder[i][j]->getValue();
              }
              motorSpeed[i] = (encoderValue[i] - last_encoderValue[i]) / (0.001 * static_cast<double>(timeStep));
              last_encoderValue[i] = encoderValue[i];
              legsCtrl[i]->updateMotorAng(encoderValue[i]);
              legsCtrl[i]->updateMotorVel(motorSpeed[i]);
              legsCtrl[i]->legStateCal(imuRPYd,gyrod);
              if (use_mpc == false)
              {
                  legsCtrl[i]->legPvCtrlForce();
              }
          }

          if (t > 0.2)
          {
              slope.updatePoints(qp_body.getFKFeetPos(), contactResult);
              slope.estRun();

             /* bdp.updateBodyHighLevelTar(Eigen::Vector3d(x_t, y_t, z_t), real_vt);
              bdp.updateFootHighLevelTar(footPoint, Eigen::Matrix<double, 3, 4>::Zero());
              bdp.updateBodyState(qpest.getEstBodyPosS(), qpest.getEstBodyVelS());
              bdp.updateFootState(qp_body.getFKFeetPos(), qp_body.getFKFeetVel());
              bdp.updateBodyParams(qp_body.Rsb_c, 0.01, 0.5, traBQ.asDiagonal(), traBF.asDiagonal(), traBR.asDiagonal(), traBW.asDiagonal());
              bdp.updateFootParams(0.01, 0.42, 0.42, traLQ.asDiagonal(), traLF.asDiagonal(), traLR.asDiagonal(), traLW.asDiagonal());
              bdp.warmUp();
              bdp.setBodyInputConstrain(bodyAccL);
              bdp.setFootInputConstrain(footAccL);
              bdp.useBodyPlan();
              bdp.useFootPlan();*/

              bjp.updateBodyHighLevelTar(Eigen::Vector3d(x_t, y_t, z_t), real_vt);
              bjp.updateFootHighLevelTar(footPoint, Eigen::Matrix<double, 3, 4>::Zero());
              bjp.updateBodyState(qpest.getEstBodyPosS(), qpest.getEstBodyVelS());
              bjp.updateFootState(qp_body.getFKFeetPos(), qp_body.getFKFeetVel());
              bjp.updateWBodyState(qpest.getEstFootPosS(), qpest.getEstFootVelS());
              bjp.updateJointParams(qp_body.Rsb_c, 0.01, 0.85, 0.55, 0.35, traQ.asDiagonal(), traF.asDiagonal(), traR.asDiagonal(), traW.asDiagonal());
              bjp.warmUp();
              bjp.useJointPlan(accL);
              //std::cout << bjp.getFootPlanPosition() << std::endl;
              
              /*std::cout << "body: \n" << bjp.getBodyPlanPosition() << std::endl;
              std::cout << "foot: \n" << bjp.getFootPlanPosition() << std::endl;*/

              gaitCtrl.calcContactPhase(WaveStatus::WAVE_ALL, robot->getTime());
              gaitCtrl.setGait(bjp.getBodyPlanVelocity().segment(0,2), vyaw_t, 0.06);
              gaitCtrl.run(feetPos, feetVel, 0.5);

              qp_ctrl.updateBalanceState();
#if USE_WHEEL == 1
              /*Eigen::Vector4d wheeltar(qp_body.initLegsXYPosition(0, 0), qp_body.initLegsXYPosition(0, 1), qp_body.initLegsXYPosition(0, 2), qp_body.initLegsXYPosition(0, 3));
              qp_ctrl.setPositionTarget(Eigen::Vector3d(x_t, y_t, z_t), qp_body.Rsb_c.transpose()*qp_body.rotMatToRPY(slope.getSlopeRotation()) + Eigen::Vector3d(roll_t, pitch_t, yaw_t), wheeltar);
              qp_ctrl.setVelocityTarget(real_vt, Eigen::Vector3d(0, 0, vyaw_t), Eigen::Vector4d(vx_t, vx_t, vx_t, vx_t));*/
              
              //qp_body.updateLegsXYPosition(bdp.getFootPlanPosition());
              //Eigen::Vector4d wheeltar(qp_body.initLegsXYPosition(0, 0), qp_body.initLegsXYPosition(0, 1), qp_body.initLegsXYPosition(0, 2), qp_body.initLegsXYPosition(0, 3));
              ////qp_ctrl.setPositionTarget(bdp.getBodyPlanPosition(), qp_body.Rsb_c.transpose()* qp_body.rotMatToRPY(slope.getSlopeRotation()) + Eigen::Vector3d(roll_t, pitch_t, yaw_t), wheeltar);
              //qp_ctrl.setPositionTarget(bdp.getBodyPlanPosition(), Eigen::Vector3d(roll_t, pitch_t, yaw_t), wheeltar);
              //qp_ctrl.setVelocityTarget(bdp.getBodyPlanVelocity(), Eigen::Vector3d(0, 0, vyaw_t), Eigen::Vector4d((qp_body.Rsb_c.transpose()*bdp.getBodyPlanVelocity())(0), (qp_body.Rsb_c.transpose()* bdp.getBodyPlanVelocity())(0), (qp_body.Rsb_c.transpose()* bdp.getBodyPlanVelocity())(0), (qp_body.Rsb_c.transpose()* bdp.getBodyPlanVelocity())(0)));

              qp_body.updateLegsXYPosition(bjp.getFootPlanPosition());
              Eigen::Vector4d wheelPos(qp_body.initLegsXYPosition(0, 0), qp_body.initLegsXYPosition(0, 1), qp_body.initLegsXYPosition(0, 2), qp_body.initLegsXYPosition(0, 3));
              //Eigen::Vector4d wheelVel = bjp.getFootPlanVelocity().row(0);
              Eigen::Vector4d wheelVel = Eigen::Vector4d(bjp.getFootPlanVelocity()(0,0), bjp.getFootPlanVelocity()(0,1), bjp.getFootPlanVelocity()(0,2), bjp.getFootPlanVelocity()(0,3));
              qp_ctrl.setPositionTarget(bjp.getBodyPlanPosition(), qp_body.Rsb_c.transpose()* qp_body.rotMatToRPY(slope.getSlopeRotation()) + Eigen::Vector3d(roll_t, pitch_t, yaw_t), wheelPos);
              //qp_ctrl.setPositionTarget(bjp.getBodyPlanPosition(), Eigen::Vector3d(roll_t, pitch_t, yaw_t), wheelPos);
              qp_ctrl.setVelocityTarget(bjp.getBodyPlanVelocity(), Eigen::Vector3d(0, 0, vyaw_t), wheelVel);
              //std::cout << wheelVel << std::endl;
#else   
              qp_ctrl.setPositionTarget(Eigen::Vector3d(x_t, y_t, z_t), Eigen::Vector3d(roll_t, pitch_t, yaw_t));
              qp_ctrl.setVelocityTarget(Eigen::Vector3d(real_vt(0), real_vt(1), 0), Eigen::Vector3d(0, 0, vyaw_t));
#endif
              qp_ctrl.setContactConstrain(contactResult,legF);
              qp_ctrl.contactDeal(Q, 1, gaitCtrl.stRatio);
              //qp_ctrl.setContactConstrain(Eigen::Vector4i::Constant(1));
              Eigen::Vector<bool, 6> en;
              if (gaitCtrl.stRatio < 1.)
              {
                  en << true, true, true, true, true, true;
              }
              else
              {
                  en << true, true, true, true, true, true;
              }
              
              qp_ctrl.mpc_adjust(en);
              //Eigen::Vector4d wheeltar(footPoint(0, 0), footPoint(0, 1), footPoint(0, 2), footPoint(0, 3));
              //qp_ctrl.wheel_adjust(wheeltar, wheeltar);
          }


          if (use_mpc == true && t > 0.2)
          {
              Eigen::Vector4d wheelT;
              wheelT.setZero();
              qp_body.updateTargetFootPoint(feetPos);
              //feetVel.setZero();
              qp_body.updateTargetFootVel(feetVel);
              qp_body.legAndBodyPosition(-1);
              //wheelT = qp_ctrl.wheelTau;
              for (int i = 0; i < 4; i++)
              {
                  if (contactResult(i) == 0)
                  {
                      legF.col(i) = legsCtrl[i]->legPvCtrlForceR();
                      //wheelT(i) = 0;
#if USE_WHEEL == 1
                      wheelT(i) = qp_ctrl.mpcOut(3, i);
#endif
                  }
                  else
                  {
                      legF.col(i) = qp_ctrl.mpcOut.block(0,i,3,1);
#if USE_WHEEL == 1
                      wheelT(i) = qp_ctrl.mpcOut(3, i);
#endif
                  }
              }

#if USE_WHEEL == 1
              qp_ctrl.setLegsForce(legF,wheelT);
#else
              qp_ctrl.setLegsForce(legF, wheelT);
#endif
          }

          for (int i = 0; i < 4; i++)
          {
              for (int k = 0; k < 4; k++)
              {
                  if (k < 2)
                  {
                      motors[i][k]->setTorque(upper::constrain(legsObj[i]->targetJoint.Torque(k), 200));
                  }
                  else if(k == 2)
                  {
                      motors[i][k]->setTorque(upper::constrain(legsObj[i]->targetJoint.Torque(k), 320));
                  }
#if USE_WHEEL == 1
                  else
                  {
                      motors[i][k]->setTorque(upper::constrain(legsObj[i]->targetJoint.Foot_Torque, 20));
                  }
#endif
              }
          }

          float data[DNUM];
          /*data[0] = float(qp_ctrl.bodyObject->est->getEstFeetPosS()(0, 0));
          data[1] = float(qp_ctrl.bodyObject->est->getEstFeetPosS()(1, 0));
          data[2] = float(qp_ctrl.bodyObject->est->getEstFeetPosS()(2, 0));
          data[3] = float(qp_ctrl.bodyObject->getFKFeetPos()(0,0));
          data[4] = float(qp_ctrl.bodyObject->getFKFeetPos()(1, 0));
          data[5] = float(qp_ctrl.bodyObject->getFKFeetPos()(2, 0));
          data[6] = float(gaitCtrl.contact(0));
          data[7] = float(qp_ctrl.bodyObject->legs[0]->targetLeg.Position(0));
          data[8] = float(qp_ctrl.bodyObject->legs[0]->targetLeg.Position(1));
          data[9] = float(qp_ctrl.bodyObject->legs[0]->targetLeg.Position(2));
          data[10] = float(qp_ctrl.bodyObject->legs[0]->currentLeg.Position(0));
          data[11] = float(qp_ctrl.bodyObject->legs[0]->currentLeg.Position(1));
          data[12] = float(qp_ctrl.bodyObject->legs[0]->currentLeg.Position(2));*/
          /*data[0] = bdp.getMpcOriOut()[0];
          data[1] = bdp.getMpcOriOut()[1];
          data[2] = bdp.getMpcOriOut()[2];
          data[3] = bdp.getBodyPlanVelocity()[0];
          data[4] = bdp.getBodyPlanVelocity()[1];
          data[5] = bdp.getBodyPlanVelocity()[2];
          data[6] = bdp.getBodyPlanPosition()[0];
          data[7] = bdp.getBodyPlanPosition()[1];
          data[8] = bdp.getBodyPlanPosition()[2];*/
          //data[0] = float(qp_ctrl.currentBalanceState.r_dot(2));
          //data[1] = float(qp_ctrl.currentBalanceState.p_dot(0));
          //data[2] = float(qp_ctrl.currentBalanceState.p_dot(1));
          //data[3] = float(bjp.getBodyPlanVelocity()(0));
          //data[4] = float(bjp.getBodyPlanVelocity()(1));
          //data[5] = float(bjp.getBodyPlanVelocity()(2));
          //data[6] = float(/*velFilterN[0].f(*/qp_ctrl.bodyObject->est->getEstBodyVelS()(0, 0) - qp_ctrl.bodyObject->est->getEstFootVelS()(0, 0)/*)*/);
          //data[7] = float(/*velFilterN[1].f(*/qp_ctrl.bodyObject->est->getEstBodyVelS()(1, 0) - qp_ctrl.bodyObject->est->getEstFootVelS()(1, 0)/*)*/);
          //data[8] = float(/*velFilterN[2].f(*/qp_ctrl.bodyObject->est->getEstBodyVelS()(2, 0) - qp_ctrl.bodyObject->est->getEstFootVelS()(2, 0)/*)*/);
          //vofa.dataTransmit(data, 5);
      }
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
