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
#include "SecondButterworthLPF.h"
#include "virtualLegCtrl.h"
#include "dataDisplay.h"
#include "RobotEst.h"
#include "BodyCtrlNew.h"



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
  lf_leg_ctrl.setEndPositionTar(Eigen::Vector3d(0.0130447 + XMOVE + XSPAN, 0.234832 + YSPAN, -0.5937));

  Leg rf_leg_obj(links, p_mass, mass, iner, Quadruped::RF);
  LegCtrl rf_leg_ctrl(&rf_leg_obj, timeStep);
  rf_leg_ctrl.setEndPositionTar(Eigen::Vector3d(0.0130447 + XMOVE + XSPAN, -0.234832 - YSPAN, -0.5937));

  Leg lb_leg_obj(links, p_mass, mass, iner, Quadruped::LB);
  LegCtrl lb_leg_ctrl(&lb_leg_obj, timeStep);
  lb_leg_ctrl.setEndPositionTar(Eigen::Vector3d(0.0130447 + XMOVE - XSPAN, 0.234832 + YSPAN, -0.5937));

  Leg rb_leg_obj(links, p_mass, mass, iner, Quadruped::RB);
  LegCtrl rb_leg_ctrl(&rb_leg_obj, timeStep);
  rb_leg_ctrl.setEndPositionTar(Eigen::Vector3d(0.0130447 + XMOVE - XSPAN, -0.234832 - YSPAN, -0.5937));

  QpwEst qpest(static_cast<double>(timeStep) * 0.001f);
  Leg* legsObj[4] = { &lf_leg_obj, &rf_leg_obj, &lb_leg_obj, &rb_leg_obj };
  Body qp_body(&qpest,legsObj, static_cast<double>(timeStep) * 0.001f);
  double mb[3] = { Quadruped::Mmid, Quadruped::Mhead, Quadruped::Mtail };
  Vector<double, 6> ib[3] = { Quadruped::Imid, Quadruped::Ihead, Quadruped::Itail };
  Vector3d pb[3] = { Quadruped::Pmid, Quadruped::Phead, Quadruped::Ptail };
  qp_body.initParams(leg2bodyFrame, initRbLegXYPosition, mb, ib, pb);
  Eigen::Matrix<double, 3, 4> footPoint;
  footPoint.col(LF) = Vector3d(0.3415447 + XMOVE + XSPAN, 0.234832 + YSPAN, 0);
  footPoint.col(RF) = Vector3d(0.3415447 + XMOVE + XSPAN, -0.234832 - YSPAN, 0);
  footPoint.col(LB) = Vector3d(-0.3154553 + XMOVE - XSPAN, 0.234832 + YSPAN, 0);
  footPoint.col(RB) = Vector3d(-0.3154553 + XMOVE - XSPAN, -0.234832 - YSPAN, 0);
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
  QpwCtrl qp_ctrl(&qp_body, legsCtrl, timeStep);
  qp_ctrl.importWeight(Q, F, R, W);
#if USE_WHEEL == 1
  qp_ctrl.importPDparam(linPD, angPD,wheelPID);
#else
  qp_ctrl.importPDparam(linPD, angPD);
#endif
  bool use_mpc = false;

  Vector4d phaseResult;
  Vector4i contactResult;
  phaseResult.setZero();
  contactResult.setOnes();
  GaitCtrl gaitCtrl(&qp_ctrl, legsCtrl, timeStep, &phaseResult, &contactResult);
  gaitCtrl.initSwingParams(0.8, 1., Eigen::Vector4d(0.5, 0, 0, 0.5), robot->getTime());
  gaitCtrl.initExpectK(gaitK);
  Eigen::Matrix<double, 3, 4> feetPos;
  Eigen::Matrix<double, 3, 4> feetVel;

  VOFA vofa("vjs.exe");

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

  // 重新配置腿部曲线跟踪参数
  Vector2d lPid_pvParams[6];
  lPid_pvParams[0] << 4000, 200;
  lPid_pvParams[1] << 80, 200;
  lPid_pvParams[2] << 4000, 200;
  lPid_pvParams[3] << 80, 200;
  lPid_pvParams[4] << 4000, 500;
  lPid_pvParams[5] << 80, 500;
  for (auto p : legsCtrl)
  {
      p->loadPid_pvParams(p->lPid_pv, lPid_pvParams);
  }

  qp_ctrl.force_c = 1000;
  qp_ctrl.u = 2;

  //LPF_SecondOrder_Classdef velFilter[3] = { LPF_SecondOrder_Classdef(5,500),LPF_SecondOrder_Classdef(5,500) ,LPF_SecondOrder_Classdef(5,500) };
  MeanFilter<100> velFilter[3];
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
      // 使用gps计算加速度
      double t = robot->getTime();

      double vx_t = 0;
      double vy_t = 0;
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
                  vx_t = 1.2;
                  break;
              case keyboard->DOWN:
                  //x_t -= 0.0002;
                  vx_t = -1.2;
                  break;
              case keyboard->RIGHT:
                  //y_t -= 0.0002;
                  vy_t = -0.2;
                  break;
              case keyboard->LEFT:
                  //y_t += 0.0002;
                  vy_t = 0.2;
                  break;
              case (keyboard->SHIFT + keyboard->RIGHT):
                  roll_t += 0.0005;
                  break;
              case (keyboard->SHIFT + keyboard->LEFT):
                  roll_t -= 0.0005;
                  break;
              case (keyboard->SHIFT + keyboard->UP):
                  z_t += 0.0002;
                  break;
              case (keyboard->SHIFT + keyboard->DOWN):
                  z_t -= 0.0002;
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
                  vyaw_t = 0.6;
                  break;
              case 'D':
                  //yaw_t -= 0.0005;
                  vyaw_t = -0.6;
                  break;
              case 'U':
                  use_mpc = true;
                  gaitCtrl.restart();
                  yaw_t = imuRPYd(2);
                  break;
              case 'I':
                  break;
              }
              key = keyboard->getKey();
          }
          Eigen::AngleAxisd rotationz_t(velFilter[2].f(yaw_t), Eigen::Vector3d::UnitZ());
          Eigen::Vector3d real_vt(velFilter[0].f(vx_t), velFilter[1].f(vy_t), 0);
          /*float data[DNUM];
          data[0] = real_vt(0);
          data[1] = real_vt(1);
          data[2] = real_vt(2);*/
          real_vt = rotationz_t.toRotationMatrix() * real_vt;
          x_t += real_vt(0) * 0.001 * timeStep;
          y_t += real_vt(1) * 0.001 * timeStep;
          yaw_t += vyaw_t * 0.001 * timeStep;

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
              // 使用宇树教程观测器
              /*Eigen::Matrix<double, 3, 1> estInput = qp_body.currentWorldState.linAcc_xyz + qp_body.g;
              Eigen::Matrix<double, 28, 1> estObserve;
              for (int i(0); i < 4; i++)
              {
                  estObserve.block<3, 1>(i * 3, 0) = qp_body.Rsb_c * qp_body.currentBodyState.leg_b[i].Position;
                  estObserve.block<3, 1>(12 + i * 3, 0) = qp_body.currentWorldState.leg_s[i].Velocity;
                  estObserve(24 + i, 0) = 0;
              }
              qp_body.est->estimatorRun(estInput, estObserve, contactResult, phaseResult);
              qp_body.currentWorldState.dist = qp_body.est->getEstBodyPosS();
              qp_body.currentWorldState.linVel_xyz = qp_body.est->getEstBodyVelS();*/

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
              /*std::cout << "estState: \n" << qpest.estimatorState.transpose() << std::endl;
              std::cout << "estOut:   \n" << qpest.estimatorOut.transpose() << std::endl;*/
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
              gaitCtrl.calcContactPhase(WaveStatus::WAVE_ALL, robot->getTime());
              gaitCtrl.setGait(Vector2d(real_vt(0), real_vt(1)), vyaw_t, 0.03);
              gaitCtrl.run(feetPos, feetVel);

              qp_ctrl.updateBalanceState();
#if USE_WHEEL == 1
              qp_ctrl.setPositionTarget(Eigen::Vector3d(x_t, y_t, z_t), Eigen::Vector3d(roll_t, pitch_t, yaw_t), Eigen::Vector4d(footPoint(0,0), footPoint(0,1), footPoint(0,2), footPoint(0,3)));
              qp_ctrl.setVelocityTarget(Eigen::Vector3d(real_vt(0), real_vt(1), 0), Eigen::Vector3d(0, 0, vyaw_t), Eigen::Vector4d(vx_t, vx_t, vx_t, vx_t));
#else   
              qp_ctrl.setPositionTarget(Eigen::Vector3d(x_t, y_t, z_t), Eigen::Vector3d(roll_t, pitch_t, yaw_t));
              qp_ctrl.setVelocityTarget(Eigen::Vector3d(real_vt(0), real_vt(1), 0), Eigen::Vector3d(0, 0, vyaw_t));
#endif
              qp_ctrl.setContactConstrain(contactResult);
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
          }


          if (use_mpc == true && t > 0.2)
          {
              Eigen::Matrix<double, 3, 4> legF;
              Eigen::Vector4d wheelT;
              legF.setZero();
              wheelT.setZero();
              qp_body.updateTargetFootPoint(feetPos);
              qp_body.updateTargetFootVel(feetVel);
              qp_body.legAndBodyPosition(-1);
              for (int i = 0; i < 4; i++)
              {
                  if (contactResult(i) == 0)
                  {
                      legF.col(i) = legsCtrl[i]->legPvCtrlForceR();
                      wheelT(i) = 0;
                  }
                  else
                  {
                      legF.col(i) = qp_ctrl.mpcOut.block(0,i,3,1);
                      wheelT(i) = qp_ctrl.mpcOut(3, i);
                  }
              }
              /*std::cout << "legF:  \n" << legF << std::endl;
              std::cout << "wheelT:\n" << wheelT << std::endl;
              std::cout << "wheetV:\n" << legsObj[0]->currentJoint.Foot_Velocity << ", " << legsObj[1]->currentJoint.Foot_Velocity
                  << ", " << legsObj[2]->currentJoint.Foot_Velocity << ", " << legsObj[3]->currentJoint.Foot_Velocity << std::endl;*/
              /*std::cout << "dis: \n" << qpest.getEstBodyPosS().transpose() << std::endl;
              std::cout << "vel: \n" << qpest.getEstBodyVelS().transpose() << std::endl;
              std::cout << "wdis:\n" << qpest.getEstFootPosS().transpose() << std::endl;
              std::cout << "wvel:\n" << qpest.getEstFootVelS().transpose() << std::endl;*/
#if USE_WHEEL == 1
              qp_ctrl.setLegsForce(legF,wheelT);
#else
              qp_ctrl.setLegsForce(legF);
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

          /*float data[DNUM];
          data[0] = float(vx_t);
          data[1] = float(vy_t);
          data[2] = float(vyaw_t);*/
          /*data[0] = float(qp_ctrl.bodyObject->legs[0]->targetLeg.Position(0));
          data[1] = float(qp_ctrl.bodyObject->legs[0]->targetLeg.Position(1));
          data[2] = float(qp_ctrl.bodyObject->legs[0]->targetLeg.Position(2));*/
          /*data[0] = float(qp_ctrl.currentBalanceState.p_dot(0));
          data[1] = float(qp_ctrl.currentBalanceState.p_dot(1));
          data[2] = float(qp_ctrl.currentBalanceState.p_dot(2));
          data[3] = float(qp_ctrl.bodyObject->legs[0]->currentLeg.Position(0));
          data[4] = float(qp_ctrl.bodyObject->legs[0]->currentLeg.Position(1));
          data[5] = float(qp_ctrl.bodyObject->legs[0]->currentLeg.Position(2));
          data[6] = float(qp_ctrl.bodyObject->legs[0]->targetLeg.Velocity(0));
          data[7] = float(qp_ctrl.bodyObject->legs[0]->targetLeg.Velocity(1));
          data[8] = float(qp_ctrl.bodyObject->legs[0]->targetLeg.Velocity(2));
          data[9] = float(qp_ctrl.bodyObject->legs[0]->currentLeg.Velocity(0));
          data[10] = float(qp_ctrl.bodyObject->legs[0]->currentLeg.Velocity(1));
          data[11] = float(qp_ctrl.bodyObject->legs[0]->currentLeg.Velocity(2));
          data[12] = float(gaitCtrl.contact(0));
          data[13] = float(gaitCtrl.phase(0));*/
          /*data[9] = float(qp_ctrl.targetBalanceState.p(0));
          data[10] = float(qp_ctrl.targetBalanceState.p(1));
          data[11] = float(qp_ctrl.targetBalanceState.p(2));
          data[12] = float(qp_ctrl.currentBalanceState.p(0));
          data[13] = float(qp_ctrl.currentBalanceState.p(1));
          data[14] = float(qp_ctrl.currentBalanceState.p(2));
          data[15] = float(qp_ctrl.targetBalanceState.p_dot(0));
          data[16] = float(qp_ctrl.targetBalanceState.p_dot(1));
          data[17] = float(qp_ctrl.targetBalanceState.p_dot(2));
          data[18] = float(qp_ctrl.currentBalanceState.p_dot(0));
          data[19] = float(qp_ctrl.currentBalanceState.p_dot(1));
          data[20] = float(qp_ctrl.currentBalanceState.p_dot(2));*/
          //vofa.dataTransmit(data, 5);
      }
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
