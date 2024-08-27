#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Compass.hpp>
#include <webots/Keyboard.hpp>
#include "LegCtrl.h"
#include "RobotParams.h"
#include "Body.h"
#include "kalmanFilter.h"
#include "BodyCtrl.h"

using namespace webots;
using namespace Quadruped;
using namespace std;

int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // get keyboard
  Keyboard *keyboard = robot->getKeyboard();
  keyboard->enable(timeStep);

  // get imu
  InertialUnit *imu = robot->getInertialUnit("trunk_imu inertial");
  imu->enable(timeStep);
  Gyro *gyro = robot->getGyro("trunk_imu gyro");
  gyro->enable(timeStep);
  Accelerometer *acc = robot->getAccelerometer("trunk_imu accelerometer");
  acc->enable(timeStep);
  Compass *compass = robot->getCompass("trunk_imu compass");
  compass->enable(timeStep);

  // get motor
  Motor *lf_hip = robot->getMotor("FL_hip_joint");
  Motor *lf_thigh = robot->getMotor("FL_thigh_joint");
  Motor *lf_calf = robot->getMotor("FL_calf_joint");
  lf_hip->setPosition(INFINITY);
  lf_thigh->setPosition(INFINITY);
  lf_calf->setPosition(INFINITY);
  lf_hip->setVelocity(0);
  lf_thigh->setVelocity(0);
  lf_calf->setVelocity(0);
  Motor *rf_hip = robot->getMotor("FR_hip_joint");
  Motor *rf_thigh = robot->getMotor("FR_thigh_joint");
  Motor *rf_calf = robot->getMotor("FR_calf_joint");
  rf_hip->setPosition(INFINITY);
  rf_thigh->setPosition(INFINITY);
  rf_calf->setPosition(INFINITY);
  rf_hip->setVelocity(0);
  rf_thigh->setVelocity(0);
  rf_calf->setVelocity(0);
  Motor *lb_hip = robot->getMotor("RL_hip_joint");
  Motor *lb_thigh = robot->getMotor("RL_thigh_joint");
  Motor *lb_calf = robot->getMotor("RL_calf_joint");
  lb_hip->setPosition(INFINITY);
  lb_thigh->setPosition(INFINITY);
  lb_calf->setPosition(INFINITY);
  lb_hip->setVelocity(0);
  lb_thigh->setVelocity(0);
  lb_calf->setVelocity(0);
  Motor *rb_hip = robot->getMotor("RR_hip_joint");
  Motor *rb_thigh = robot->getMotor("RR_thigh_joint");
  Motor *rb_calf = robot->getMotor("RR_calf_joint");
  rb_hip->setPosition(INFINITY);
  rb_thigh->setPosition(INFINITY);
  rb_calf->setPosition(INFINITY);
  rb_hip->setVelocity(0);
  rb_thigh->setVelocity(0);
  rb_calf->setVelocity(0);

  // get position sensor
  PositionSensor *lf_hip_ps = robot->getPositionSensor("FL_hip_joint_sensor");
  PositionSensor *lf_thigh_ps = robot->getPositionSensor("FL_thigh_joint_sensor");
  PositionSensor *lf_calf_ps = robot->getPositionSensor("FL_calf_joint_sensor");
  lf_hip_ps->enable(timeStep);
  lf_thigh_ps->enable(timeStep);
  lf_calf_ps->enable(timeStep);
  PositionSensor *rf_hip_ps = robot->getPositionSensor("FR_hip_joint_sensor");
  PositionSensor *rf_thigh_ps = robot->getPositionSensor("FR_thigh_joint_sensor");
  PositionSensor *rf_calf_ps = robot->getPositionSensor("FR_calf_joint_sensor");
  rf_hip_ps->enable(timeStep);
  rf_thigh_ps->enable(timeStep);
  rf_calf_ps->enable(timeStep);
  PositionSensor *lb_hip_ps = robot->getPositionSensor("RL_hip_joint_sensor");
  PositionSensor *lb_thigh_ps = robot->getPositionSensor("RL_thigh_joint_sensor");
  PositionSensor *lb_calf_ps = robot->getPositionSensor("RL_calf_joint_sensor");
  lb_hip_ps->enable(timeStep);
  lb_thigh_ps->enable(timeStep);
  lb_calf_ps->enable(timeStep);
  PositionSensor *rb_hip_ps = robot->getPositionSensor("RR_hip_joint_sensor");
  PositionSensor *rb_thigh_ps = robot->getPositionSensor("RR_thigh_joint_sensor");
  PositionSensor *rb_calf_ps = robot->getPositionSensor("RR_calf_joint_sensor");
  rb_hip_ps->enable(timeStep);
  rb_thigh_ps->enable(timeStep);
  rb_calf_ps->enable(timeStep);

  // self controll classes
  Leg lf_leg_obj(Quadruped::L1, Quadruped::L2, Quadruped::L3, 1);
  LegCtrl lf_leg_ctrl(&lf_leg_obj, timeStep);
  Eigen::Vector3f lastAngle_lf = Eigen::Vector3f::Zero(); // 零初始化一个上次角度
  lf_leg_ctrl.setEndPositionTar(Eigen::Vector3f(0, 0.0838, -0.27));

  Leg rf_leg_obj(Quadruped::L1, Quadruped::L2, Quadruped::L3, -1);
  LegCtrl rf_leg_ctrl(&rf_leg_obj, timeStep);
  Eigen::Vector3f lastAngle_rf = Eigen::Vector3f::Zero(); // 零初始化一个上次角度
  rf_leg_ctrl.setEndPositionTar(Eigen::Vector3f(0, -0.0838, -0.27));

  Leg lb_leg_obj(Quadruped::L1, Quadruped::L2, Quadruped::L3, 1);
  LegCtrl lb_leg_ctrl(&lb_leg_obj, timeStep);
  Eigen::Vector3f lastAngle_lb = Eigen::Vector3f::Zero(); // 零初始化一个上次角度
  lb_leg_ctrl.setEndPositionTar(Eigen::Vector3f(0, 0.0838, -0.27));

  Leg rb_leg_obj(Quadruped::L1, Quadruped::L2, Quadruped::L3, -1);
  LegCtrl rb_leg_ctrl(&rb_leg_obj, timeStep);
  Eigen::Vector3f lastAngle_rb = Eigen::Vector3f::Zero(); // 零初始化一个上次角度
  rb_leg_ctrl.setEndPositionTar(Eigen::Vector3f(0, -0.0838, -0.27));

  Body qp_body(&lf_leg_obj, &rf_leg_obj, &lb_leg_obj, &rb_leg_obj, static_cast<float>(timeStep) * 0.001f);
  qp_body.initParams(leg2bodyFrame, Eigen::Vector3f(-0.1805, -0.1308, 0), Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(), Eigen::Vector3f::Zero());
  Vector3f footPoint[4] = {Vector3f(0.361, 0.2616, 0.), Vector3f(0.361, 0., 0.), Vector3f(0., 0.2616, 0.), Vector3f(0., 0., 0.)};
  qp_body.updateTargetFootPoint(footPoint);
  Vector3f angle_t(0, 0, 0);
  Vector3f p_t(0, 0, 0.27);
  float x_t = 0;
  float y_t = 0;
  float z_t = 0.27;
  float roll_t = 0;
  float pitch_t = 0;
  float yaw_t = 0;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1)
  {
    // 键盘控制
    int key = keyboard->getKey();
    while (key > 0)
    {
      switch (key)
      {
      case keyboard->UP:
        x_t += 0.0002;
        break;
      case keyboard->DOWN:
        x_t -= 0.0002;
        break;
      case keyboard->RIGHT:
        y_t -= 0.0002;
        break;
      case keyboard->LEFT:
        y_t += 0.0002;
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
        yaw_t += 0.0005;
        break;
      case 'D':
        yaw_t -= 0.0005;
        break;
      case 'U':
        break;
      case 'I':
        break;
      }
      key = keyboard->getKey();
    }

    IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");
    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
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
    // std::cout<<"lf_targets:"<<std::endl;
    // std::cout<<lf_leg_obj.targetLeg.Position.format(CommaInitFmt)<<std::endl;
    // std::cout<<qp_body.targetBodyState.leg_b[LF].Position.format(CommaInitFmt)<<std::endl;
    // std::cout<<"rf_targets:"<<std::endl;
    // std::cout<<rf_leg_obj.targetLeg.Position.format(CommaInitFmt)<<std::endl;
    // std::cout<<qp_body.targetBodyState.leg_b[RF].Position.format(CommaInitFmt)<<std::endl;
    // std::cout<<"lb_targets:"<<std::endl;
    // std::cout<<lb_leg_obj.targetLeg.Position.format(CommaInitFmt)<<std::endl;
    // std::cout<<qp_body.targetBodyState.leg_b[LB].Position.format(CommaInitFmt)<<std::endl;
    // std::cout<<"rb_targets:"<<std::endl;
    // std::cout<<rb_leg_obj.targetLeg.Position.format(CommaInitFmt)<<std::endl;
    // std::cout<<qp_body.targetBodyState.leg_b[RB].Position.format(CommaInitFmt)<<std::endl;

    const double *imu_data = imu->getRollPitchYaw();
    const double *gyro_data = gyro->getValues();
    const double *acc_data = acc->getValues();

    Vector3f imud(static_cast<float>(imu_data[0]), static_cast<float>(imu_data[1]), static_cast<float>(imu_data[2]));
    Vector3f gyrod(static_cast<float>(gyro_data[0]), static_cast<float>(gyro_data[1]), static_cast<float>(gyro_data[2]));
    Vector3f accd(static_cast<float>(acc_data[0]), static_cast<float>(acc_data[1]), static_cast<float>(acc_data[2]));
    // std::cout<<"imu:"<<std::endl;
    // std::cout<<imud.format(CommaInitFmt)<<std::endl;
    // std::cout<<"gyro:"<<std::endl;
    // std::cout<<gyrod.format(CommaInitFmt)<<std::endl;
    // std::cout<<"acc:"<<std::endl;
    // std::cout<<accd.format(CommaInitFmt)<<std::endl;
    qp_body.updateBodyImu(imud);
    qp_body.updateBodyGyro(gyrod);
    qp_body.updateBodyAcc(accd);
    qp_body.calTbs(1);
    qp_body.bodyAndWorldFramePosition(1);
    qp_body.legAndBodyPosition(1);
    qp_body.legVelocityInWorldFrame();
    qp_body.estimatorRun();
    std::cout<<"estimatorOut:"<<std::endl;
    std::cout<<qp_body.estimatorOut.segment(0,3).format(CommaInitFmt)<<std::endl;
    std::cout<<qp_body.currentWorldState.leg_s[0].Position.format(CommaInitFmt)<<std::endl;
    std::cout<<qp_body.estimator.getState().segment(0,3).format(CommaInitFmt)<<std::endl;

    Eigen::Vector3f positionSensor_lf;
    positionSensor_lf(0) = lf_hip_ps->getValue();
    positionSensor_lf(1) = lf_thigh_ps->getValue();
    positionSensor_lf(2) = lf_calf_ps->getValue();
    Eigen::Vector3f motorSpeed_lf = (positionSensor_lf - lastAngle_lf) / (0.001f * static_cast<float>(timeStep));
    lastAngle_lf = positionSensor_lf;
    lf_leg_ctrl.updateMotorAng(positionSensor_lf);
    lf_leg_ctrl.updateMotorVel(motorSpeed_lf);
    lf_leg_ctrl.legStateCal();
    lf_leg_ctrl.legCtrlMix();
    lf_hip->setTorque(upper::constrain(lf_leg_obj.targetJoint.Torque(0), 43));
    lf_thigh->setTorque(upper::constrain(lf_leg_obj.targetJoint.Torque(1), 43));
    lf_calf->setTorque(upper::constrain(lf_leg_obj.targetJoint.Torque(2), 43));

    // cout<<"current:"<<endl;
    // cout << lf_leg_obj.currentJoint.Angle.format(CommaInitFmt) << "\n" << endl;
    // cout<<"target:"<<endl;
    // cout << lf_leg_obj.targetJoint.Angle.format(CommaInitFmt) << "\n" << endl;

    Eigen::Vector3f positionSensor_rf;
    positionSensor_rf(0) = rf_hip_ps->getValue();
    positionSensor_rf(1) = rf_thigh_ps->getValue();
    positionSensor_rf(2) = rf_calf_ps->getValue();
    Eigen::Vector3f motorSpeed_rf = (positionSensor_rf - lastAngle_rf) / (0.001f * static_cast<float>(timeStep));
    lastAngle_rf = positionSensor_rf;
    rf_leg_ctrl.updateMotorAng(positionSensor_rf);
    rf_leg_ctrl.updateMotorVel(motorSpeed_rf);
    rf_leg_ctrl.legStateCal();
    rf_leg_ctrl.legCtrlMix();
    rf_hip->setTorque(upper::constrain(rf_leg_obj.targetJoint.Torque(0), 43));
    rf_thigh->setTorque(upper::constrain(rf_leg_obj.targetJoint.Torque(1), 43));
    rf_calf->setTorque(upper::constrain(rf_leg_obj.targetJoint.Torque(2), 43));

    Eigen::Vector3f positionSensor_lb;
    positionSensor_lb(0) = lb_hip_ps->getValue();
    positionSensor_lb(1) = lb_thigh_ps->getValue();
    positionSensor_lb(2) = lb_calf_ps->getValue();
    Eigen::Vector3f motorSpeed_lb = (positionSensor_lb - lastAngle_lb) / (0.001f * static_cast<float>(timeStep));
    lastAngle_lb = positionSensor_lb;
    lb_leg_ctrl.updateMotorAng(positionSensor_lb);
    lb_leg_ctrl.updateMotorVel(motorSpeed_lb);
    lb_leg_ctrl.legStateCal();
    lb_leg_ctrl.legCtrlMix();
    lb_hip->setTorque(upper::constrain(lb_leg_obj.targetJoint.Torque(0), 43));
    lb_thigh->setTorque(upper::constrain(lb_leg_obj.targetJoint.Torque(1), 43));
    lb_calf->setTorque(upper::constrain(lb_leg_obj.targetJoint.Torque(2), 43));

    Eigen::Vector3f positionSensor_rb;
    positionSensor_rb(0) = rb_hip_ps->getValue();
    positionSensor_rb(1) = rb_thigh_ps->getValue();
    positionSensor_rb(2) = rb_calf_ps->getValue();
    Eigen::Vector3f motorSpeed_rb = (positionSensor_rb - lastAngle_rb) / (0.001f * static_cast<float>(timeStep));
    lastAngle_rb = positionSensor_rb;
    rb_leg_ctrl.updateMotorAng(positionSensor_rb);
    rb_leg_ctrl.updateMotorVel(motorSpeed_rb);
    rb_leg_ctrl.legStateCal();
    rb_leg_ctrl.legCtrlMix();
    rb_hip->setTorque(upper::constrain(rb_leg_obj.targetJoint.Torque(0), 43));
    rb_thigh->setTorque(upper::constrain(rb_leg_obj.targetJoint.Torque(1), 43));
    rb_calf->setTorque(upper::constrain(rb_leg_obj.targetJoint.Torque(2), 43));
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
