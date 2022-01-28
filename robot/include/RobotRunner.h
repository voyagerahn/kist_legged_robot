/*****************************************************************
 Copyright (c) 2021, Korea Institute of Science and Technology (KIST). All
rights reserved.
******************************************************************/
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include "ControlParameters/ControlParameterInterface.h"
#include "ControlParameters/RobotParameters.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"
#include "kist_legged_robot.h"
#include "KIST_Controller.hpp"
using namespace std;
using namespace UNITREE_LEGGED_SDK;

class RobotRunner {
 public:
  RobotRunner(uint8_t level, KIST_Controller* kist_ctrl) : safe(LeggedType::A1), udp(level) {
    udp.InitCmdData(cmd);
    _robot_ctrl = kist_ctrl;
    // Initialize();
  }

  virtual ~RobotRunner();

  void Initialize();
  void UDPRecv();
  void UDPSend();
  void setupStep();
  void finalizeStep();
  void Run();
  void initializeStateEstimator();
  // double jointLinearInterpolation();

  Safety safe;
  UDP udp;
  IMU imu;
  LowCmd cmd = {0};
  LowState state = {0};
  Vector4i contact_modes;
  int motiontime = 0;
  float _time = 0.0;
  float dt = 0.002;
  float q[12] = {0};
  float dq[12] = {0};
  // float quat[4] = {0};
  float gyro[3] = {0};
  float accel[3] = {0};

  KIST_Controller* _robot_ctrl;
  RobotControlParameters* controlParameters;
  clock_t start, end;
  double result;
 private:
  Quadruped<float> _quadruped;
  LegController<float>* _legController = nullptr;
  StateEstimate<float> _stateEstimate;
  StateEstimatorContainer<float>* _stateEstimator;
  FloatingBaseModel<float> _model;

  // void ContactForceOptimization(double mass, Matrix3d inertiaRot, VectorCXd
  // x_dist, VectorCXd acc_des, VectorCXd Fc_prev, VectorCXd &OptFc); //m, I_G
  // (3x3), [p_1-p_c .. p_4 - p_c]T (12x1), [p_ddot_c w_dot_b]T, F^star
};
