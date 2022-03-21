/*****************************************************************
 Copyright (c) 2021, Korea Institute of Science and Technology (KIST). All
rights reserved.
******************************************************************/
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <fstream>
#include <iostream>

#include "ControlParameters/ControlParameterInterface.h"
#include "ControlParameters/RobotParameters.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"
#include "KIST_Controller.hpp"
#include "kist_legged_robot.h"
using namespace std;
using namespace UNITREE_LEGGED_SDK;
#define data 60

class RobotRunner {
 public:
  RobotRunner(uint8_t level, KIST_Controller* kist_ctrl)
      : safe(LeggedType::A1), udp(level) {
    udp.InitCmdData(cmd);
    _robot_ctrl = kist_ctrl;
  }

  virtual ~RobotRunner();

  void Initialize();
  void UDPRecv();
  void UDPSend();
  void setupStep();
  void finalizeStep();
  void Run();
  void initializeStateEstimator();
  void saveLogData(int motiontime);

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

 public:
  float logdata[data][100000];  // 12 joint position,velocity, imu,com, com dot,
  string fileName[data] = {"FR_hip_q.txt",
                         "FR_thigh_q.txt",
                         "FR_calf_q.txt",
                         "FL_hip_q.txt",
                         "FL_thigh_q.txt",
                         "FL_calf_q.txt",
                         "RR_hip_q.txt",
                         "RR_thigh_q.txt",
                         "RR_calf_q.txt",
                         "RL_hip_q.txt",
                         "RL_thigh_q.txt",
                         "RL_calf_q.txt",
                         "FR_hip_qdot.txt",
                         "FR_thigh_qdot.txt",
                         "FR_calf_qdot.txt",
                         "FL_hip_qdot.txt",
                         "FL_thigh_qdot.txt",
                         "FL_calf_qdot.txt",
                         "RR_hip_qdot.txt",
                         "RR_thigh_qdot.txt",
                         "RR_calf_qdot.txt",
                         "RL_hip_qdot.txt",
                         "RL_thigh_qdot.txt",
                         "RL_calf_qdot.txt",
                         "Quaternion_w.txt",
                         "Quaternion_x.txt",
                         "Quaternion_y.txt",
                         "Quaternion_z.txt",
                         "Gyro_x.txt",
                         "Gyro_y.txt",
                         "Gyro_z.txt",
                         "Accel_x.txt",
                         "Accel_y.txt",
                         "Accel_z.txt",
                         "CoM_x.txt",
                         "CoM_y.txt",
                         "CoM_z.txt",
                         "CoM_xdot.txt",
                         "CoM_ydot.txt",
                         "CoM_zdot.txt",
                         "CoM_lp_x.txt",
                         "CoM_lp_y.txt",
                         "CoM_lp_z.txt",
                         "RR_hip_torque.txt",
                         "RR_thigh_torque.txt",
                         "RR_calf_torque.txt",
                         "RL_hip_torque.txt",
                         "RL_thigh_torque.txt",
                         "RL_calf_torque.txt",
                         "CoM_x_desired.txt",
                         "CoM_y_desired.txt",
                         "CoM_z_desired.txt",
                         "omega_b_x.txt",
                         "omega_b_y.txt",
                         "omega_b_z.txt",
                         "omega_b_x_desired.txt",
                         "omega_b_y_desired.txt",
                         "omega_b_z_desired.txt"};

};
