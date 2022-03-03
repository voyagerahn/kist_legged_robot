#ifndef KIST_A1_LOCOMOTION_H
#define KIST_A1_LOCOMOTION_H

#include <Controllers/FootSwingTrajectory.h>
#include <FSM_States/ControlFSMData.h>
#include <SparseCMPC/SparseCMPC.h>
#include "cppTypes.h"
#include "Gait.h"
#include "../BalanceController/BalanceController.hpp"
#include <cstdio>

using Eigen::Array4f;
using Eigen::Array4i;


template<typename T>
struct CLocomotion_Result {
  LegControllerCommand<T> commands[4];
  Vec4<T> contactPhase;
};

class Locomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Locomotion(float _dt,KIST_UserParameters* parameters);
  void initialize();

  template<typename T>
  void run(ControlFSMData<T>& data);
  bool currently_jumping = false;

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> height_des;
  Vec3<float> foot_target_positions;
  Vec3<float> v_des_world;
  Vec3<float> hip_offset;
  Vec3<float> twisting_vector;
  Vec3<float> hip_horizontal_velocity;
  Vec3<float> zero;
  float foot_clearance = 0.01; //The foot clearance on the ground at the end of the swing cycle
  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;

  BalanceController balanceController;
  double minForce = 0.1;
  double maxForce = 1000;
  double contactStateScheduled[4];  // = {1, 1, 1, 1};
  double minForces[4];  // = {minForce, minForce, minForce, minForce};
  double maxForces[4];  // = {maxForce, maxForce, maxForce, maxForce};
  double COM_weights_stance[3] = {1, 1, 10};
  double Base_weights_stance[3] = {20, 10, 10};
  double pFeet[12], p_des[3], p_act[3], v_des[3], v_act[3], O_err[3], rpy[3],
      omegaDes[3];
  double p_tmp[3];
  double se_xfb[13];
  double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
  
  Vec3<float> pFeetVec;
  Vec3<float> pFeetVecCOM;
  double fOpt[12];

 private:
  void _SetupCommand(ControlFSMData<float> & data);

  float _yaw_turn_rate;
  float _yaw_des;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  // High speed running
  //float _body_height = 0.34;
  float _body_height = 0.29;

  float _body_height_running = 0.29;
  float _body_height_jumping = 0.36;

  Vec3<float> getJointAngleFromFootPosition(Vec3<float> pDes, int leg);
 
  float dt;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  CLocomotion_Result<float> result;
  float trajAll[12*36];

  KIST_UserParameters* _parameters = nullptr;
//   CMPC_Jump jump_state;



};


#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H