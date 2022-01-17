#include "RobotRunner.h"

#include <fstream>

#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "Dynamics/A1.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

void RobotRunner::Initialize() {
  _quadruped = buildA1<float>();
  _model = _quadruped.buildModel();

  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(
      &imu, _legController->datas, &_stateEstimate, controlParameters);

  initializeStateEstimator();

  // TODO :   Initialize the DesiredStateCommand object
  // -----------------------------example-----------------------
  // _desiredStateCommand = new DesiredStateCommand<float>(
  //     driverCommand, &rc_control, controlParameters, &_stateEstimate,
  //     controlParameters->controller_dt);

  // Controller initializations
  _robot_ctrl->_model = &_model;
  _robot_ctrl->_quadruped = &_quadruped;
  _robot_ctrl->_legController = _legController;
  _robot_ctrl->_stateEstimator = _stateEstimator;
  _robot_ctrl->_stateEstimate = &_stateEstimate;
  _robot_ctrl->_controlParameters = controlParameters;
  // _kist_ctrl->_desiredStateCommand = _desiredStateCommand;

  _robot_ctrl->initializeController();
}

void RobotRunner::initializeStateEstimator() {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);
  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
}

void RobotRunner::UDPRecv() { udp.Recv(); }

void RobotRunner::UDPSend() { udp.Send(); }

void RobotRunner::Run() {
  // _stateEstimator->run();

  setupStep();
  // udp.GetRecv(state);
  _time = (float)motiontime * dt;

  _robot_ctrl->runController();
  finalizeStep();
  // cout << "state:" << state.imu.quaternion[2] << endl;
  // safe.PositionLimit(cmd);
  // safe.PowerProtect(cmd, state, 6);
  // udp.SetSend(cmd);
}

void RobotRunner::setupStep() {
  udp.GetRecv(state);
  _legController->updateData(&state);
}

void RobotRunner::finalizeStep() {
  _legController->updateCommand(&cmd);

  safe.PositionLimit(cmd);
  safe.PowerProtect(cmd, state, 6);
  udp.SetSend(cmd);
  motiontime++;
}

RobotRunner::~RobotRunner() {
  delete _legController;
  delete _stateEstimator;
}
// string fileName[data] = {
//     "FL_hip_joint_position.txt",
//     "FL_thigh_joint_position.txt",
//     "FL_calf_joint_position.txt",
//     "FR_hip_joint_position.txt",
//     "FR_thigh_joint_position.txt",
//     "FR_calf_joint_position.txt",
//     "RL_hip_joint_position.txt",
//     "RL_thigh_joint_position.txt",
//     "RL_calf_joint_position.txt",
//     "RR_hip_joint_position.txt",
//     "RR_thigh_joint_position.txt",
//     "RR_calf_joint_position.txt",
//     "FL_hip_joint_velocity.txt",
//     "FL_thigh_joint_velocity.txt",
//     "FL_calf_joint_velocity.txt",
//     "FR_hip_joint_velocity.txt",
//     "FR_thigh_joint_velocity.txt",
//     "FR_calf_joint_velocity.txt",
//     "RL_hip_joint_velocity.txt",
//     "RL_thigh_joint_velocity.txt",
//     "RL_calf_joint_velocity.txt",
//     "RR_hip_joint_velocity.txt",
//     "RR_thigh_joint_velocity.txt",
//     "RR_calf_joint_velocity.txt",
//     "Quaternion_w.txt",
//     "Quaternion_x.txt",
//     "Quaternion_y.txt",
//     "Quaternion_z.txt",
//     "Gyro_x.txt",
//     "Gyro_y.txt",
//     "Gyro_z.txt",
//     "Accel_x.txt",
//     "Accel_y.txt",
//     "Accel_z.txt",
//     "COM_x.txt",
//     "COM_y.txt",
//     "COM_z.txt",
//     "COM_vel_x.txt",
//     "COM_vel_y.txt",
//     "COM_vel_z.txt",
//     "COM_vel_lp_x.txt",
//     "COM_vel_lp_y.txt",
//     "COM_vel_lp_z.txt",
//     "RL_hip_torque.txt",
//     "RL_thigh_torque.txt",
//     "RL_calf_torque.txt",
//     "RR_hip_torque.txt",
//     "RR_thigh_torque.txt",
//     "RR_calf_torque.txt",
//     "COM_x_desired.txt",
//     "COM_y_desired.txt",
//     "COM_z_desired.txt",
//     "omega_b_x.txt",
//     "omega_b_y.txt",
//     "omega_b_z.txt",
//     "omega_b_x_desired.txt",
//     "omega_b_y_desired.txt",
//     "omega_b_z_desired.txt",
//     "orientation_x_error.txt",
//     "orientation_y_error.txt",
//     "orientation_z_error.txt"
//     };

//  if (_time >= 0 && _time < 0.02) {
//     _FL_qInit[0] = state.motorState[FL_0].q;
//     _FL_qInit[1] = state.motorState[FL_1].q;
//     _FL_qInit[2] = state.motorState[FL_2].q;

//     _FR_qInit[0] = state.motorState[FR_0].q;
//     _FR_qInit[1] = state.motorState[FR_1].q;
//     _FR_qInit[2] = state.motorState[FR_2].q;

//     _RL_qInit[0] = state.motorState[RL_0].q;
//     _RL_qInit[1] = state.motorState[RL_1].q;
//     _RL_qInit[2] = state.motorState[RL_2].q;

//     _RR_qInit[0] = state.motorState[RR_0].q;
//     _RR_qInit[1] = state.motorState[RR_1].q;
//     _RR_qInit[2] = state.motorState[RR_2].q;
//   } else if (_time >= 0.02 && _time < 1.0)  // stand up
//   {
//     rate_count++;
//     double rate = rate_count / 500.0;  // needs count to 500

//     _FL_qDes[0] = jointLinearInterpolation(_FL_qInit[0], _sin_mid_q[0],
//     rate); _FL_qDes[1] = jointLinearInterpolation(_FL_qInit[1],
//     _sin_mid_q[1], rate); _FL_qDes[2] =
//     jointLinearInterpolation(_FL_qInit[2], _sin_mid_q[2], rate);

//     _FR_qDes[0] = jointLinearInterpolation(_FR_qInit[0], _sin_mid_q[0],
//     rate); _FR_qDes[1] = jointLinearInterpolation(_FR_qInit[1],
//     _sin_mid_q[1], rate); _FR_qDes[2] =
//     jointLinearInterpolation(_FR_qInit[2], _sin_mid_q[2], rate);

//     _RL_qDes[0] = jointLinearInterpolation(_RL_qInit[0], _sin_mid_q[0],
//     rate); _RL_qDes[1] = jointLinearInterpolation(_RL_qInit[1],
//     _sin_mid_q[1], rate); _RL_qDes[2] =
//     jointLinearInterpolation(_RL_qInit[2], _sin_mid_q[2], rate);

//     _RR_qDes[0] = jointLinearInterpolation(_RR_qInit[0], _sin_mid_q[0],
//     rate); _RR_qDes[1] = jointLinearInterpolation(_RR_qInit[1],
//     _sin_mid_q[1], rate); _RR_qDes[2] =
//     jointLinearInterpolation(_RR_qInit[2], _sin_mid_q[2], rate);

//     _torque(0) = _Kp[0] * (_FL_qDes[0] - state.motorState[FL_0].q) +
//                  _Kd[0] * (0 - state.motorState[FL_0].dq);
//     _torque(1) = _Kp[0] * (_FL_qDes[1] - state.motorState[FL_1].q) +
//                  _Kd[0] * (0 - state.motorState[FL_1].dq);
//     _torque(2) = _Kp[0] * (_FL_qDes[2] - state.motorState[FL_2].q) +
//                  _Kd[0] * (0 - state.motorState[FL_2].dq);
//     _torque(3) = _Kp[0] * (_FR_qDes[0] - state.motorState[FR_0].q) +
//                  _Kd[0] * (0 - state.motorState[FR_0].dq);
//     _torque(4) = _Kp[0] * (_FR_qDes[1] - state.motorState[FR_1].q) +
//                  _Kd[0] * (0 - state.motorState[FR_1].dq);
//     _torque(5) = _Kp[0] * (_FR_qDes[2] - state.motorState[FR_2].q) +
//                  _Kd[0] * (0 - state.motorState[FR_2].dq);
//     _torque(6) = _Kp[0] * (_RL_qDes[0] - state.motorState[RL_0].q) +
//                  _Kd[0] * (0 - state.motorState[RL_0].dq);
//     _torque(7) = _RL_Kp[1] * (_RL_qDes[1] - state.motorState[RL_1].q) +
//                  _Kd[0] * (0 - state.motorState[RL_1].dq);
//     _torque(8) = _RL_Kp[2] * (_RL_qDes[2] - state.motorState[RL_2].q) +
//                  _Kd[0] * (0 - state.motorState[RL_2].dq);
//     _torque(9) = _Kp[0] * (_RR_qDes[0] - state.motorState[RR_0].q) +
//                  _Kd[0] * (0 - state.motorState[RR_0].dq);
//     _torque(10) = _RR_Kp[1] * (_RR_qDes[1] - state.motorState[RR_1].q) +
//                   _Kd[0] * (0 - state.motorState[RR_1].dq);
//     _torque(11) = _RR_Kp[2] * (_RR_qDes[2] - state.motorState[RR_2].q) +
//                   _Kd[0] * (0 - state.motorState[RR_2].dq);

//     for (int i = 0; i < 12; i++) {
//       _torque(i, 0) =
//           (_torque(i, 0) > torque_limit) ? torque_limit : _torque(i, 0);
//       _torque(i, 0) =
//           (_torque(i, 0) < -torque_limit) ? -torque_limit : _torque(i, 0);
//     }

//     cmd.motorCmd[FL_0].tau = _torque(0);
//     cmd.motorCmd[FL_1].tau = _torque(1);
//     cmd.motorCmd[FL_2].tau = _torque(2);
//     cmd.motorCmd[FR_0].tau = _torque(3);
//     cmd.motorCmd[FR_1].tau = _torque(4);
//     cmd.motorCmd[FR_2].tau = _torque(5);
//     cmd.motorCmd[RL_0].tau = _torque(6);
//     cmd.motorCmd[RL_1].tau = _torque(7);
//     cmd.motorCmd[RL_2].tau = _torque(8);
//     cmd.motorCmd[RR_0].tau = _torque(9);
//     cmd.motorCmd[RR_1].tau = _torque(10);
//     cmd.motorCmd[RR_2].tau = _torque(11);
//   } else if (_time >= 1.0 && _time < 5.0) {
//     _q << 0, 0, 0,  // base x, y, z hip: 0 thigh: 1 calf: 2
//         state.imu.quaternion[1], state.imu.quaternion[2],
//         state.imu.quaternion[3], state.motorState[FL_0].q,
//         state.motorState[FL_1].q, state.motorState[FL_2].q,
//         state.motorState[FR_0].q, state.motorState[FR_1].q,
//         state.motorState[FR_2].q, state.motorState[RL_0].q,
//         state.motorState[RL_1].q, state.motorState[RL_2].q,
//         state.motorState[RR_0].q, state.motorState[RR_1].q,
//         state.motorState[RR_2].q, state.imu.quaternion[0];
//     _dq << 0, 0, 0,  //  base x, y ,z
//         state.imu.quaternion[1], state.imu.quaternion[2],
//         state.imu.quaternion[3], state.motorState[FL_0].dq,
//         state.motorState[FL_1].dq, state.motorState[FL_2].dq,
//         state.motorState[FR_0].dq, state.motorState[FR_1].dq,
//         state.motorState[FR_2].dq, state.motorState[RL_0].dq,
//         state.motorState[RL_1].dq, state.motorState[RL_2].dq,
//         state.motorState[RR_0].dq, state.motorState[RR_1].dq,
//         state.motorState[RR_2].dq;

//     m.calculate_COM(_q, _dq);
//     _desired_COM = m._COM;
//     _desired_COM_vel = m._COM_vel;
//     _quat << state.imu.quaternion[0], state.imu.quaternion[1],
//         state.imu.quaternion[2], state.imu.quaternion[3];
//     quaternion_to_rotationMatrix(_desired_R, _quat);
//     _desired_w << state.imu.gyroscope[0], state.imu.gyroscope[1],
//         state.imu.gyroscope[2];

//     _FL_qDes[0] = _sin_mid_q[0];
//     _FL_qDes[1] = _sin_mid_q[1];
//     _FL_qDes[2] = _sin_mid_q[2];
//     _FR_qDes[0] = _sin_mid_q[0];
//     _FR_qDes[1] = _sin_mid_q[1];
//     _FR_qDes[2] = _sin_mid_q[2];
//     _RL_qDes[0] = _sin_mid_q[0];
//     _RL_qDes[1] = _sin_mid_q[1];
//     _RL_qDes[2] = _sin_mid_q[2];
//     _RR_qDes[0] = _sin_mid_q[0];
//     _RR_qDes[1] = _sin_mid_q[1];
//     _RR_qDes[2] = _sin_mid_q[2];

//     _torque(0) = _Kp[0] * (_FL_qDes[0] - state.motorState[FL_0].q) +
//                  (0 - state.motorState[FL_0].dq) * _Kd[0];
//     _torque(1) = _Kp[0] * (_FL_qDes[1] - state.motorState[FL_1].q) +
//                  (0 - state.motorState[FL_1].dq) * _Kd[0];
//     _torque(2) = _Kp[0] * (_FL_qDes[2] - state.motorState[FL_2].q) +
//                  (0 - state.motorState[FL_2].dq) * _Kd[0];
//     _torque(3) = _Kp[0] * (_FR_qDes[0] - state.motorState[FR_0].q) +
//                  (0 - state.motorState[FR_0].dq) * _Kd[0];
//     _torque(4) = _Kp[0] * (_FR_qDes[1] - state.motorState[FR_1].q) +
//                  (0 - state.motorState[FR_1].dq) * _Kd[0];
//     _torque(5) = _Kp[0] * (_FR_qDes[2] - state.motorState[FR_2].q) +
//                  (0 - state.motorState[FR_2].dq) * _Kd[0];
//     _torque(6) = _Kp[0] * (_RL_qDes[0] - state.motorState[RL_0].q) +
//                  (0 - state.motorState[RL_0].dq) * _Kd[0];
//     _torque(7) = _RL_Kp[1] * (_RL_qDes[1] - state.motorState[RL_1].q) +
//                  (0 - state.motorState[RL_1].dq) * _Kd[0];
//     _torque(8) = _RL_Kp[2] * (_RL_qDes[2] - state.motorState[RL_2].q) +
//                  (0 - state.motorState[RL_2].dq) * _Kd[0];
//     _torque(9) = _Kp[0] * (_RR_qDes[0] - state.motorState[RR_0].q) +
//                  (0 - state.motorState[RR_0].dq) * _Kd[0];
//     _torque(10) = _RR_Kp[1] * (_RR_qDes[1] - state.motorState[RR_1].q) +
//                   (0 - state.motorState[RR_1].dq) * _Kd[0];
//     _torque(11) = _RR_Kp[2] * (_RR_qDes[2] - state.motorState[RR_2].q) +
//                   (0 - state.motorState[RR_2].dq) * _Kd[0];

//     for (int i = 0; i < 12; i++) {
//       _torque(i, 0) =
//           (_torque(i, 0) > torque_limit) ? torque_limit : _torque(i, 0);
//       _torque(i, 0) =
//           (_torque(i, 0) < -torque_limit) ? -torque_limit : _torque(i, 0);
//     }
//     cmd.motorCmd[FL_0].tau = _torque(0);
//     cmd.motorCmd[FL_1].tau = _torque(1);
//     cmd.motorCmd[FL_2].tau = _torque(2);
//     cmd.motorCmd[FR_0].tau = _torque(3);
//     cmd.motorCmd[FR_1].tau = _torque(4);
//     cmd.motorCmd[FR_2].tau = _torque(5);
//     cmd.motorCmd[RL_0].tau = _torque(6);
//     cmd.motorCmd[RL_1].tau = _torque(7);
//     cmd.motorCmd[RL_2].tau = _torque(8);
//     cmd.motorCmd[RR_0].tau = _torque(9);
//     cmd.motorCmd[RR_1].tau = _torque(10);
//     cmd.motorCmd[RR_2].tau = _torque(11);
//   } else if (_time >= 5.0) {
//     _COM.setZero();
//     _COM_vel.setZero();
//     _pddot.setZero(3);
//     _wdot.setZero(3);
//     _w.setZero(3);
//     // _R.setIdentity();

//     _q << 0, 0, 0,  // base x, y, z hip: 0 thigh: 1 calf: 2
//         state.imu.quaternion[1], state.imu.quaternion[2],
//         state.imu.quaternion[3], state.motorState[FL_0].q,
//         state.motorState[FL_1].q, state.motorState[FL_2].q,
//         state.motorState[FR_0].q, state.motorState[FR_1].q,
//         state.motorState[FR_2].q, state.motorState[RL_0].q,
//         state.motorState[RL_1].q, state.motorState[RL_2].q,
//         state.motorState[RR_0].q, state.motorState[RR_1].q,
//         state.motorState[RR_2].q, state.imu.quaternion[0];

//     _dq << 0, 0, 0,  //  base x, y ,z
//         state.imu.quaternion[1], state.imu.quaternion[2],
//         state.imu.quaternion[3], state.motorState[FL_0].dq,
//         state.motorState[FL_1].dq, state.motorState[FL_2].dq,
//         state.motorState[FR_0].dq, state.motorState[FR_1].dq,
//         state.motorState[FR_2].dq, state.motorState[RL_0].dq,
//         state.motorState[RL_1].dq, state.motorState[RL_2].dq,
//         state.motorState[RR_0].dq, state.motorState[RR_1].dq,
//         state.motorState[RR_2].dq;

//     m.update_kinematics(_q, _dq);
//     m.update_dynamics();
//     m.calculate_EE_Jacobians();
//     m.calculate_EE_positions_orientations();
//     m.calculate_foot_Jacobians();
//     m.calculate_COM(_q, _dq);

//     I_g(0, 0) = m._A(3, 3);
//     I_g(0, 1) = m._A(3, 4);
//     I_g(0, 2) = m._A(3, 5);
//     I_g(1, 0) = m._A(4, 3);
//     I_g(1, 1) = m._A(4, 4);
//     I_g(1, 2) = m._A(4, 5);
//     I_g(2, 0) = m._A(5, 3);
//     I_g(2, 1) = m._A(5, 4);
//     I_g(2, 2) = m._A(5, 5);
//     _quat << state.imu.quaternion[0], state.imu.quaternion[1],
//         state.imu.quaternion[2], state.imu.quaternion[3];
//     quaternion_to_rotationMatrix(_R, _quat);
//     _COM = m._COM;
//     _COM_vel = m._COM_vel;
//     _COM_vel_lp(0) = CustomMath::VelLowpassFilter(
//         dt, 2.0 * M_PI * 10.0, _pre_COM(0), _COM(0), _pre_COM_vel(0));
//     _COM_vel_lp(1) = CustomMath::VelLowpassFilter(
//         dt, 2.0 * M_PI * 10.0, _pre_COM(1), _COM(1), _pre_COM_vel(1));
//     _COM_vel_lp(2) = CustomMath::VelLowpassFilter(
//         dt, 2.0 * M_PI * 10.0, _pre_COM(2), _COM(2), _pre_COM_vel(2));
//     _pre_COM = _COM;
//     _pre_COM_vel = _COM_vel_lp;

//     matrixLogRot(_desired_R * _R.transpose(), _orientation_error);
//     _w << state.imu.gyroscope[0], state.imu.gyroscope[1],
//         state.imu.gyroscope[2];

//     for (int i = 0; i < 12; i++) {
//       // FL: 6~8 FR: 9~11 RL: 12~14 RR: 15~17
//       _mem_fprint[i][motiontime] = _q(i + 6);        // joint position
//       _mem_fprint[i + 12][motiontime] = _dq(i + 6);  // joint velocity
//     }
//     _mem_fprint[24][motiontime] = _q(18);                  // quaternion w
//     _mem_fprint[25][motiontime] = _q(3);                   // x
//     _mem_fprint[26][motiontime] = _q(4);                   // y
//     _mem_fprint[27][motiontime] = _q(5);                   // z
//     _mem_fprint[28][motiontime] = state.imu.gyroscope[0];  // gyro
//     _mem_fprint[29][motiontime] = state.imu.gyroscope[1];
//     _mem_fprint[30][motiontime] = state.imu.gyroscope[2];
//     _mem_fprint[31][motiontime] = state.imu.accelerometer[0];  // accel
//     _mem_fprint[32][motiontime] = state.imu.accelerometer[1];
//     _mem_fprint[33][motiontime] = state.imu.accelerometer[2];
//     _mem_fprint[34][motiontime] = _COM(0);
//     _mem_fprint[35][motiontime] = _COM(1);
//     _mem_fprint[36][motiontime] = _COM(2);
//     _mem_fprint[37][motiontime] = _COM_vel(0);
//     _mem_fprint[38][motiontime] = _COM_vel(1);
//     _mem_fprint[39][motiontime] = _COM_vel(2);
//     _mem_fprint[40][motiontime] = _COM_vel_lp(0);
//     _mem_fprint[41][motiontime] = _COM_vel_lp(1);
//     _mem_fprint[42][motiontime] = _COM_vel_lp(2);
//     _mem_fprint[49][motiontime] = _desired_COM(0);
//     _mem_fprint[50][motiontime] = _desired_COM(1);
//     _mem_fprint[51][motiontime] = _desired_COM(2);
//     _mem_fprint[52][motiontime] = _w(0);
//     _mem_fprint[53][motiontime] = _w(1);
//     _mem_fprint[54][motiontime] = _w(2);
//     _mem_fprint[55][motiontime] = _desired_w(0);
//     _mem_fprint[56][motiontime] = _desired_w(1);
//     _mem_fprint[57][motiontime] = _desired_w(2);
//     _mem_fprint[58][motiontime] = _orientation_error(0);
//     _mem_fprint[59][motiontime] = _orientation_error(1);
//     _mem_fprint[60][motiontime] = _orientation_error(2);

//     // p_i - p_c
//     x_dist.setZero();
//     x_dist_1_tmp.setZero();
//     x_dist_2_tmp.setZero();
//     x_dist_3_tmp.setZero();
//     x_dist_4_tmp.setZero();
//     x_dist_1.setZero();
//     x_dist_2.setZero();
//     x_dist_3.setZero();
//     x_dist_4.setZero();

//     x_dist << m._x_left_front_leg - _COM, m._x_right_front_leg - _COM,
//         m._x_left_rear_leg - _COM, m._x_right_rear_leg - _COM;

//     x_dist_1_tmp << x_dist(0), x_dist(1), x_dist(2);
//     x_dist_2_tmp << x_dist(3), x_dist(4), x_dist(5);
//     x_dist_3_tmp << x_dist(6), x_dist(7), x_dist(8);
//     x_dist_4_tmp << x_dist(9), x_dist(10), x_dist(11);

//     x_dist_1 = skew(x_dist_1_tmp);
//     x_dist_2 = skew(x_dist_2_tmp);
//     x_dist_3 = skew(x_dist_3_tmp);
//     x_dist_4 = skew(x_dist_4_tmp);
//     MatrixCXd A(6, 12);
//     VectorCXd b(6);

//     S.setIdentity();
//     I_3.setIdentity();
//     gamma1.setIdentity();
//     gamma2.setIdentity();
//     gravity.setZero();
//     gravity(2) = -9.81;
//     A.setZero();
//     b.setZero();
//     _torque.setZero();

//     _pddot(0) = Kp_COMx * (_desired_COM(0) - _COM(0)) +
//                 Kd_COMx * (_desired_COM_vel(0) - _COM_vel_lp(0));
//     _pddot(1) = Kp_COMy * (_desired_COM(1) - _COM(1)) +
//                 Kd_COMy * (_desired_COM_vel(1) - _COM_vel_lp(1));
//     _pddot(2) = Kp_COMz * (_desired_COM(2) - _COM(2)) +
//                 Kd_COMz * (_desired_COM_vel(2) - _COM_vel_lp(2));
//     _wdot(0) = Kp_Base_roll * _orientation_error(0) +
//                Kd_Base_roll * (_desired_w(0) - _w(0));
//     _wdot(1) = Kp_Base_pitch * _orientation_error(1) +
//                Kd_Base_pitch * (_desired_w(1) - _w(1));
//     _wdot(2) = Kp_Base_yaw * _orientation_error(2) +
//                Kd_Base_yaw * (_desired_w(2) - _w(2));
//     // acc_des << _pddot(0), _pddot(1), _pddot(2), 0, 0, 0;
//     acc_des << _pddot(0), _pddot(1), _pddot(2), _wdot(0), _wdot(1), _wdot(2);
//     // acc_des << 0, 0, 0, 0, 0, 0;

//     A << I_3, I_3, I_3, I_3, x_dist_1, x_dist_2, x_dist_3, x_dist_4;
//     S = s * S;
//     gamma1 = gamma_1 * gamma1;
//     gamma2 = gamma_2 * gamma2;
//     mI_3 = I_3 * mass;

//     // mass * (p_ddot_c+g)
//     b(0) = (mI_3(0, 0) * acc_des(0)) + (mass * gravity(0));
//     b(1) = (mI_3(1, 1) * acc_des(1)) + (mass * gravity(1));
//     b(2) = (mI_3(2, 2) * acc_des(2)) + (mass * gravity(2));
//     // I_g * W_dot_b
//     b(3) = (I_g(0, 0) * acc_des(3)) + (I_g(0, 1) * acc_des(4)) +
//            (I_g(0, 2) * acc_des(5));
//     b(4) = (I_g(1, 0) * acc_des(3)) + (I_g(1, 1) * acc_des(4)) +
//            (I_g(1, 2) * acc_des(5));
//     b(5) = (I_g(2, 0) * acc_des(3)) + (I_g(2, 1) * acc_des(4)) +
//            (I_g(2, 2) * acc_des(5));

//     _H = (A.transpose() * S * A) + gamma1 + gamma2;  // 12x12
//     //_g = ((b.transpose() * S * A) + (F_frev.transpose() *
//     // gamma2)).transpose(); //12x1
//     _g = ((A.transpose() * S * b) + (F_frev * gamma_2));
//     _A.setZero();
//     _lbA.setZero();
//     _ubA.setZero();

//     // calcualte A<16,12>
//     for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
//       _A.block<1, 3>(4 * i + 0, 3 * i)
//           << -friction_coeff * direction_normal_flatGround.transpose() +
//                  t1x.transpose();
//       _A.block<1, 3>(4 * i + 1, 3 * i)
//           << -friction_coeff * direction_normal_flatGround.transpose() +
//                  t2y.transpose();
//       _A.block<1, 3>(4 * i + 2, 3 * i)
//           << friction_coeff * direction_normal_flatGround.transpose() +
//                  t2y.transpose();
//       _A.block<1, 3>(4 * i + 3, 3 * i)
//           << friction_coeff * direction_normal_flatGround.transpose() +
//                  t1x.transpose();
//     }

//     // calculate lbA ubA
//     for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
//       _lbA(NUM_CONSTRAINTS_PER_FOOT * i) = contact_state(i) *
//       NEGATIVE_NUMBER; _lbA(NUM_CONSTRAINTS_PER_FOOT * i + 1) =
//           contact_state(i) * NEGATIVE_NUMBER;
//       _lbA(NUM_CONSTRAINTS_PER_FOOT * i + 2) = 0;
//       _lbA(NUM_CONSTRAINTS_PER_FOOT * i + 3) = 0;

//       _ubA(NUM_CONSTRAINTS_PER_FOOT * i) = 0;
//       _ubA(NUM_CONSTRAINTS_PER_FOOT * i + 1) = 0;
//       _ubA(NUM_CONSTRAINTS_PER_FOOT * i + 2) =
//           contact_state(i) * POSITIVE_NUMBER;
//       _ubA(NUM_CONSTRAINTS_PER_FOOT * i + 3) =
//           contact_state(i) * POSITIVE_NUMBER;
//     }

//     // calculate lb ub
//     _lb.setZero();
//     _ub.setZero();
//     _lb.setConstant(NEGATIVE_NUMBER);
//     _ub.setConstant(POSITIVE_NUMBER);
//     _lb(2) = normal_force_threashold;
//     _lb(5) = normal_force_threashold;
//     _lb(8) = normal_force_threashold;
//     _lb(11) = normal_force_threashold;

//     ForceOptimization.UpdateMinProblem(_H, _g);
//     // ForceOptimization.PrintMinProb();
//     ForceOptimization.UpdateSubjectToAx(_A, _lbA, _ubA);
//     ForceOptimization.UpdateSubjectToX(_lb, _ub);
//     // ForceOptimization.PrintSubjectToAx();
//     // ForceOptimization.PrintSubjectTox();
//     ForceOptimization.SolveQPoases(_max_iter);
//     // ForceOptimization.EnablePrintOptionDebug();
//     OptFc = ForceOptimization._Xopt;
//     _torque = m._J_foot.transpose() * (-OptFc);

//     for (int i = 0; i < 12; i++) {
//       _torque(i, 0) =
//           (_torque(i, 0) > torque_limit) ? torque_limit : _torque(i, 0);
//       _torque(i, 0) =
//           (_torque(i, 0) < -torque_limit) ? -torque_limit : _torque(i, 0);
//     }

//     _mem_fprint[43][motiontime] = _torque(6);
//     _mem_fprint[44][motiontime] = _torque(7);
//     _mem_fprint[45][motiontime] = _torque(8);
//     _mem_fprint[46][motiontime] = _torque(9);
//     _mem_fprint[47][motiontime] = _torque(10);
//     _mem_fprint[48][motiontime] = _torque(11);

//     cmd.motorCmd[FL_0].tau = _torque(0);
//     cmd.motorCmd[FL_1].tau = _torque(1);
//     cmd.motorCmd[FL_2].tau = _torque(2);

//     cmd.motorCmd[FR_0].tau = _torque(3);
//     cmd.motorCmd[FR_1].tau = _torque(4);
//     cmd.motorCmd[FR_2].tau = _torque(5);

//     cmd.motorCmd[RL_0].tau = _torque(6);
//     cmd.motorCmd[RL_1].tau = _torque(7);
//     cmd.motorCmd[RL_2].tau = _torque(8);

//     cmd.motorCmd[RR_0].tau = _torque(9);
//     cmd.motorCmd[RR_1].tau = _torque(10);
//     cmd.motorCmd[RR_2].tau = _torque(11);
//   }