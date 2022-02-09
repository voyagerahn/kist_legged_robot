#ifndef CUSTOM_MODEL_H
#define CUSTOM_MODEL_H
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

class CModel {
 public:
  CModel();
  ~CModel(){};

  void update_kinematics(Eigen::VectorXd& q, Eigen::VectorXd& qdot);
  void update_dynamics();
  void calculate_EE_positions_orientations();
  void calculate_EE_Jacobians();
  void calculate_EE_velocity();
  void calculate_foot_Jacobians();
  void calculate_COM(Eigen::VectorXd& q, Eigen::VectorXd& qdot);

  RigidBodyDynamics::Model _model;
  RigidBodyDynamics::Math::Vector3d _COM;
  RigidBodyDynamics::Math::Vector3d _COM_vel;
  Eigen::MatrixXd _A;  // inertia matrix
  Eigen::VectorXd _g;  // gravity force vector
  Eigen::VectorXd _b;  // Coriolis/centrifugal force vector
  Eigen::VectorXd
      _bg;  // Coriolis/centrifugal force vector + gravity force vector

  Eigen::MatrixXd _J_left_front_leg, _J_right_front_leg, _J_left_rear_leg,
      _J_right_rear_leg;  // jacobian matrix
  Eigen::Vector3d _x_left_front_leg, _x_right_front_leg, _x_left_rear_leg,
      _x_right_rear_leg;  // positon of foot
  Eigen::Vector3d _xdot_left_front_leg, _xdot_right_front_leg,
      _xdot_left_rear_leg,
      _xdot_right_rear_leg;  // positon of foot
  Eigen::MatrixXd _J_FL_foot, _J_FR_foot, _J_RR_foot, _J_RL_foot, _J_foot;

  double _mass;  // 13.71kg
  int _k;        // joint number

 private:
  void Initialize();
  void load_model();
  void set_robot_config();

  bool _bool_model_update, _bool_kinematics_update, _bool_dynamics_update,
      _bool_Jacobian_update;

  long _id_left_front_leg, _id_right_front_leg, _id_left_rear_leg,
      _id_right_rear_leg;

  Eigen::VectorXd _q, _qdot;
  Eigen::VectorXd _zero_vec_joint;

  Eigen::MatrixXd _Axd;
  Eigen::VectorXd _gxd, _bgxd;

  Eigen::Vector3d _position_local_task_left_front_leg;
  Eigen::Vector3d _position_local_task_right_front_leg;
  Eigen::Vector3d _position_local_task_left_rear_leg;
  Eigen::Vector3d _position_local_task_right_rear_leg;

  Eigen::MatrixXd _J_tmp;
};

#endif
