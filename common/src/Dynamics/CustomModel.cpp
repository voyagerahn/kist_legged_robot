#include "Dynamics/CustomModel.h"

#define JDOF 18

CModel::CModel()
{
    Initialize();
}
void CModel::Initialize()
{
    _bool_model_update = false;
    _bool_kinematics_update = false;
    _bool_dynamics_update = false;
    _bool_Jacobian_update = false;

    _k = JDOF;
    _q.setZero(_k+1);
    _qdot.setZero(_k);
    _zero_vec_joint.setZero(_k);

    _id_left_front_leg = 2147483648; //FL
    _id_right_front_leg = 2147483650; //Fr
    _id_left_rear_leg = 2147483652; //RL
    _id_right_rear_leg = 2147483654; //RR

    _A.setZero(_k, _k);
    _g.setZero(_k);
    _b.setZero(_k);
    _bg.setZero(_k);

    _J_tmp.setZero(6, _k);
    _J_left_front_leg.setZero(3, _k);
    _J_right_front_leg.setZero(3, _k);
    _J_left_rear_leg.setZero(3, _k);
    _J_right_rear_leg.setZero(3, _k);
    _x_left_front_leg.setZero();
    _x_left_rear_leg.setZero();
    _x_right_front_leg.setZero();
    _x_right_rear_leg.setZero();
    _xdot_left_front_leg.setZero();
    _xdot_right_front_leg.setZero();
    _xdot_left_rear_leg.setZero();
    _xdot_right_rear_leg.setZero();
    _J_FL_foot.setZero(3,3);
    _J_FR_foot.setZero(3,3);
    _J_RR_foot.setZero(3,3);
    _J_RL_foot.setZero(3,3);
    _J_foot.setZero(12,12);

    _Axd.setZero(_k, _k);
    _gxd.setZero(_k);
    _bgxd.setZero(_k);

    _position_local_task_left_front_leg.setZero();
    _position_local_task_right_front_leg.setZero();
    _position_local_task_left_rear_leg.setZero();
    _position_local_task_right_rear_leg.setZero();

    load_model();
}

//Read urdf model
void CModel::load_model()
{
    //TODO: change realive path not absolute path
    RigidBodyDynamics::Addons::URDFReadFromFile("../../config/a1.urdf", &_model, true, true); 
    cout << endl
         << endl
         << "Model Loaded for RBDL." << endl
         << "Total DoFs: " << _model.dof_count << endl
         << endl;

    if (_model.dof_count != _k)
    {
        cout << "Simulation model and RBDL model mismatch!!!" << endl
             << endl;
    }
    _bool_model_update = true; //check model update

    cout << "Model Loading Complete." << endl
         << endl;
}
void CModel::update_kinematics(Eigen::VectorXd &q, Eigen::VectorXd &qdot)
{
    for (int i = 0; i < _k; i++)
    {
        _q(i) = q(i);
        _qdot(i) = qdot(i);
    }

        _q(_k) = q(_k);

    if (_bool_model_update == true)
    {
        RigidBodyDynamics::UpdateKinematicsCustom(_model, &_q, &_qdot, NULL); //update kinematics
    }
    else
    {
        cout << "Robot model is not ready. Please load model first." << endl
             << endl;
    }
    _bool_kinematics_update = true; //check kinematics update
}
void CModel::update_dynamics()
{
    if (_bool_kinematics_update == true)
    {
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(_model, _q, _Axd, false);                      //update dynamics
        RigidBodyDynamics::InverseDynamics(_model, _q, _zero_vec_joint, _zero_vec_joint, _gxd, NULL); //get _g
        RigidBodyDynamics::InverseDynamics(_model, _q, _qdot, _zero_vec_joint, _bgxd, NULL);          //get _g+_b
        _A = _Axd;
        _g = _gxd;
        _bg = _bgxd;
        _b = _bg - _g; //get _b
    }
    else
    {
        cout << "Robot kinematics is not ready. Please update kinematics first." << endl
             << endl;
    }
    // cout << _A << endl;
    // cout << "-------------------------------------------------------------------" << endl;

    _bool_dynamics_update = true; //check kinematics update
}
void CModel::calculate_EE_Jacobians()
{
    if (_bool_kinematics_update == true)
    {
        _J_left_front_leg.setZero();
        _J_tmp.setZero();
        RigidBodyDynamics::CalcPointJacobian6D(_model, _q, _id_left_front_leg, _position_local_task_left_front_leg, _J_tmp, false); //left front leg
        _J_left_front_leg = _J_tmp.block<3, JDOF>(3, 0);
  
        _J_right_front_leg.setZero();
        _J_tmp.setZero();
        RigidBodyDynamics::CalcPointJacobian6D(_model, _q, _id_right_front_leg, _position_local_task_right_front_leg, _J_tmp, false); //right front hand
        _J_right_front_leg = _J_tmp.block<3, JDOF>(3, 0);

        _J_left_rear_leg.setZero();
        _J_tmp.setZero();
        RigidBodyDynamics::CalcPointJacobian6D(_model, _q, _id_left_rear_leg, _position_local_task_left_rear_leg, _J_tmp, false); //left rear hand
        _J_left_rear_leg = _J_tmp.block<3, JDOF>(3, 0);

        _J_right_rear_leg.setZero();
        _J_tmp.setZero();
        RigidBodyDynamics::CalcPointJacobian6D(_model, _q, _id_right_rear_leg, _position_local_task_right_rear_leg, _J_tmp, false); //right rear hand
        _J_right_rear_leg = _J_tmp.block<3, JDOF>(3, 0);
        
        _J_tmp.setZero();

        _bool_Jacobian_update = true;
    }
    else
    {
        cout << "Robot kinematics is not ready. Please update kinematics first." << endl
             << endl;
    }
}
void CModel::calculate_EE_positions_orientations()
{
    if (_bool_kinematics_update == true)
    {
        _x_left_front_leg.setZero();
        _x_left_front_leg = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q, _id_left_front_leg, _position_local_task_left_front_leg, false);

        _x_right_front_leg.setZero();
        _x_right_front_leg = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q, _id_right_front_leg, _position_local_task_right_front_leg, false);

        _x_left_rear_leg.setZero();
        _x_left_rear_leg = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q, _id_left_rear_leg, _position_local_task_left_rear_leg, false);

        _x_right_rear_leg.setZero();
        _x_right_rear_leg = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q, _id_right_rear_leg, _position_local_task_right_rear_leg, false);
    }
    else
    {
        cout << "Robot kinematics is not ready. Please update kinematics first." << endl
             << endl;
    }
}
void CModel::calculate_COM(Eigen::VectorXd &q, Eigen::VectorXd &qdot)
{
    for (int i = 0; i < _k; i++)
    {
        _q(i) = q(i);
        _qdot(i) = qdot(i);
    }

        _q(_k) = q(_k);

    if (_bool_dynamics_update == true)
    {
        _COM.setZero();
        _COM_vel.setZero();
        RigidBodyDynamics::Utils::CalcCenterOfMass(_model, _q, _qdot, NULL, _mass, _COM, &_COM_vel);
    }
    else
    {
        cout << "Robot dynamics are not ready. Please update dynamics first." << endl
             << endl;
    }
}
void CModel::calculate_EE_velocity()
{
    if (_bool_Jacobian_update == true)
    {
        _xdot_left_front_leg = _J_left_front_leg * _qdot;
        _xdot_right_front_leg = _J_right_front_leg * _qdot;
        _xdot_left_rear_leg = _J_left_rear_leg * _qdot;
        _xdot_right_rear_leg = _J_right_rear_leg * _qdot;
    }
    else
    {
        cout << "Jacobian matrices are not ready. Please calculate Jacobians first." << endl
             << endl;
    }
}
void CModel::calculate_foot_Jacobians()
{
    if (_bool_Jacobian_update == true)
    {
        _J_FR_foot << _J_right_front_leg(0, 9), _J_right_front_leg(0, 10), _J_right_front_leg(0, 11),
            _J_right_front_leg(1, 9), _J_right_front_leg(1, 10), _J_right_front_leg(1, 11),
            _J_right_front_leg(2, 9), _J_right_front_leg(2, 10), _J_right_front_leg(2, 11);
        
        _J_FL_foot << _J_left_front_leg(0, 6), _J_left_front_leg(0, 7), _J_left_front_leg(0, 8),
            _J_left_front_leg(1, 6), _J_left_front_leg(1, 7), _J_left_front_leg(1, 8),
            _J_left_front_leg(2, 6), _J_left_front_leg(2, 7), _J_left_front_leg(2, 8);

        _J_RR_foot << _J_right_rear_leg(0, 15), _J_right_rear_leg(0, 16), _J_right_rear_leg(0, 17),
            _J_right_rear_leg(1, 15), _J_right_rear_leg(1, 16), _J_right_rear_leg(1, 17),
            _J_right_rear_leg(2, 15), _J_right_rear_leg(2, 16), _J_right_rear_leg(2, 17);

        _J_RL_foot << _J_left_rear_leg(0, 12), _J_left_rear_leg(0, 13), _J_left_rear_leg(0, 14),
            _J_left_rear_leg(1, 12), _J_left_rear_leg(1, 13), _J_left_rear_leg(1, 14),
            _J_left_rear_leg(2, 12), _J_left_rear_leg(2, 13), _J_left_rear_leg(2, 14);

        // _J_foot(0, 0) = _J_FR_foot(0, 0);
        // _J_foot(0, 1) = _J_FR_foot(0, 1);
        // _J_foot(0, 2) = _J_FR_foot(0, 2);
        // _J_foot(1, 0) = _J_FR_foot(1, 0);
        // _J_foot(1, 1) = _J_FR_foot(1, 1);
        // _J_foot(1, 2) = _J_FR_foot(1, 2);
        // _J_foot(2, 0) = _J_FR_foot(2, 0);
        // _J_foot(2, 1) = _J_FR_foot(2, 1);
        // _J_foot(2, 2) = _J_FR_foot(2, 2);

        // _J_foot(3, 3) = _J_FL_foot(0, 0);
        // _J_foot(3, 4) = _J_FL_foot(0, 1);
        // _J_foot(3, 5) = _J_FL_foot(0, 2);
        // _J_foot(4, 3) = _J_FL_foot(1, 0);
        // _J_foot(4, 4) = _J_FL_foot(1, 1);
        // _J_foot(4, 5) = _J_FL_foot(1, 2);
        // _J_foot(5, 3) = _J_FL_foot(2, 0);
        // _J_foot(5, 4) = _J_FL_foot(2, 1);
        // _J_foot(5, 5) = _J_FL_foot(2, 2);

        // _J_foot(6, 6) = _J_RR_foot(0, 0);
        // _J_foot(6, 7) = _J_RR_foot(0, 1);
        // _J_foot(6, 8) = _J_RR_foot(0, 2);
        // _J_foot(7, 6) = _J_RR_foot(1, 0);
        // _J_foot(7, 7) = _J_RR_foot(1, 1);
        // _J_foot(7, 8) = _J_RR_foot(1, 2);
        // _J_foot(8, 6) = _J_RR_foot(2, 0);
        // _J_foot(8, 7) = _J_RR_foot(2, 1);
        // _J_foot(8, 8) = _J_RR_foot(2, 2);

        // _J_foot(9, 9) = _J_RL_foot(0, 0);
        // _J_foot(9, 10) = _J_RL_foot(0, 1);
        // _J_foot(9, 11) = _J_RL_foot(0, 2);
        // _J_foot(10, 9) = _J_RL_foot(1, 0);
        // _J_foot(10, 10) = _J_RL_foot(1, 1);
        // _J_foot(10, 11) = _J_RL_foot(1, 2);
        // _J_foot(11, 9) = _J_RL_foot(2, 0);
        // _J_foot(11, 10) = _J_RL_foot(2, 1);
        // _J_foot(11, 11) = _J_RL_foot(2, 2);

        // cout << _J_foot << endl;
        // cout <<"-------------------------------------------"<<endl;
    }
    else
    {
        cout << "Jacobian matrices are not ready. Please calculate Jacobians first." << endl
             << endl;
    }
}