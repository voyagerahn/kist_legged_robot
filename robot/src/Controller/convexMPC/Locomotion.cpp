#include "Locomotion.h"

#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>

#include <iostream>

#include "../../../../common/FootstepPlanner/GraphSearch.h"
#include "Gait.h"
#include "convexMPC_interface.h"

//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH

////////////////////
// Controller
////////////////////

Locomotion::Locomotion(float _dt, KIST_UserParameters* parameters) : dt(_dt) {
  _parameters = parameters;

  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for (int i = 0; i < 4; i++) firstSwing[i] = true;
  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
}

void Locomotion::initialize() {
  for (int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
}

void Locomotion::_SetupCommand(ControlFSMData<float>& data) {
  _body_height = 0.29;

  float x_vel_cmd, y_vel_cmd;
  float filter(0.1);
  
  _yaw_turn_rate = 0.0;  
  x_vel_cmd = 0.0;       
  y_vel_cmd = 0.0;       
  _x_vel_des = _x_vel_des * (1 - filter) + x_vel_cmd * filter;
  _y_vel_des = _y_vel_des * (1 - filter) + y_vel_cmd * filter;
  height_des << 0, 0, 0.26 - foot_clearance;
  _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  _roll_des = 0.;
  _pitch_des = 0.;
}

template <>
void Locomotion::run(ControlFSMData<float>& data) {
  // bool omniMode = false;
  
  // Command Setup
  _SetupCommand(data);
  // gaitNumber = data.userParameters->cmpc_gait;

  auto& seResult = data._stateEstimator->getResult();

  if (_body_height < 0.02) {
    _body_height = 0.29;
  }

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_robot = seResult.vWorld;

  // Integral-esque pitche and roll compensation
  if (fabs(v_robot[0]) > .2)  // avoid dividing by zero
  {
    rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
  }
  if (fabs(v_robot[1]) > 0.1) {
    rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);  //-0.25~0.25
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];  // compensation
  rpy_comp[0] =
      v_robot[1] * rpy_int[0] * (gaitNumber != 8);  // turn off for pronking

  // get the foot end position (world coordinate system) in base frame
  // Body coordinates + Body rotation matrix^T*(side swing coordinates + Foot
  // base coordinates)
  for (int i = 0; i < 4; i++) {
    pFoot[i] = data._quadruped->getFootPositionInHipFrame(
                   **&data._quadruped, data._legController->datas[i].q, i) +
               data._quadruped->getHipOffsets(i);
  }

  // some first time initialization
  if (firstRun) {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];

    for (int i = 0; i < 4; i++) {
      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);  // set p0
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);    // set pf
    }
    firstRun = false;
  }

  // foot placement
  for (int l = 0; l < 4; l++) {
    swingTimes[l] = data._gaitScheduler->gaitData.phaseSwing(l);
  }
  float side_sign[4] = {-1, 1, -1, 1};
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  float interleave_gain = -0.2; //-0.13
  float v_abs = std::fabs(v_des_robot[0]);   // float v_abs = std::fabs(seResult.vBody[0]);
  for (int i = 0; i < 4; i++) {
    if (firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    // get hip position in base frame
    hip_offset = data._quadruped->getHipLocation(i);
    twisting_vector << -hip_offset(1), hip_offset(0), 0;
    hip_horizontal_velocity =
        seResult.vBody + seResult.rpy(2) * twisting_vector;

    //   v_des_robot = seResult.vBody;//+seResult.rpy(2);
    
    v_des_world =
        v_des_robot;// + seResult.rBody.transpose() * twisting_vector;

    footSwingTrajectories[i].setHeight(.06);
    Vec3<float> offset(0, side_sign[i] * .065, 0);

    float stance_time = 0.3;//data._gaitScheduler->gaitData.timeStance(i); //gait->getCurrentStanceTime(dtMPC, i);

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;
    foot_target_positions[0] = hip_horizontal_velocity[0] * (.5) * stance_time -
                    .03f * (v_des_world[0] - hip_horizontal_velocity[0]) -
                    height_des(0) + hip_offset(0);
    foot_target_positions[1] = hip_horizontal_velocity[1] * .5 * stance_time -
                    .03f * (v_des_world[1] - hip_horizontal_velocity[1]) -
                    height_des(1) + hip_offset(1);
    foot_target_positions[2] = -height_des(2);
    // cout << i << " foot : target_positions" << foot_target_positions.transpose() << endl;
    footSwingTrajectories[i].setFinalPosition(foot_target_positions);
  }

  // calc gait
  iterationCounter++;

//   // load LCM leg swing gains
  Kp << 25, 0, 0, 0, 25, 0, 0, 0, 25;
  Kp_stance = 0 * Kp;

  Kd << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5;
  Kd_stance = 0* Kd;
  // gait
  Vec4<float> contactStates = data._gaitScheduler->gaitData.phaseStance; //gait->getContactState();
  Vec4<float> swingStates = data._gaitScheduler->gaitData.phaseSwing;//gait->getSwingState();
  Vec4<float> se_contactState(0, 0, 0, 0);

  // custom balance
  for (int i = 0; i < 4; i++) {
    contactStateScheduled[i] = contactStates[i] > 0 ? 1 : 0;
  }

  for (int leg = 0; leg < 4; leg++) {
    minForces[leg] = contactStateScheduled[leg] * minForce;
    maxForces[leg] = contactStateScheduled[leg] * maxForce;
  }
  for (int i = 0; i < 4; i++) {
    se_xfb[i] = seResult.orientation(i);
  }
  // se_xfb[3] = 1.0;
  for (int i = 0; i < 3; i++) {
    rpy[i] = 0;  //(double)_data->_stateEstimator->getResult().rpy(i);
    p_des[i] = seResult.position(i);
    p_act[i] = seResult.position(i);
    omegaDes[i] =
        0;  //(double)_data->_stateEstimator->getResult().omegaBody(i);
    v_act[i] = seResult.vBody(i);
    v_des[i] = seResult.vBody(i);

    se_xfb[4 + i] = seResult.position(i);
    se_xfb[7 + i] = seResult.omegaBody(i);
    se_xfb[10 + i] = seResult.vBody(i);

    // Set the translational and orientation gains
    kpCOM[i] = data.controlParameters->kpCOM(i);
    kdCOM[i] = data.controlParameters->kdCOM(i);
    kpBase[i] = data.controlParameters->kpBase(i);
    kdBase[i] = data.controlParameters->kdBase(i);
  }

  // Get the foot locations relative to COM
  for (int leg = 0; leg < 4; leg++) {
    computeLegJacobianAndPosition(**&data._quadruped,
                                  data._legController->datas[leg].q,
                                  (Mat3<float>*)nullptr, &pFeetVec, 1);
    pFeetVecCOM =
        seResult.rBody.transpose() * (data._quadruped->getHipLocation(leg) +
                                      data._legController->datas[leg].p);

    pFeet[leg * 3] = (double)pFeetVecCOM[0];
    pFeet[leg * 3 + 1] = (double)pFeetVecCOM[1];
    pFeet[leg * 3 + 2] = (double)pFeetVecCOM[2];
  }
  balanceController.set_alpha_control(0.01);
  balanceController.set_friction(0.5);
  balanceController.set_mass(12.0);
  balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
  balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
  balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
  balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
  balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, 0.0);
  balanceController.solveQP_nonThreaded(fOpt);
  // cout << "Stance phase : " <<data._gaitScheduler->gaitData.phaseStance.transpose() << endl;
  // cout << "Stance phase : " <<data._gaitScheduler->gaitData.phaseStance.transpose() << endl;
  // cout << "original Swing phase : " <<data._gaitScheduler->gaitData.phaseSwing.transpose() << endl;
  // cout << "-----------------------------------------------------------" << endl;

  for (int foot = 0; foot < 4; foot++) {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
   
    if (swingState > 0)  // foot is in swing
    {
      if (firstSwing[foot]) {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(
          swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
    //   Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();

      Vec3<float> qDes = getJointAngleFromFootPosition(
          pDesFootWorld - (data._quadruped->getHipOffsets(foot)),
          data._quadruped->getSideSign(foot));
      if (foot == 2) {
        cout << foot << " foot pDes : " << pDesFootWorld.transpose() << endl;
        cout << foot << " foot qDes : " << qDes.transpose() << endl;
      }
      Vec3<float> zero(0.0, 0.0, 0.0);
      data._legController->commands[foot].qDes = qDes;
      data._legController->commands[foot].kpJoint = Kp;
      data._legController->commands[foot].kdJoint = Kd;
      data._legController->commands[foot].forceFeedForward = zero;

      // }
    } else  // foot is in stance
    {
      firstSwing[foot] = true;

      // Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      // Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      // cout << "Foot " << foot
      //      << " relative velocity desired: " << vDesLeg.transpose() << "\n";
      Vec3<float> forceDesLeg(fOpt[foot * 3], fOpt[foot * 3 + 1],
                              fOpt[foot * 3 + 2]);
      if (foot == 2) {
        cout << foot << " foot force : " << forceDesLeg.transpose() << endl;
      }
      data._legController->commands[foot].kpJoint = Kp_stance;
      data._legController->commands[foot].kdJoint = Kd_stance;
      data._legController->commands[foot].forceFeedForward = forceDesLeg;
      // cout << foot  <<" foot force : " << forceDesLeg.transpose() << endl;

      // data._legController->commands[foot].kpCartesian = Kp_stance;
      // data._legController->commands[foot].kdCartesian = Kd_stance;

      se_contactState[foot] = contactState;
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);


}


template <>
void Locomotion::run(ControlFSMData<double>& data) {
  (void)data;
  printf("call to old CMPC with double!\n");
}

//get joint angle from foot position in hip frame
Vec3<float> Locomotion::getJointAngleFromFootPosition(
    Vec3<float> pDes, int sideSign) {
  // position of foot relative to hip
  const auto x = pDes(0);
  const auto y = pDes(1);
  const auto z = pDes(2);

  Vec3<float> q;
  const auto l2 = 0.2;                 // hip
  const auto l3 = 0.2;                 // knee
  const auto l1 = 0.08505 * sideSign;  // abad
  q.setZero();
  q(2) = -std::acos((x * x + y * y + z * z - l1 * l1 - l2 * l2 - l3 * l3) / (2.0 * l2 * l3));
  auto l = std::sqrt(l2 * l2 + l3 * l3 + 2 * l2 * l3 * std::cos(q(2)));

  q(1) = std::asin(-x / l) - q(2) / 2;

  auto c1 = l1 * y - l * std::cos(q(1) + q(2) / 2) * z;
  auto s1 = l * std::cos(q(1) + q(2) / 2) * y + l1 * z;

  q(0) = std::atan2(s1, c1);

  return q;
}