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

Locomotion::Locomotion(float _dt, int _iterations_between_mpc,
                                         KIST_UserParameters* parameters)
    : iterationsBetweenMPC(_iterations_between_mpc),
      horizonLength(10),
      dt(_dt),
      trotting(horizonLength, Vec4<int>(0, 5, 5, 0), Vec4<int>(5, 5, 5, 5),
               "Trotting"),
      bounding(horizonLength, Vec4<int>(5, 5, 0, 0), Vec4<int>(4, 4, 4, 4),
               "Bounding"),
      // bounding(horizonLength,
      // Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
      pronking(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(4, 4, 4, 4),
               "Pronking"),
      jumping(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(2, 2, 2, 2),
              "Jumping"),
      // galloping(horizonLength,
      // Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
      // galloping(horizonLength,
      // Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
      galloping(horizonLength, Vec4<int>(0, 2, 7, 9), Vec4<int>(4, 4, 4, 4),
                "Galloping"),
      standing(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(10, 10, 10, 10),
               "Standing"),
      // trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot
      // Running"),
      trotRunning(horizonLength, Vec4<int>(0, 5, 5, 0), Vec4<int>(4, 4, 4, 4),
                  "Trot Running"),
      walking(horizonLength, Vec4<int>(0, 3, 5, 8), Vec4<int>(5, 5, 5, 5),
              "Walking"),
      walking2(horizonLength, Vec4<int>(0, 5, 5, 0), Vec4<int>(7, 7, 7, 7),
               "Walking2"),
      pacing(horizonLength, Vec4<int>(5, 0, 5, 0), Vec4<int>(5, 5, 5, 5),
             "Pacing"),
      random(horizonLength, Vec4<int>(9, 13, 13, 9), 0.4,
             "Flying nine thirteenths trot"),
      random2(horizonLength, Vec4<int>(8, 16, 16, 8), 0.5, "Double Trot") {
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt,
         iterationsBetweenMPC, dtMPC);
//   setup_problem(dtMPC, horizonLength, 0.4, 120);
  // setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
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

void Locomotion::recompute_timing(int iterations_per_mpc) {
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void Locomotion::_SetupCommand(ControlFSMData<float>& data) {
  _body_height = 0.29;

  float x_vel_cmd, y_vel_cmd;
  float filter(0.1);
  // if(data.controlParameters->use_rc){
  //   const rc_control_settings* rc_cmd = data._desiredStateCommand->rcCommand;
  //   data.userParameters->cmpc_gait = rc_cmd->variable[0];
  //   _yaw_turn_rate = -rc_cmd->omega_des[2];
  //   x_vel_cmd = rc_cmd->v_des[0];
  //   y_vel_cmd = rc_cmd->v_des[1] * 0.5;
  //   _body_height += rc_cmd->height_variation * 0.08;
  // }else{
  // _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];
  // x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];
  // y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];
  _yaw_turn_rate = 0.0;  // TODO
  x_vel_cmd = 0.0;       // TODO
  y_vel_cmd = 0.0;       // TODO
  // }
  _x_vel_des = _x_vel_des * (1 - filter) + x_vel_cmd * filter;
  _y_vel_des = _y_vel_des * (1 - filter) + y_vel_cmd * filter;
  height_des << 0, 0, 0.24 - foot_clearance;
  _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  _roll_des = 0.;
  _pitch_des = 0.;
}

template <>
void Locomotion::run(ControlFSMData<float>& data) {
  bool omniMode = false;

  // Command Setup
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;
  if (gaitNumber >= 10) {
    gaitNumber -= 10;
    omniMode = true;
  }

  auto& seResult = data._stateEstimator->getResult();

  // Check if transition to standing
  if (((gaitNumber == 4) && current_gait != 4) || firstRun) {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait* gait = &trotting;
  if (gaitNumber == 1)
    gait = &bounding;
  else if (gaitNumber == 2)
    gait = &pronking;
  else if (gaitNumber == 3)
    gait = &random;
  else if (gaitNumber == 4)
    gait = &standing;
  else if (gaitNumber == 5)
    gait = &trotRunning;
  else if (gaitNumber == 6)
    gait = &random2;
  else if (gaitNumber == 7)
    gait = &random2;
  else if (gaitNumber == 8)
    gait = &pacing;
  current_gait = gaitNumber;

  gait->setIterations(iterationsBetweenMPC, iterationCounter);

  // jumping.setIterations(iterationsBetweenMPC, iterationCounter);

  // jumping.setIterations(27/2, iterationCounter);

  //  // printf("[%d] [%d]\n", jumping.get_current_gait_phase(),
  //  gait->get_current_gait_phase());
  //  // check jump trigger

  // jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
  // data._desiredStateCommand->trigger_pressed);

  //  // bool too_high = seResult.position[2] > 0.29;
  //  // check jump action
  // if(jump_state.should_jump(jumping.getCurrentGaitPhase())) {
  //   gait = &jumping;
  //   recompute_timing(27/2);
  //   _body_height = _body_height_jumping;
  //   currently_jumping = true;

  // } else {
  // recompute_timing(default_iterations_between_mpc);
  // }

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

  // get the foot end position (world coordinate system)
  // Body coordinates + Body rotation matrix^T*(side swing coordinates + Foot
  // base coordinates)
  for (int i = 0; i < 4; i++) {
    pFoot[i] = data._quadruped->getFootPositionInHipFrame(
                   **&data._quadruped, data._legController->datas[i].q, i) +
               data._quadruped->getHipLocation(i);
  }

//   if (gait != &standing) {
//     world_position_desired +=
//         dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
//   }

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
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);
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

    hip_offset = data._quadruped->getHipLocation(i);
    twisting_vector << -hip_offset(1), hip_offset(0), 0;
    hip_horizontal_velocity =
        seResult.vBody + seResult.rpy(2) * twisting_vector;

    //   v_des_robot = seResult.vBody;//+seResult.rpy(2);
    v_des_world =
        v_des_robot + seResult.rBody.transpose() * twisting_vector;

    footSwingTrajectories[i].setHeight(.06);
    Vec3<float> offset(0, side_sign[i] * .065, 0);

    float stance_time = gait->getCurrentStanceTime(dtMPC, i);

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
     foot_target_positions[2] = hip_horizontal_velocity[2] * .5 * stance_time -
                    .03f * (v_des_world[2] - hip_horizontal_velocity[2]) -
                    height_des(2) + 0.0;
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
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  // int* mpcTable = gait->getMpcTable();
  // updateMPCIfNeeded(mpcTable, data, omniMode);

  //  StateEstimator* se = hw_i->state_estimator;
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

  // custom
  for (int foot = 0; foot < 4; foot++) {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    // cout << foot << "  first foot : "  << "  " << contactStates[foot] <<
    // endl; cout  << foot << "  second foot : " << "  " <<
    // contactStateScheduled[foot] << endl;
    if (swingState > 0)  // foot is in swing
    {
      if (firstSwing[foot]) {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(
          swingState, swingTimes[foot]);

      //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd,
      //                                          0); // velocity dependent
      //                                          friction compensation todo
      //                                          removed
      // hw_i->leg_controller->leg_datas[foot].qd,
      // fsm->main_control_settings.variable[2]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
    //   Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();

      Vec3<float> qDes = getJointAngleFromFootPosition(
          pDesFootWorld - (data._quadruped->getHipLocation(foot)),
          data._quadruped->getSideSign(foot));
    //   if (foot == 0 || foot == 3) {
    //     cout << foot << " foot pDes : " << pDesFootWorld.transpose() << endl;
    //     cout << foot << " foot qDes : " << qDes.transpose() << endl;
    //     cout << "---------------------------------------------------" << endl;
    //   }
      data._legController->commands[foot].qDes = qDes;
      data._legController->commands[foot].kpJoint = Kp;
      data._legController->commands[foot].kdJoint = Kd;
      
      // }
    } else  // foot is in stance
    {
      firstSwing[foot] = true;

      // Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      // Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      // Vec3<float> pDesLeg =
      //     seResult.rBody * (pDesFootWorld - seResult.position) -
      //     data._quadruped->getHipLocation(foot);
      // Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // cout << "Foot " << foot
      //      << " relative velocity desired: " << vDesLeg.transpose() << "\n";
      Vec3<float> forceDesLeg(fOpt[foot * 3], fOpt[foot * 3 + 1],
                              fOpt[foot * 3 + 2]);
      data._legController->commands[foot].kpJoint = Kp_stance;
      data._legController->commands[foot].kdJoint = Kd_stance;
      // cout << foot  <<" foot force : " << forceDesLeg.transpose() << endl;
      // cout << "---------------------------------------------------"<< endl;
      // if (!data.userParameters->use_wbc) {
      // data._legController->commands[foot].pDes = pDesLeg;
      // data._legController->commands[foot].vDes = vDesLeg;
      // data._legController->commands[foot].kpCartesian = Kp_stance;
      // data._legController->commands[foot].kdCartesian = Kd_stance;

      // data._legController->commands[foot].forceFeedForward = f_ff[foot];
      // data._legController->commands[foot].kdJoint =
      //     Mat3<float>::Identity() * 0.2;

      //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd,
      //                                          0); todo removed
      // hw_i->leg_controller->leg_commands[foot].tau_ff +=
      // 0*footSwingController[foot]->getTauFF();
      // } else {  // Stance foot damping
      //   data._legController->commands[foot].pDes = pDesLeg;
      //   data._legController->commands[foot].vDes = vDesLeg;
      //   data._legController->commands[foot].kpCartesian = 0. * Kp_stance;
      //   data._legController->commands[foot].kdCartesian = Kd_stance;
      // }
      //            cout << "Foot " << foot << " force: " <<
      //            f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      // Fr_des[foot] = -f_ff[foot];
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);

  // contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update
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