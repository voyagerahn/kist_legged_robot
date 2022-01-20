/*! @file LegController.cpp
 *  @brief Common Leg Control Interface
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#include "Controllers/LegController.h"

/*!
 * Zero the leg command so the leg will not output torque
 */
template <typename T>
void LegControllerCommand<T>::zero() {
  tauFeedForward = Vec3<T>::Zero();
  forceFeedForward = Vec3<T>::Zero();
  qDes = Vec3<T>::Zero();
  qdDes = Vec3<T>::Zero();
  pDes = Vec3<T>::Zero();
  vDes = Vec3<T>::Zero();
  kpCartesian = Mat3<T>::Zero();
  kdCartesian = Mat3<T>::Zero();
  kpJoint = Mat3<T>::Zero();
  kdJoint = Mat3<T>::Zero();
}

/*!
 * Zero the leg data
 */
template <typename T>
void LegControllerData<T>::zero() {
  q = Vec3<T>::Zero();
  qd = Vec3<T>::Zero();
  p = Vec3<T>::Zero();
  v = Vec3<T>::Zero();
  J = Mat3<T>::Zero();
  tauEstimate = Vec3<T>::Zero();
}

/*!
 * Zero all leg commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
template <typename T>
void LegController<T>::zeroCommand() {
  for (auto& cmd : commands) {
    cmd.zero();
  }
  _legsEnabled = false;
}

/*!
 * Update the "leg data" from a SPIne board message
 */
template <typename T>
void LegController<T>::updateData(const LowState* state) {
  for (int leg = 0; leg < 4; leg++) {
    // q:
    datas[0].q(0) = state->motorState[FR_0].q;
    datas[0].q(1) = state->motorState[FR_1].q;
    datas[0].q(2) = state->motorState[FR_2].q;

    datas[1].q(0) = state->motorState[FL_0].q;
    datas[1].q(1) = state->motorState[FL_1].q;
    datas[1].q(2) = state->motorState[FL_2].q;

    datas[2].q(0) = state->motorState[RR_0].q;
    datas[2].q(1) = state->motorState[RR_1].q;
    datas[2].q(2) = state->motorState[RR_2].q;

    datas[3].q(0) = state->motorState[RL_0].q;
    datas[3].q(1) = state->motorState[RL_1].q;
    datas[3].q(2) = state->motorState[RL_2].q;
    // qd
    datas[0].qd(0) = state->motorState[FL_0].dq;
    datas[0].qd(1) = state->motorState[FL_1].dq;
    datas[0].qd(2) = state->motorState[FL_2].dq;

    datas[1].qd(0) = state->motorState[FR_0].dq;
    datas[1].qd(1) = state->motorState[FR_1].dq;
    datas[1].qd(2) = state->motorState[FR_2].dq;

    datas[2].qd(0) = state->motorState[RR_0].dq;
    datas[2].qd(1) = state->motorState[RR_1].dq;
    datas[2].qd(2) = state->motorState[RR_2].dq;

    datas[3].qd(0) = state->motorState[RL_0].dq;
    datas[3].qd(1) = state->motorState[RL_1].dq;
    datas[3].qd(2) = state->motorState[RL_2].dq;

    // cout << "----------------------------" << endl;
    // cout << "FR[0] : " << datas[0].q(0) << endl;
    // cout << "FR[1] : " << datas[0].q(1) << endl;
    // cout << "FR[2] : " << datas[0].q(2) << endl;
    // cout << "----------------------------" << endl;

    // cout << "FL[0] : " << datas[1].q(0) << endl;
    // cout << "FL[1] : " << datas[1].q(1) << endl;
    // cout << "FL[2] : " << datas[1].q(2) << endl;
    // //tau
    // datas[0].tauEstimate(FL_0) = state->motorState[FL_0].tauEst;
    // datas[0].tauEstimate(FL_1) = state->motorState[FL_1].tauEst;
    // datas[0].tauEstimate(FL_2) = state->motorState[FL_2].tauEst;

    // datas[1].tauEstimate(FR_0) = state->motorState[FR_0].tauEst;
    // datas[1].tauEstimate(FR_1) = state->motorState[FR_1].tauEst;
    // datas[1].tauEstimate(FR_2) = state->motorState[FR_2].tauEst;

    // datas[2].tauEstimate(RL_0) = state->motorState[RL_0].tauEst;
    // datas[2].tauEstimate(RL_1) = state->motorState[RL_1].tauEst;
    // datas[2].tauEstimate(RL_2) = state->motorState[RL_2].tauEst;

    // datas[3].tauEstimate(RR_0) = state->motorState[RR_0].tauEst;
    // datas[3].tauEstimate(RR_1) = state->motorState[RR_1].tauEst;
    // datas[3].tauEstimate(RR_2) = state->motorState[RR_2].tauEst;

    // // J and p
    // computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q,
    // &(datas[leg].J),
    //                                  &(datas[leg].p), leg);

    // // v
    // datas[leg].v = datas[leg].J * datas[leg].qd;
  }
}

/*!
 * Update the "leg command" for the SPIne board message
 */
template <typename T>
void LegController<T>::updateCommand(LowCmd* cmd) {
  // tauFF
  Vec12<T> legTorque;
  legTorque.setZero();
  // legTorque(FL_0) = commands[0].tauFeedForward(FL_0);
  // legTorque(FL_1) = commands[0].tauFeedForward(FL_1);
  // legTorque(FL_2) = commands[0].tauFeedForward(FL_2);

  // legTorque(FR_0) = commands[1].tauFeedForward(FR_0);
  // legTorque(FR_1) = commands[1].tauFeedForward(FR_1);
  // legTorque(FR_2) = commands[1].tauFeedForward(FR_2);

  // legTorque(RL_0) = commands[2].tauFeedForward(RL_0);
  // legTorque(RL_1) = commands[2].tauFeedForward(RL_1);
  // legTorque(RL_2) = commands[2].tauFeedForward(RL_2);

  // legTorque(RR_0) = commands[3].tauFeedForward(RR_0);
  // legTorque(RR_1) = commands[3].tauFeedForward(RR_1);
  // legTorque(RR_2) = commands[3].tauFeedForward(RR_2);

  // forceFF
  // Vec12<T> footForce;
  // footForce.setZero();
  // footForce(FL_0) = commands[0].forceFeedForward(FL_0);
  // footForce(FL_1) = commands[0].forceFeedForward(FL_1);
  // footForce(FL_2) = commands[0].forceFeedForward(FL_2);
  // footForce(FR_0) = commands[1].forceFeedForward(FR_0);
  // footForce(FR_1) = commands[1].forceFeedForward(FR_1);
  // footForce(FR_2) = commands[1].forceFeedForward(FR_2);
  // footForce(RL_0) = commands[2].forceFeedForward(RL_0);
  // footForce(RL_1) = commands[2].forceFeedForward(RL_1);
  // footForce(RL_2) = commands[2].forceFeedForward(RL_2);
  // footForce(RR_0) = commands[3].forceFeedForward(RR_0);
  // footForce(RR_1) = commands[3].forceFeedForward(RR_1);
  // footForce(RR_2) = commands[3].forceFeedForward(RR_2);

  // // cartesian PD

  // Torque
  // legTorque(FL_0) += datas[FL_0].J.transpose() * footForce(FL_0);
  // legTorque(FL_1) += datas[FL_1].J.transpose() * footForce(FL_1);
  // legTorque(FL_2) += datas[FL_2].J.transpose() * footForce(FL_2);
  // legTorque(FR_0) += datas[FR_0].J.transpose() * footForce(FR_0);
  // legTorque(FR_1) += datas[FR_1].J.transpose() * footForce(FR_1);
  // legTorque(FR_2) += datas[FR_2].J.transpose() * footForce(FR_2);
  // legTorque(RL_0) += datas[RL_0].J.transpose() * footForce(RL_0);
  // legTorque(RL_1) += datas[RL_1].J.transpose() * footForce(RL_1);
  // legTorque(RL_2) += datas[RL_2].J.transpose() * footForce(RL_2);
  // legTorque(RR_0) += datas[RR_0].J.transpose() * footForce(RR_0);
  // legTorque(RR_1) += datas[RR_1].J.transpose() * footForce(RR_1);
  // legTorque(RR_2) += datas[RR_2].J.transpose() * footForce(RR_2);

  legTorque(0) +=
      commands[0].kpJoint(0, 0) * (commands[0].qDes(0) - datas[0].q(0)) +
      commands[0].kdJoint(0, 0) * (commands[0].qdDes(0) - datas[0].qd(0));

  legTorque(1) +=
      commands[0].kpJoint(1, 1) * (commands[0].qDes(1) - datas[0].q(1)) +
      commands[0].kdJoint(1, 1) * (commands[0].qdDes(1) - datas[0].qd(1));

  legTorque(2) +=
      commands[0].kpJoint(2, 2) * (commands[0].qDes(2) - datas[0].q(2)) +
      commands[0].kdJoint(2, 2) * (commands[0].qdDes(2) - datas[0].qd(2));

  // legTorque(3) +=
  //     commands[1].kpJoint(0, 0) * (commands[1].qDes(0) - datas[1].q(0)) +
  //     commands[1].kdJoint(0, 0) * (commands[1].qdDes(0) - datas[1].qd(0));

  // legTorque(4) +=
  //     commands[1].kpJoint(1, 1) * (commands[1].qDes(1) - datas[1].q(1)) +
  //     commands[1].kdJoint(1, 1) * (commands[1].qdDes(1) - datas[1].qd(1));

  // legTorque(5) +=
  //     commands[1].kpJoint(2, 2) * (commands[1].qDes(2) - datas[1].q(2)) +
  //     commands[1].kdJoint(2, 2) * (commands[1].qdDes(2) - datas[1].qd(2));

  // legTorque(6) +=
  //     commands[2].kpJoint(0, 0) * (commands[2].qDes(0) - datas[2].q(0)) +
  //     commands[2].kdJoint(0, 0) * (commands[2].qdDes(0) - datas[2].qd(0));

  // legTorque(7) +=
  //     commands[2].kpJoint(1, 1) * (commands[2].qDes(1) - datas[2].q(1)) +
  //     commands[2].kdJoint(1, 1) * (commands[2].qdDes(1) - datas[2].qd(1));

  // legTorque(8) +=
  //     commands[2].kpJoint(2, 2) * (commands[2].qDes(2) - datas[2].q(2)) +
  //     commands[2].kdJoint(2, 2) * (commands[2].qdDes(2) - datas[2].qd(2));

  // legTorque(9) +=
  //     commands[3].kpJoint(0, 0) * (commands[3].qDes(0) - datas[3].q(0)) +
  //     commands[3].kdJoint(0, 0) * (commands[3].qdDes(0) - datas[3].qd(0));

  // legTorque(10) +=
  //     commands[3].kpJoint(1, 1) * (commands[3].qDes(1) - datas[3].q(1)) +
  //     commands[3].kdJoint(1, 1) * (commands[3].qdDes(1) - datas[3].qd(1));

  // legTorque(11) +=
  //     commands[3].kpJoint(2, 2) * (commands[3].qDes(2) - datas[3].q(2)) +
  //     commands[3].kdJoint(2, 2) * (commands[3].qdDes(2) - datas[3].qd(2));
  // cout << "legTorque(0)" << legTorque(0) << endl;
  // cout << "legTorque(1)" << legTorque(1) << endl;
  // cout << "legTorque(2)" << legTorque(2) << endl;

  // set command:
  // cmd->motorCmd[0].tau = legTorque(0);
  // cmd->motorCmd[1].tau = legTorque(1);
  // cmd->motorCmd[2].tau = legTorque(2);

  cmd->motorCmd[0].tau = 0.0;
  cmd->motorCmd[1].tau = 0.0;
  cmd->motorCmd[2].tau = 0.0;


  // cmd->motorCmd[3].tau = legTorque(3);
  // cmd->motorCmd[4].tau = legTorque(4);
  // cmd->motorCmd[5].tau = legTorque(5);

  // cmd->motorCmd[6].tau = legTorque(6);
  // cmd->motorCmd[7].tau = legTorque(7);
  // cmd->motorCmd[8].tau = legTorque(8);

  // cmd->motorCmd[9].tau = legTorque(9);
  // cmd->motorCmd[10].tau = legTorque(10);
  // cmd->motorCmd[11].tau = legTorque(11);

  // cout << "----------------------------" << endl;
  // cout << "TorqueFR[0] : " << legTorque(0) << endl;
  // cout << "TorqueFR[1] : " << legTorque(1) << endl;
  // cout << "TorqueFR[2] : " << legTorque(2) << endl;
  // cmd->motorCmd[FR_0].tau = legTorque(3);
  // cmd->motorCmd[FR_1].tau = legTorque(4);
  // cmd->motorCmd[FR_2].tau = legTorque(5);

  // cmd->motorCmd[RL_0].tau = legTorque(6);
  // cmd->motorCmd[RL_1].tau = legTorque(7);
  // cmd->motorCmd[RL_2].tau = legTorque(8);

  // cmd->motorCmd[RR_0].tau = legTorque(9);
  // cmd->motorCmd[RR_1].tau = legTorque(10);
  // cmd->motorCmd[RR_2].tau = legTorque(11);

  // joint space PD
  // cmd->kd_abad[leg] = commands[leg].kdJoint(0, 0);
  // cmd->kd_hip[leg] = commands[leg].kdJoint(1, 1);
  // cmd->kd_knee[leg] = commands[leg].kdJoint(2, 2);

  // cmd->kp_abad[leg] = commands[leg].kpJoint(0, 0);
  // cmd->kp_hip[leg] = commands[leg].kpJoint(1, 1);
  // cmd->kp_knee[leg] = commands[leg].kpJoint(2, 2);

  // cmd->q_des_abad[leg] = commands[leg].qDes(0);
  // cmd->q_des_hip[leg] = commands[leg].qDes(1);
  // cmd->q_des_knee[leg] = commands[leg].qDes(2);

  // cmd->qd_des_abad[leg] = commands[leg].qdDes(0);
  // cmd->qd_des_hip[leg] = commands[leg].qdDes(1);
  // cmd->qd_des_knee[leg] = commands[leg].qdDes(2);

  // // estimate torque
  // datas[leg].tauEstimate =
  //     legTorque + commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q)
  //     + commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);

  // spiCommand->flags[leg] = _legsEnabled ? 1 : 0;
}

// constexpr float CHEETAH_3_ZERO_OFFSET[4][3] = {{1.f, 4.f, 7.f},
//                                                {2.f, 5.f, 8.f},
//                                                {3.f, 6.f, 9.f}};

template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;

template struct LegControllerData<double>;
template struct LegControllerData<float>;

template class LegController<double>;
template class LegController<float>;

/*!
 * Compute the position of the foot and its Jacobian.  This is done in the local
 * leg coordinate system. If J/p are NULL, the calculation will be skipped.
 */
template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg) {
  T l1 = quad._abadLinkLength;
  T l2 = quad._hipLinkLength;
  T l3 = quad._kneeLinkLength;
  T l4 = quad._kneeLinkY_offset;
  T sideSign = quad.getSideSign(leg);

  T s1 = std::sin(q(0));
  T s2 = std::sin(q(1));
  T s3 = std::sin(q(2));

  T c1 = std::cos(q(0));
  T c2 = std::cos(q(1));
  T c3 = std::cos(q(2));

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;

  if (J) {
    J->operator()(0, 0) = 0;
    J->operator()(0, 1) = l3 * c23 + l2 * c2;
    J->operator()(0, 2) = l3 * c23;
    J->operator()(1, 0) =
        l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) =
        l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1;
    J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) = l3 * c1 * s23;
  }

  if (p) {
    p->operator()(0) = l3 * s23 + l2 * s2;
    p->operator()(1) =
        (l1 + l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) =
        (l1 + l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

template void computeLegJacobianAndPosition<double>(Quadruped<double>& quad,
                                                    Vec3<double>& q,
                                                    Mat3<double>* J,
                                                    Vec3<double>* p, int leg);
template void computeLegJacobianAndPosition<float>(Quadruped<float>& quad,
                                                   Vec3<float>& q,
                                                   Mat3<float>* J,
                                                   Vec3<float>* p, int leg);

// /*! @file LegController.cpp
//  *  @brief Common Leg Control Interface
//  *
//  *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
//  *  Abstracts away the difference between the SPIne and the TI Boards
//  *  All quantities are in the "leg frame" which has the same orientation as
//  the
//  * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
//  * frame").
//  */

// #include "Controllers/LegController.h"

// /*!
//  * Zero the leg command so the leg will not output torque
//  */
// template <typename T>
// void LegControllerCommand<T>::zero() {
//   tauFeedForward = Vector3d::Zero();
//   forceFeedForward = Vector3d::Zero();
//   qDes = Vector3d::Zero();
//   qdDes = Vector3d::Zero();
//   pDes = Vector3d::Zero();
//   vDes = Vector3d::Zero();
//   kpCartesian = Matrix3d::Zero();
//   kdCartesian = Matrix3d::Zero();
//   kpJoint = Matrix3d::Zero();
//   kdJoint = Matrix3d::Zero();
// }

// /*!
//  * Zero the leg data
//  */
// template <typename T>
// void LegControllerData<T>::zero() {
//   q = Vector3d::Zero();
//   qd = Vector3d::Zero();
//   p = Vector3d::Zero();
//   v = Vector3d::Zero();
//   J = Matrix3d::Zero();
//   tauEstimate = Vector3d::Zero();
// }

// /*!
//  * Zero all leg commands.  This should be run *before* any control code, so
//  if
//  * the control code is confused and doesn't change the leg command, the legs
//  * won't remember the last command.
//  */
// // void LegController::zeroCommand() {
// //   for (auto& cmd : commands) {
// //     cmd.zero();
// //   }
// //   _legsEnabled = false;
// // }

// /*!
//  * Update the "leg data" from Lowstate
//  */
// void LegController::updateData(const LowState* state) {
//   for (int leg = 0; leg < 4; leg++) {
//     // q
//     datas[0].q(FL_0) = state->motorState[FL_0].q;
//     datas[0].q(FL_1) = state->motorState[FL_1].q;
//     datas[0].q(FL_2) = state->motorState[FL_2].q;

//     datas[1].q(FR_0) = state->motorState[FR_0].q;
//     datas[1].q(FR_1) = state->motorState[FR_1].q;
//     datas[1].q(FR_2) = state->motorState[FR_2].q;

//     datas[2].q(RL_0) = state->motorState[RL_0].q;
//     datas[2].q(RL_1) = state->motorState[RL_1].q;
//     datas[2].q(RL_2) = state->motorState[RL_2].q;

//     datas[3].q(RR_0) = state->motorState[RR_0].q;
//     datas[3].q(RR_1) = state->motorState[RR_1].q;
//     datas[3].q(RR_2) = state->motorState[RR_2].q;

//     // dq
//     datas[0].q(FL_0) = state->motorState[FL_0].dq;
//     datas[0].q(FL_1) = state->motorState[FL_1].dq;
//     datas[0].q(FL_2) = state->motorState[FL_2].dq;

//     datas[1].q(FR_0) = state->motorState[FR_0].dq;
//     datas[1].q(FR_1) = state->motorState[FR_1].dq;
//     datas[1].q(FR_2) = state->motorState[FR_2].dq;

//     datas[2].q(RL_0) = state->motorState[RL_0].dq;
//     datas[2].q(RL_1) = state->motorState[RL_1].dq;
//     datas[2].q(RL_2) = state->motorState[RL_2].dq;

//     datas[3].q(RR_0) = state->motorState[RR_0].dq;
//     datas[3].q(RR_1) = state->motorState[RR_1].dq;
//     datas[3].q(RR_2) = state->motorState[RR_2].dq;

//     datas[0].tauEstimate(FL_0) = state->motorState[FL_0].tauEst;
//     datas[0].tauEstimate(FL_1) = state->motorState[FL_1].tauEst;
//     datas[0].tauEstimate(FL_2) = state->motorState[FL_2].tauEst;

//     datas[1].tauEstimate(FR_0) = state->motorState[FR_0].tauEst;
//     datas[1].tauEstimate(FR_1) = state->motorState[FR_1].tauEst;
//     datas[1].tauEstimate(FR_2) = state->motorState[FR_2].tauEst;

//     datas[2].tauEstimate(RL_0) = state->motorState[RL_0].tauEst;
//     datas[2].tauEstimate(RL_1) = state->motorState[RL_1].tauEst;
//     datas[2].tauEstimate(RL_2) = state->motorState[RL_2].tauEst;

//     datas[3].tauEstimate(RR_0) = state->motorState[RR_0].tauEst;
//     datas[3].tauEstimate(RR_1) = state->motorState[RR_1].tauEst;
//     datas[3].tauEstimate(RR_2) = state->motorState[RR_2].tauEst;

//     // J and p
//     // computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q,
//     // &(datas[leg].J),
//     //                                  &(datas[leg].p), leg);

//     // // v
//     // datas[leg].v = datas[leg].J * datas[leg].qd;
//   }
// }

// /*!
//  * Update the "leg command" for the SPIne board message
//  */
// void LegController::updateCommand(LowCmd* cmd) {
//   // tauFF
//   VectorCXd legTorque;
//   legTorque.setZero(12);
//   legTorque.setZero();
//   legTorque(FL_0) = commands[0].tauFeedForward(FL_0);
//   legTorque(FL_1) = commands[0].tauFeedForward(FL_1);
//   legTorque(FL_2) = commands[0].tauFeedForward(FL_2);

//   legTorque(FR_0) = commands[1].tauFeedForward(FR_0);
//   legTorque(FR_1) = commands[1].tauFeedForward(FR_1);
//   legTorque(FR_2) = commands[1].tauFeedForward(FR_2);

//   legTorque(RL_0) = commands[2].tauFeedForward(RL_0);
//   legTorque(RL_1) = commands[2].tauFeedForward(RL_1);
//   legTorque(RL_2) = commands[2].tauFeedForward(RL_2);

//   legTorque(RR_0) = commands[3].tauFeedForward(RR_0);
//   legTorque(RR_1) = commands[3].tauFeedForward(RR_1);
//   legTorque(RR_2) = commands[3].tauFeedForward(RR_2);
//   // forceFF
//   VectorCXd footForce;
//   footForce.setZero(12);
//   footForce(FL_0) = commands[0].forceFeedForward(FL_0);
//   footForce(FL_1) = commands[0].forceFeedForward(FL_1);
//   footForce(FL_2) = commands[0].forceFeedForward(FL_2);
//   footForce(FR_0) = commands[1].forceFeedForward(FR_0);
//   footForce(FR_1) = commands[1].forceFeedForward(FR_1);
//   footForce(FR_2) = commands[1].forceFeedForward(FR_2);
//   footForce(RL_0) = commands[2].forceFeedForward(RL_0);
//   footForce(RL_1) = commands[2].forceFeedForward(RL_1);
//   footForce(RL_2) = commands[2].forceFeedForward(RL_2);
//   footForce(RR_0) = commands[3].forceFeedForward(RR_0);
//   footForce(RR_1) = commands[3].forceFeedForward(RR_1);
//   footForce(RR_2) = commands[3].forceFeedForward(RR_2);

// //   // cartesian PD
// //   footForce(FL_0) +=
// //       commands.kpCartesian(0) * (commands.pDes(FL_0) - datas[FL_0].p) +
// //       commands.kdCartesian(0) * (commands.vDes(FL_0) - datas[FL_0].v);
// //   footForce(FL_1) +=
// //       commands.kpCartesian(1) * (commands.pDes(FL_1) - datas[FL_1].p) +
// //       commands.kdCartesian(1) * (commands.vDes(FL_1) - datas[FL_1].v);
// //   footForce(FL_2) +=
// //       commands.kpCartesian(2) * (commands.pDes(FL_2) - datas[FL_2].p) +
// //       commands.kdCartesian(2) * (commands.vDes(FL_2) - datas[FL_2].v);
// //   footForce(FR_0) +=
// //       commands.kpCartesian(0) * (commands.pDes(FR_0) - datas[FR_0].p) +
// //       commands.kdCartesian(0) * (commands.vDes(FR_0) - datas[FR_0].v);
// //   footForce(FR_1) +=
// //       commands.kpCartesian(1) * (commands.pDes(FR_1) - datas[FR_1].p) +
// //       commands.kdCartesian(1) * (commands.vDes(FR_1) - datas[FR_1].v);
// //   footForce(FR_2) +=
// //       commands.kpCartesian(2) * (commands.pDes(FR_2) - datas[FR_2].p) +
// //       commands.kdCartesian(2) * (commands.vDes(FR_2) - datas[FR_2].v);
// //   footForce(RL_0) +=
// //       commands.kpCartesian(0) * (commands.pDes(RL_0) - datas[RL_0].p) +
// //       commands.kdCartesian(0) * (commands.vDes(RL_0) - datas[RL_0].v);
// //   footForce(RL_1) +=
// //       commands.kpCartesian(1) * (commands.pDes(RL_1) - datas[RL_1].p) +
// //       commands.kdCartesian(1) * (commands.vDes(RL_1) - datas[RL_1].v);
// //   footForce(RL_2) +=
// //       commands.kpCartesian(2) * (commands.pDes(RL_2) - datas[RL_2].p) +
// //       commands.kdCartesian(2) * (commands.vDes(RL_2) - datas[RL_2].v);
// //   footForce(RR_0) +=
// //       commands.kpCartesian(0) * (commands.pDes(RR_0) - datas[RR_0].p) +
// //       commands.kdCartesian(0) * (commands.vDes(RR_0) - datas[RR_0].v);
// //   footForce(RR_1) +=
// //       commands.kpCartesian(1) * (commands.pDes(RR_1) - datas[RR_1].p) +
// //       commands.kdCartesian(1) * (commands.vDes(RR_1) - datas[RR_1].v);
// //   footForce(RR_2) +=
// //       commands.kpCartesian(2) * (commands.pDes(RR_2) - datas[RR_2].p) +
// //       commands.kdCartesian(2) * (commands.vDes(RR_2) - datas[RR_2].v);

//   // // Torque
//   // legTorque(FL_0) += datas[FL_0].J.transpose() * footForce(FL_0);
//   // legTorque(FL_1) += datas[FL_1].J.transpose() * footForce(FL_1);
//   // legTorque(FL_2) += datas[FL_2].J.transpose() * footForce(FL_2);
//   // legTorque(FR_0) += datas[FR_0].J.transpose() * footForce(FR_0);
//   // legTorque(FR_1) += datas[FR_1].J.transpose() * footForce(FR_1);
//   // legTorque(FR_2) += datas[FR_2].J.transpose() * footForce(FR_2);
//   // legTorque(RL_0) += datas[RL_0].J.transpose() * footForce(RL_0);
//   // legTorque(RL_1) += datas[RL_1].J.transpose() * footForce(RL_1);
//   // legTorque(RL_2) += datas[RL_2].J.transpose() * footForce(RL_2);
//   // legTorque(RR_0) += datas[RR_0].J.transpose() * footForce(RR_0);
//   // legTorque(RR_1) += datas[RR_1].J.transpose() * footForce(RR_1);
//   // legTorque(RR_2) += datas[RR_2].J.transpose() * footForce(RR_2);
//   // set command:
//   cmd->motorCmd[FL_0].tau = legTorque(0);
//   cmd->motorCmd[FL_1].tau = legTorque(1);
//   cmd->motorCmd[FL_2].tau = legTorque(2);

//   cmd->motorCmd[FR_0].tau = legTorque(3);
//   cmd->motorCmd[FR_1].tau = legTorque(4);
//   cmd->motorCmd[FR_2].tau = legTorque(5);

//   cmd->motorCmd[RL_0].tau = legTorque(6);
//   cmd->motorCmd[RL_1].tau = legTorque(7);
//   cmd->motorCmd[RL_2].tau = legTorque(8);

//   cmd->motorCmd[RR_0].tau = legTorque(9);
//   cmd->motorCmd[RR_1].tau = legTorque(10);
//   cmd->motorCmd[RR_2].tau = legTorque(11);
// }

// // constexpr float CHEETAH_3_ZERO_OFFSET[4][3] = {{1.f, 4.f, 7.f},
// //                                                {2.f, 5.f, 8.f},
// //                                                {3.f, 6.f, 9.f}};

// struct LegControllerCommand;
// struct LegControllerData;

// class LegController;

// /*!
//  * Compute the position of the foot and its Jacobian.  This is done in the
//  local
//  * leg coordinate system. If J/p are NULL, the calculation will be skipped.
//  */
// // template <typename T>
// // void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q,
// Mat3<T>*
// // J,
// //                                    Vec3<T>* p, int leg) {
// //   T l1 = quad._abadLinkLength;
// //   T l2 = quad._hipLinkLength;
// //   T l3 = quad._kneeLinkLength;
// //   T l4 = quad._kneeLinkY_offset;
// //   T sideSign = quad.getSideSign(leg);

// //   T s1 = std::sin(q(0));
// //   T s2 = std::sin(q(1));
// //   T s3 = std::sin(q(2));

// //   T c1 = std::cos(q(0));
// //   T c2 = std::cos(q(1));
// //   T c3 = std::cos(q(2));

// //   T c23 = c2 * c3 - s2 * s3;
// //   T s23 = s2 * c3 + c2 * s3;

// //   if (J) {
// //     J->operator()(0, 0) = 0;
// //     J->operator()(0, 1) = l3 * c23 + l2 * c2;
// //     J->operator()(0, 2) = l3 * c23;
// //     J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) *
// sideSign *
// //     s1; J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
// J->operator()(1,
// //     2) = -l3 * s1 * s23; J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 *
// s1 +
// //     (l1+l4) * sideSign * c1; J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1
// *
// //     s2; J->operator()(2, 2) = l3 * c1 * s23;
// //   }

// //   if (p) {
// //     p->operator()(0) = l3 * s23 + l2 * s2;
// //     p->operator()(1) = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2
// *
// //     s1; p->operator()(2) = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2
// *
// //     c1 * c2;
// //   }
// // }

// // template void computeLegJacobianAndPosition<double>(Quadruped<double>&
// quad,
// //                                                     Vec3<double>& q,
// //                                                     Mat3<double>* J,
// //                                                     Vec3<double>* p, int
// //                                                     leg);
// // template void computeLegJacobianAndPosition<float>(Quadruped<float>& quad,
// //                                                    Vec3<float>& q,
// //                                                    Mat3<float>* J,
// //                                                    Vec3<float>* p, int
// leg);
