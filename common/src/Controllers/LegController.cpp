/*! @file LegController.cpp
 *  @brief Common Leg Control Interface
 *
 *  Implements low-level leg control for unitree A1 Robot
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
 * Update the "leg data" from a LowState
 *
 * Representation of a quadruped robot's physical properties.
 * (q, qd, tau, J, v and p)
 * When viewed from the top, the quadruped's legs are:
 *
 * FRONT
 * 1 0   RIGHT
 * 3 2
 * BACK
 *
 * 0:Front Right 1: Front Left 2: Rear Right 3: Rear Left
 */
template <typename T>
void LegController<T>::updateData(const LowState* state) {
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      datas[leg].q(joint) = state->motorState[3 * leg + joint].q;
      datas[leg].qd(joint) = state->motorState[3 * leg + joint].dq;
      // datas[leg]                 .tauEstimate(joint) =
      //     state->motorState[3 * leg + joint].tauEst;
    }
    // J and p
    computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &(datas[leg].J),
                                     &(datas[leg].p), leg);
    datas[leg].v = datas[leg].J * datas[leg].qd;
  }
}

/*!
 * Update the "leg command" for the LowCommand
 */
template <typename T>
void LegController<T>::updateCommand(LowCmd* cmd) {
  for (int leg = 0; leg < 4; leg++) {
    // tauFF
    Vec3<T> legTorque;
    // forceFF
    Vec3<T> footForce;
    cout<<leg << " : pDes " <<commands[leg].pDes.transpose()<< endl;
    cout<<leg << " : p " <<datas[leg].p.transpose()<< endl;
    // cout << "--------------------------------------" << endl;
    // cartesian PD
    commands[leg].vDes << 0.0,0.0,0.0;
    footForce =
        commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p) +
        commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v);
    // cout << "qDes : " << datas[0].q.transpose()<<endl;

    // // cout << "command : " << commands[0].qDes.transpose()<<endl;
    // // cout << "--------------------------------------" << endl;
    // cout << "footForce : " << footForce.transpose()<<endl;

    // Torque
    legTorque += datas[leg].J.transpose() * footForce;

    // legTorque = commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +
    //             commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);
   
    legTorque(0) = (legTorque(0) > torque_limit) ? torque_limit : legTorque(0);
    legTorque(1) = (legTorque(1) > torque_limit) ? torque_limit : legTorque(1);
    legTorque(2) = (legTorque(2) > torque_limit) ? torque_limit : legTorque(2);
    cout << leg <<" legTorque : " << legTorque.transpose()<<endl;
    cout << "--------------------------------------" << endl;
    // set command
    cmd->motorCmd[3 * leg].tau = legTorque(0);
    cmd->motorCmd[3 * leg + 1].tau = legTorque(1);
    cmd->motorCmd[3 * leg + 2].tau = legTorque(2);
  }

  // cout<<"FR(0)" << cmd->motorCmd[0].tau << endl;
  // cout<<"FR(1)" << cmd->motorCmd[1].tau << endl;
  // cout<<"FR(2)" << cmd->motorCmd[2].tau << endl;
  // cout << "---------------------------------------" <<endl;

  // cout<<"FL(0)" << cmd->motorCmd[3].tau << endl;
  // cout<<"FL(1)" << cmd->motorCmd[4].tau << endl;
  // cout<<"FL(2)" << cmd->motorCmd[5].tau << endl;
  // cout << "---------------------------------------" <<endl;

  // cout<<"RR(0)" << cmd->motorCmd[6].tau << endl;
  // cout<<"RR(1)" << cmd->motorCmd[7].tau << endl;
  // cout<<"RR(2)" << cmd->motorCmd[8].tau << endl;
  // cout << "---------------------------------------" <<endl;

  // cout<<"RL(0)" << cmd->motorCmd[9].tau << endl;
  // cout<<"RL(1)" << cmd->motorCmd[10].tau << endl;
  // cout<<"RL(2)" << cmd->motorCmd[11].tau << endl;
  // cout << "---------------------------------------" <<endl;

  // cmd->motorCmd[0].tau = 0.0;
  // cmd->motorCmd[1].tau = 0.0;
  // cmd->motorCmd[2].tau = 0.0;

  // cmd->motorCmd[3].tau = 0.0;
  // cmd->motorCmd[4].tau = 0.0;
  // cmd->motorCmd[5].tau = 0.0;

  // cmd->motorCmd[6].tau = 0.0;
  // cmd->motorCmd[7].tau = 0.0;
  // cmd->motorCmd[8].tau = 0.0;

  // cmd->motorCmd[9].tau = 0.0;
  // cmd->motorCmd[10].tau =0.0;
  // cmd->motorCmd[11].tau =0.0;


  // // estimate torque
  // datas[leg].tauEstimate =
  //     legTorque + commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q)
  //     + commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);

  // spiCommand->flags[leg] = _legsEnabled ? 1 : 0;
}
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
