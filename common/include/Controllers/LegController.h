/*! @file LegController.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards (the low level leg control boards)
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H

#include "cppTypes.h"
#include "Dynamics/Quadruped.h"
#include "Dynamics/CustomModel.h"
#include "../../robot/include/kist_legged_robot.h"
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>

using namespace UNITREE_LEGGED_SDK;

/*!
 * Data sent from the control algorithm to the legs.
 */
template <typename T>
struct LegControllerCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }

  void zero();

  Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
  Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

/*!
 * Data returned from the legs to the control code.
 */
template <typename T>
struct LegControllerData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() { zero(); }

  void setQuadruped(Quadruped<T>& quad) { quadruped = &quad; }

  void zero();

  Vec3<T> q, qd, p, v;
  Mat3<T> J;
  Vec3<T> tauEstimate;
  Quadruped<T>* quadruped;
};

/*!
 * Controller for 4 legs of a quadruped.  Works for both Mini Cheetah and Cheetah 3
 */
template <typename T>
class LegController {
 public:
  LegController(Quadruped<T>& quad) : _quadruped(quad) {
    datas->setQuadruped(_quadruped);
    _q.setZero(19);
    _dq.setZero(18);
  }

  void zeroCommand();
  void updateData(const LowState* state);
  void updateCommand(LowCmd* cmd);
  void setEnabled(bool enabled) { _legsEnabled = enabled; };
  float torque_limit = 7.5;

  /*!
   * Set the maximum torque.  This only works on cheetah 3!
   */
  void setMaxTorqueCheetah3(T tau) { _maxTorque = tau; }

  LegControllerCommand<T> commands[4];
  LegControllerData<T> datas[4];
  Quadruped<T>& _quadruped;
  bool _legsEnabled = false;
  T _maxTorque = 0;
  bool _zeroEncoders = false;
  u32 _calibrateEncoders = 0;

  VectorXd _q, _dq;
  CModel m;
};



template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg);
// template <typename T>
// void computeLegJacobian(RigidBodyDynamics::Model& _model, DVec<T>& _q,
//                         Quat<T>& orientation, DMat<T>* J);

#endif  // PROJECT_LEGCONTROLLER_H