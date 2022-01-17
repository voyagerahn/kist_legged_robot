/*!
 * @file FootSwingTrajectory.h
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#ifndef A1_FOOTSWINGTRAJECTORY_H
#define A1_FOOTSWINGTRAJECTORY_H

#include "cppTypes.h"

/*!
 * A foot swing trajectory for a single foot
 */
template<typename T>
class FootSwingTrajectory {
public:

  /*!
   * Construct a new foot swing trajectory with everything set to zero
   */
  FootSwingTrajectory() {
    _p0.setZero();
    _pf.setZero();
    _p.setZero();
    _v.setZero();
    _a.setZero();
    _height = 0;
  }

  /*!
   * Set the starting location of the foot
   * @param p0 : the initial foot position
   */
  void setInitialPosition(Vec3<T> p0) {
    _p0 = p0;
  }

  /*!
   * Set the desired final position of the foot
   * @param pf : the final foot posiiton
   */
  void setFinalPosition(Vec3<T> pf) {
    _pf = pf;
  }

  /*!
   * Set the maximum height of the swing
   * @param h : the maximum height of the swing, achieved halfway through the swing
   */
  void setHeight(T h) {
    _height = h;
  }

  void computeSwingTrajectoryBezier(T phase, T swingTime);

  /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
   */
  Vec3<T> getPosition() {
    return _p;
  }

  /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
   */
  Vec3<T> getVelocity() {
    return _v;
  }

  /*!
   * Get the foot acceleration at the current point along the swing
   * @return : the foot acceleration
   */
  Vec3<T> getAcceleration() {
    return _a;
  }

private:
  Vec3<T> _p0, _pf, _p, _v, _a;
  T _height;
};


#endif //A1_FOOTSWINGTRAJECTORY_H






// /*!
//  * @file FootSwingTrajectory.h
//  * @brief Utility to generate foot swing trajectories.
//  *
//  * Currently uses Bezier curves like Cheetah 3 does
//  */

// #ifndef CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
// #define CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H

// #include "kist_legged_robot/kist_legged_robot.h"

// /*!
//  * A foot swing trajectory for a single foot
//  */
// class FootSwingTrajectory {
// public:

//   /*!
//    * Construct a new foot swing trajectory with everything set to zero
//    */
//   FootSwingTrajectory() {
//     _p0.setZero();
//     _pf.setZero();
//     _p.setZero();
//     _v.setZero();
//     _a.setZero();
//     _height = 0;
//   }

//   /*!
//    * Set the starting location of the foot
//    * @param p0 : the initial foot position
//    */
//   void setInitialPosition(Vector3f p0) {
//     _p0 = p0;
//   }

//   /*!
//    * Set the desired final position of the foot
//    * @param pf : the final foot posiiton
//    */
//   void setFinalPosition(Vector3f pf) {
//     _pf = pf;
//   }

//   /*!
//    * Set the maximum height of the swing
//    * @param h : the maximum height of the swing, achieved halfway through the swing
//    */
//   void setHeight(float h) {
//     _height = h;
//   }

//   void computeSwingTrajectoryBezier(float phase, float swingTime);

//   /*!
//    * Get the foot position at the current point along the swing
//    * @return : the foot position
//    */
//   Vector3f getPosition() {
//     return _p;
//   }

//   /*!
//    * Get the foot velocity at the current point along the swing
//    * @return : the foot velocity
//    */
//   Vector3f getVelocity() {
//     return _v;
//   }

//   /*!
//    * Get the foot acceleration at the current point along the swing
//    * @return : the foot acceleration
//    */
//   Vector3f getAcceleration() {
//     return _a;
//   }

// private:
//   Vector3f _p0, _pf, _p, _v, _a;
//   float _height;
// };


// #endif //CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
