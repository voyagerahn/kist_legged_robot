/*! @file Quadruped.h
 *  @brief Data structure containing parameters for quadruped robot
 *
 *  This file contains the Quadruped class.  This stores all the parameters for
 * a quadruped robot.  There are utility functions to generate Quadruped objects
 * for Cheetah 3 (and eventually mini-cheetah). There is a buildModel() method
 * which can be used to create a floating-base dynamics model of the quadruped.
 */

#ifndef LIBBIOMIMETICS_QUADRUPED_H
#define LIBBIOMIMETICS_QUADRUPED_H

#include <eigen3/Eigen/StdVector>
#include <vector>

#include "Dynamics/ActuatorModel.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/SpatialInertia.h"

/*!
 * Basic parameters for a a1 robot
 */
namespace a1 {
constexpr size_t num_act_joint = 12;
constexpr size_t num_q = 19;
constexpr size_t dim_config = 18;
constexpr size_t num_leg = 4;
constexpr size_t num_leg_joint = 3;
}  // namespace a1

/*!
 * Link indices for cheetah-shaped robots
 */
namespace linkID {
constexpr size_t FR = 0;  // Front Right Foot
constexpr size_t FL = 1;  // Front Left Foot
constexpr size_t HR = 2;  // Hind Right Foot
constexpr size_t HL = 3;  // Hind Left Foot

constexpr size_t FR_abd = 0;  // Front Right Abduction
constexpr size_t FL_abd = 3;  // Front Left Abduction
constexpr size_t HR_abd = 6;  // Hind Right Abduction
constexpr size_t HL_abd = 9;  // Hind Left Abduction
}  // namespace linkID

using std::vector;

/*!
 * Representation of a quadruped robot's physical properties.
 *
 * When viewed from the top, the quadruped's legs are:
 *
 * FRONT
 * 2 1   RIGHT
 * 4 3
 * BACK
 *
 */
template <typename T>
class Quadruped {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotType _robotType;
  T _bodyLength, _bodyWidth, _bodyHeight, _bodyMass;
  T _abadGearRatio, _hipGearRatio, _kneeGearRatio;
  T _abadLinkLength, _hipLinkLength, _kneeLinkLength, _kneeLinkY_offset,
      _maxLegLength;
  T _motorKT, _motorR, _batteryV;
  T _motorTauMax;
  T _jointDamping, _jointDryFriction;
  SpatialInertia<T> _abadInertia, _hipInertia, _kneeInertia, _abadRotorInertia,
      _hipRotorInertia, _kneeRotorInertia, _bodyInertia;
  Vec3<T> _abadLocation, _abadRotorLocation, _hipLocation, _hipRotorLocation,
      _kneeLocation, _kneeRotorLocation;
  FloatingBaseModel<T> buildModel();
  bool buildModel(FloatingBaseModel<T>& model);
  std::vector<ActuatorModel<T>> buildActuatorModels();

  /*!
   * Get if the i-th leg is on the left (+) or right (-) of the robot.
   * @param leg : the leg index
   * @return The side sign (-1 for right legs, +1 for left legs)
   */
  static T getSideSign(int leg) {
    const T sideSigns[4] = {-1, 1, -1, 1};
    assert(leg >= 0 && leg < 4);
    return sideSigns[leg];
  }

  /*!
   * Get location of the hip for the given leg in robot frame
   * @param leg : the leg index
   */
  Vec3<T> getHipLocation(int leg) {
    assert(leg >= 0 && leg < 4);
    // Vec3<T> pHip((leg == 0 || leg == 1) ? _abadLocation(0) :
    // -_abadLocation(0), //0.1805
    //              (leg == 1 || leg == 3) ? _abadLocation(1) :
    //              -_abadLocation(1), // 0.047 _abadLocation(2));
    Vec3<T> pHip((leg == 0 || leg == 1) ? 0.17 : -0.195,
                 (leg == 1 || leg == 3) ? 0.13 : -0.135,  // 0.047
                 0);
    return pHip;
  }

  // /*!
  //  * Get location of the foot for the given leg in robot frame
  //  */
  // Vec3<T> get_Foot_Positions_In_Base_Frame(Quadruped<T>& quad, Vec3<float> jointAngle, int leg,
  //                                          Vec3<float> jointAngles) {
  //   Vec3<T> hip_offset(getHipLocation(0),getHipLocation(1),getHipLocation(2));
  //   Vec3<T> foot_positions = get_Foot_Position_In_Hip_Frame(quad, jointAngle, leg);
  //   return foot_positions + hip_offset;
  // }
  /*!
   * Get location of the foot for the given leg in hip frame
   */
  Vec3<T> get_Foot_Position_In_Hip_Frame(Quadruped<T>& quad, Vec3<float> jointAngle, int leg) {
    T l_hip = quad._hipLinkLength;
    T l_knee = quad._kneeLinkLength;
    T l_abad = quad._abadLinkLength * quad.getSideSign(leg);
    T leg_distance =
        std::sqrt(l_hip * l_hip + l_knee * l_knee + 2 * l_hip * l_knee * std::cos(jointAngle(2)));
    T eff_swing = jointAngle(1) + jointAngle(2) / 2;
    T off_x_hip = -leg_distance * std::sin(eff_swing);
    T off_z_hip = -leg_distance * std::cos(eff_swing);
    T off_y_hip = l_abad;

    T off_x = off_x_hip;
    T off_y = std::cos(jointAngle(0)) * off_y_hip -
              std::sin(jointAngle(0)) * off_z_hip;
    T off_z = std::sin(jointAngle(0)) * off_y_hip +
              std::cos(jointAngle(0)) * off_z_hip;
    Vec3<T> off(off_x, off_y, off_z);
    return off;
  }
};

template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2>& v, int legID);

#endif  // LIBBIOMIMETICS_QUADRUPED_H
