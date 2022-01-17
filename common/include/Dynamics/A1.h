/*! @file A1.h
 *  @brief Utility function to build a A1 Quadruped object
 *
 * This file is based on A1FullRotorModel_mex.m and builds a model
 * of the A1 robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_A1_H
#define PROJECT_A1_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of A1
 */
template <typename T>
Quadruped<T> buildA1() {
  Quadruped<T> a1;
  a1._robotType = RobotType::A1;

  a1._bodyMass = 10.0; //before 6
  a1._bodyLength = 0.1805 * 2;
  a1._bodyWidth = 0.047 * 2;
  a1._bodyHeight = 0.05 * 2;
  a1._abadGearRatio = 6;
  a1._hipGearRatio = 6;
  a1._kneeGearRatio = 9.33;
  a1._abadLinkLength = 0.0838;
  a1._hipLinkLength = 0.20;
  a1._kneeLinkY_offset = 0.004;
  a1._kneeLinkLength = 0.20;
  a1._maxLegLength = 0.409;


  a1._motorTauMax = 3.f;
  a1._batteryV = 24;
  a1._motorKT = .05;  // this is flux linkage * pole pairs
  a1._motorR = 0.173;
  a1._jointDamping = .01;
  a1._jointDryFriction = .2;
  //cheetah._jointDamping = .0;
  //cheetah._jointDryFriction = .0;


  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0, 0.036, 0);  // LEFT
  SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0, 0.016, -0.02);
  SpatialInertia<T> hipInertia(0.634, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0, 0, -0.061);
  SpatialInertia<T> kneeInertia(0.064, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(a1._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  a1._abadInertia = abadInertia;
  a1._hipInertia = hipInertia;
  a1._kneeInertia = kneeInertia;
  a1._abadRotorInertia = rotorInertiaX;
  a1._hipRotorInertia = rotorInertiaY;
  a1._kneeRotorInertia = rotorInertiaY;
  a1._bodyInertia = bodyInertia;

  // locations
  a1._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  a1._abadLocation =
      Vec3<T>(a1._bodyLength, a1._bodyWidth, 0) * 0.5;
  a1._hipLocation = Vec3<T>(0, a1._abadLinkLength, 0);
  a1._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  a1._kneeLocation = Vec3<T>(0, 0, -a1._hipLinkLength);
  a1._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return a1;
}

#endif  // PROJECT_A1_H
