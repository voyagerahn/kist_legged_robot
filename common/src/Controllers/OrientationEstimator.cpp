
/*! @file OrientationEstimator.cpp
 *  @brief All Orientation Estimation Algorithms
 *
 *  This file will contain all orientation algorithms.
 *  Orientation estimators should compute:
 *  - orientation: a quaternion representing orientation
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 */

#include "Controllers/OrientationEstimator.h"

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */
template <typename T>
void VectorNavOrientationEstimator<T>::run() {
  this->_stateEstimatorData.result->orientation[0] =
      this->_stateEstimatorData.vectorNavData->quaternion[0];
  this->_stateEstimatorData.result->orientation[1] =
      this->_stateEstimatorData.vectorNavData->quaternion[1];
  this->_stateEstimatorData.result->orientation[2] =
      this->_stateEstimatorData.vectorNavData->quaternion[2];
  this->_stateEstimatorData.result->orientation[3] =
      this->_stateEstimatorData.vectorNavData->quaternion[3];

  if (_b_first_visit) {
    Vec3<T> rpy_ini =
        ori::quatToRPY(this->_stateEstimatorData.result->orientation);
    rpy_ini[0] = 0;
    rpy_ini[1] = 0;
    _ori_ini_inv = rpyToQuat(-rpy_ini);
    _b_first_visit = false;
  }
  this->_stateEstimatorData.result->orientation = ori::quatProduct(
      _ori_ini_inv, this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->rpy =
      ori::quatToRPY(this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->omegaBody(0) =
      this->_stateEstimatorData.vectorNavData->gyroscope[0];
  this->_stateEstimatorData.result->omegaBody(1) =
      this->_stateEstimatorData.vectorNavData->gyroscope[1];
  this->_stateEstimatorData.result->omegaBody(2) =
      this->_stateEstimatorData.vectorNavData->gyroscope[2];

  this->_stateEstimatorData.result->omegaWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->omegaBody;

  this->_stateEstimatorData.result->aBody(0) =
      this->_stateEstimatorData.vectorNavData->accelerometer[0];
  this->_stateEstimatorData.result->aBody(1) =
      this->_stateEstimatorData.vectorNavData->accelerometer[1];
  this->_stateEstimatorData.result->aBody(2) =
      this->_stateEstimatorData.vectorNavData->accelerometer[2];
  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->aBody;
}

template class VectorNavOrientationEstimator<float>;
template class VectorNavOrientationEstimator<double>;

// /*! @file OrientationEstimator.cpp
//  *  @brief All Orientation Estimation Algorithms
//  *
//  *  This file will contain all orientation algorithms.
//  *  Orientation estimators should compute:
//  *  - orientation: a quaternion representing orientation
//  *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody *
//  vWorld)
//  *  - omegaBody: angular velocity in body frame
//  *  - omegaWorld: angular velocity in world frame
//  *  - rpy: roll pitch yaw
//  */

// #include "Controllers/OrientationEstimator.h"

// /*!
//  * Get quaternion, rotation matrix, angular velocity (body and world),
//  * rpy, acceleration (world, body) from vector nav IMU
//  */
// void VectorNavOrientationEstimator::run() {
//   this->_stateEstimatorData.result->orientation(0) =
//       this->_stateEstimatorData.vectorNavData->quaternion[0];
//   this->_stateEstimatorData.result->orientation(1) =
//       this->_stateEstimatorData.vectorNavData->quaternion[1];
//   this->_stateEstimatorData.result->orientation(2) =
//       this->_stateEstimatorData.vectorNavData->quaternion[2];
//   this->_stateEstimatorData.result->orientation(3) =
//       this->_stateEstimatorData.vectorNavData->quaternion[3];

//   if (_b_first_visit) {
//     Vector3d rpy_ini =
//         CustomMath::quatToRPY(this->_stateEstimatorData.result->orientation);
//     rpy_ini[0] = 0;
//     rpy_ini[1] = 0;
//     _ori_ini_inv = CustomMath::rpyToQuat(-rpy_ini);
//     _b_first_visit = false;
//   }
//   this->_stateEstimatorData.result->orientation = CustomMath::quatProduct(
//       _ori_ini_inv, this->_stateEstimatorData.result->orientation);

//   this->_stateEstimatorData.result->rpy =
//       CustomMath::quatToRPY(this->_stateEstimatorData.result->orientation);

//   this->_stateEstimatorData.result->rBody =
//   CustomMath::quaternionToRotationMatrix(
//       this->_stateEstimatorData.result->orientation);

//   this->_stateEstimatorData.result->omegaBody(0) =
//       this->_stateEstimatorData.vectorNavData->gyroscope[0];
//   this->_stateEstimatorData.result->omegaBody(1) =
//       this->_stateEstimatorData.vectorNavData->gyroscope[1];
//   this->_stateEstimatorData.result->omegaBody(2) =
//       this->_stateEstimatorData.vectorNavData->gyroscope[2];

//   this->_stateEstimatorData.result->omegaWorld =
//       this->_stateEstimatorData.result->rBody.transpose() *
//       this->_stateEstimatorData.result->omegaBody;

//   this->_stateEstimatorData.result->aBody(0) =
//       this->_stateEstimatorData.vectorNavData->accelerometer[0];
//   this->_stateEstimatorData.result->aBody(1) =
//       this->_stateEstimatorData.vectorNavData->accelerometer[1];
//   this->_stateEstimatorData.result->aBody(2) =
//       this->_stateEstimatorData.vectorNavData->accelerometer[2];

//   this->_stateEstimatorData.result->aWorld =
//       this->_stateEstimatorData.result->rBody.transpose() *
//       this->_stateEstimatorData.result->aBody;
// }
