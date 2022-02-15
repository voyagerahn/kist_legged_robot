/*=========================== Balance Stand ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "FSM_States/FSM_State_BalanceWalking.h"
#include "../Controller/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_BalanceWalking<T>::FSM_State_BalanceWalking(
    ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_WALKING,
                   "BALANCE_WALKING") {
  cLocomotion = new Locomotion(
      _controlFSMData->controlParameters->controller_dt,
      _controlFSMData->userParameters);
  // Set the pre controls safety checks
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;
  // Initialize GRF and footstep locations to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();
  this->footstepLocations = Mat34<T>::Zero();
}

template <typename T>
void FSM_State_BalanceWalking<T>::onEnter() {
   // Default is to not transition
  this->nextStateName = this->stateName;

  this->transitionData.zero();
  cLocomotion->initialize();
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
  printf("[FSM BALANCE_WALKING] On Enter\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_BalanceWalking<T>::run() {

  BalanceStandStep();
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_BalanceWalking<T>::checkTransition() {
  // Get the next state
  _iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_BALANCE_WALKING:
      // Normal operation for state based transitions

      // Need a working state estimator for this
      /*if (velocity > v_max) {
        // Notify the State of the upcoming next state
        this->nextStateName = FSM_StateName::LOCOMOTION;

        // Transition instantaneously to locomotion state on request
        this->transitionDuration = 0.0;

        // Set the next gait in the scheduler to
        this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;

      }*/
      break;

    case K_LOCOMOTION:
      // Requested change to balance stand
      this->nextStateName = FSM_StateName::LOCOMOTION;

      // Transition instantaneously to locomotion state on request
      this->transitionDuration = 0.0;

      // Set the next gait in the scheduler to
      this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
      break;

    case K_PASSIVE:
      this->nextStateName = FSM_StateName::PASSIVE;
      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_BALANCE_WALKING << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Return the next state name to the FSM
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_BalanceWalking<T>::transition() {
//   // Switch FSM control mode
//   switch (this->nextStateName) {
//     case FSM_StateName::LOCOMOTION:
//       BalanceStandStep();

//       _iter++;
//       if (_iter >= this->transitionDuration * 1000) {
//         this->transitionData.done = true;
//       } else {
//         this->transitionData.done = false;
//       }

//       break;

//     case FSM_StateName::PASSIVE:
//       this->turnOffAllSafetyChecks();
//       this->transitionData.done = true;
//       break;
//     default:
//       std::cout << "[CONTROL FSM] Something went wrong in transition"
//                 << std::endl;
//   }

//   // Return the transition data to the FSM
//   return this->transitionData;
}
template<typename T>
bool FSM_State_BalanceWalking<T>::locomotionSafe() {
  auto& seResult = this->_data->_stateEstimator->getResult();

  const T max_roll = 40;
  const T max_pitch = 40;

  if(std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if(std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for(int leg = 0; leg < 4; leg++) {
    auto p_leg = this->_data->_legController->datas[leg].p;
    if(p_leg[2] > 0) {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    if(std::fabs(p_leg[1] > 0.18)|| std::fabs(p_leg[1] < -0.18)) {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return false;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();
    if(std::fabs(v_leg) > 9.) {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }

  return true;

}
/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_BalanceWalking<T>::onExit() {
  _iter = 0;
}

/**
 * Calculate the commands for the leg controllers for each of the feet.
 */
template <typename T>
void FSM_State_BalanceWalking<T>::BalanceStandStep() {
  // this->runBalanceController();
  // cLocomotion->run<T>(*this->_data);
  cLocomotion->run<T>(*this->_data);
  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

}

// template class FSM_State_BalanceWalking<double>;
template class FSM_State_BalanceWalking<float>;
