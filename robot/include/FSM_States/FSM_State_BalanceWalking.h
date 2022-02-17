#ifndef FSM_STATE_BALANCEWALKING_H
#define FSM_STATE_BALANCEWALKING_H

#include "../../src/Controller/convexMPC/Locomotion.h"
#include "FSM_State.h"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;

/**
 *
 */
template <typename T>
class FSM_State_BalanceWalking : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_BalanceWalking(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter() override;

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

 private:
  // Keep track of the control iterations
  int _iter = 0;
  Locomotion* cLocomotion;
  // Parses contact specific controls to the leg controller
  void BalanceStandStep();
  bool locomotionSafe();
};

#endif  // FSM_STATE_BALANCESTAND_H
