#ifndef _KIST_CONTROLLER_
#define _KIST_CONTROLLER_

// #include "../../src/FSM_States/ControlFSM.h"
#include "ControlParameters/RobotParameters.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/FloatingBaseModel.h"
#include "KIST_UserParameters.h"

// #include "Controllers/DesiredStateCommand.h"

class KIST_Controller {
    friend class RobotRunner;
 public:
  KIST_Controller();
  virtual ~KIST_Controller() {}

  void initializeController();
  void runController();
  void updateVisualization() {}
  void Estop() {}

  virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
  }
  //   virtual void Estop(){ _controlFSM->initialize(); }

 protected:
  RobotControlParameters* _controlParameters = nullptr;
  Quadruped<float>* _quadruped = nullptr;
  FloatingBaseModel<float>* _model = nullptr;
  LegController<float>* _legController = nullptr;
  StateEstimatorContainer<float>* _stateEstimator = nullptr;
  StateEstimate<float>* _stateEstimate = nullptr;
  // ControlFSM<float>* _controlFSM;
  // Gait Scheduler controls the nominal contact schedule for the feet
  KIST_UserParameters userParameters;
  GaitScheduler<float>* _gaitScheduler;
  // DesiredStateCommand<float>* _desiredStateCommand = nullptr;
};

#endif
