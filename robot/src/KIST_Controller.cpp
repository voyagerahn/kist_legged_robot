#include "KIST_Controller.hpp"

KIST_Controller::KIST_Controller() {}

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void KIST_Controller::initializeController() {
  // Initialize a new GaitScheduler object
  _gaitScheduler =
      new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);

  // Initialize a new ContactEstimator object
  //_contactEstimator = new ContactEstimator<double>();
  ////_contactEstimator->initialize();

  // Initializes the Control FSM with all the required data
  _controlFSM = new ControlFSM<float>(
      _quadruped, _stateEstimator, _legController, _gaitScheduler,
      _controlParameters, &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void KIST_Controller::runController() {
  // Find the current gait schedule
  _gaitScheduler->step();
  // Find the desired state trajectory
  //   _desiredStateCommand->convertToStateCommands();

  // Run the Control FSM code
  _controlFSM->runFSM();
}
