#include "RobotRunner.h"

#include <fstream>

#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "Dynamics/A1.h"
#include <time.h>
using namespace std;
using namespace UNITREE_LEGGED_SDK;

void RobotRunner::Initialize() {
  _quadruped = buildA1<float>();
  _model = _quadruped.buildModel();

  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);

  _stateEstimator = new StateEstimatorContainer<float>(
      &imu, _legController->datas, &_stateEstimate, controlParameters);

  initializeStateEstimator();

  // TODO :   Initialize the DesiredStateCommand object
  // -----------------------------example-----------------------
  // _desiredStateCommand = new DesiredStateCommand<float>(
  //     driverCommand, &rc_control, controlParameters, &_stateEstimate,
  //     controlParameters->controller_dt);

  // Controller initializations
  _robot_ctrl->_model = &_model;
  _robot_ctrl->_quadruped = &_quadruped;
  _robot_ctrl->_legController = _legController;
  _robot_ctrl->_stateEstimator = _stateEstimator;
  _robot_ctrl->_stateEstimate = &_stateEstimate;
  _robot_ctrl->_controlParameters = controlParameters;
  // _kist_ctrl->_desiredStateCommand = _desiredStateCommand;

  _robot_ctrl->initializeController();
}

void RobotRunner::initializeStateEstimator() {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);
  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
}

void RobotRunner::UDPRecv() { udp.Recv(); }

void RobotRunner::UDPSend() { udp.Send(); }

void RobotRunner::Run() {
  // cout << result << endl;
  // start = clock();
  _stateEstimator->run();

  udp.GetRecv(state);
  _legController->updateData(&state);
  // _time = (float)motiontime * dt;
  _robot_ctrl->runController();
  saveLogData(motiontime);

  _legController->updateCommand(&cmd);
  safe.PositionLimit(cmd);
  safe.PowerProtect(cmd, state, 7);
  udp.SetSend(cmd);
  motiontime++;
  // end = clock();
  // result = (double)(end-start)/CLOCKS_PER_SEC;
}

void RobotRunner::setupStep() {
  udp.GetRecv(state);
  _legController->updateData(&state);
}

void RobotRunner::finalizeStep() {
  _legController->updateCommand(&cmd);

  safe.PositionLimit(cmd);
  safe.PowerProtect(cmd, state, 7);
  udp.SetSend(cmd);
  motiontime++;
}


void RobotRunner::saveLogData(int motiontime) {
  logdata[0][motiontime] = _legController->datas[0].q(0);
  logdata[1][motiontime] = _legController->datas[0].q(1); 
  logdata[2][motiontime] = _legController->datas[0].q(2); 
  logdata[3][motiontime] = _legController->datas[1].q(0);
  logdata[4][motiontime] = _legController->datas[1].q(1); 
  logdata[5][motiontime] = _legController->datas[1].q(2);   
}

RobotRunner::~RobotRunner() {
  delete _legController;
  delete _stateEstimator;
}