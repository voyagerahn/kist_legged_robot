// #include "Controllers/GaitScheduler.h"

#include "KIST_Controller.hpp"
#include "KIST_UserParameters.h"
#include "RobotRunner.h"
#include "loop.h"

using namespace UNITREE_LEGGED_SDK;

int main(void) {
  KIST_Controller ctrl;
  KIST_UserParameters userParameters;
  RobotControlParameters _robotParams;
  ControlParameters* _userControlParameters = nullptr;
  _userControlParameters = ctrl.getUserControlParameters();

  // Load parameters from file
  printf("Loading parameters from file...\n");
  try {
    _robotParams.initializeFromYamlFile(
        "/home/kist/kist_legged_project/config/kist-legged-robot-defaults.yaml");
  } catch (std::exception& e) {
    printf("Failed to initialize robot parameters from yaml file: %s\n",
           e.what());
    exit(1);
  }
  printf("Loaded robot parameters\n");
  try {
    _userControlParameters->initializeFromYamlFile(
        "/home/kist/kist_legged_project/config/kist-ctrl-user-parameters.yaml");
  } catch (std::exception& e) {
    printf("Failed to initialize user parameters from yaml file: %s\n",
           e.what());
    exit(1);
  }
  printf("Loaded user parameters\n");

  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  RobotRunner _robotRunner(LOWLEVEL, &ctrl);
  _robotRunner.controlParameters = &_robotParams;
  _robotRunner.Initialize();
  LoopFunc loop_torque_control("torque control", _robotRunner.dt,
                               boost::bind(&RobotRunner::Run, &_robotRunner));
  LoopFunc loop_udpSend("udp_send", _robotRunner.dt, 3,
                        boost::bind(&RobotRunner::UDPSend, &_robotRunner));
  LoopFunc loop_udpRecv("udp_recv", _robotRunner.dt, 3,
                        boost::bind(&RobotRunner::UDPRecv, &_robotRunner));
  loop_udpSend.start();
  loop_udpRecv.start();
  loop_torque_control.start();

  // ofstream file[data];
  // int LengthOfFile = sizeof(file) / sizeof(*file); //60
  if (cin.get() == '\n') {
    // delete _robotRunner;
    cout << "-------------PROGRAM FINISHED-------------" << endl;

    return 0;
  }

  while (1) {
  };
  return 0;
}