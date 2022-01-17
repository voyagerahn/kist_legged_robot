// #include "Controllers/GaitScheduler.h"
#include "KIST_Controller.hpp"
#include "KIST_UserParameters.h"
#include "RobotRunner.h"
#include "loop.h"

using namespace UNITREE_LEGGED_SDK;

int main(void) {
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();
  
  KIST_Controller* ctrl;

  ctrl = new KIST_Controller();
  
  RobotRunner _robotRunner(LOWLEVEL, ctrl);
  // KIST_UserParameters userParameters;
  // ofstream file[data];
  // int LengthOfFile = sizeof(file) / sizeof(*file); //60
  LoopFunc loop_torque_control("torque control", _robotRunner.dt,
                               boost::bind(&RobotRunner::Run, &_robotRunner));
  LoopFunc loop_udpSend("udp_send", _robotRunner.dt, 3,
                        boost::bind(&RobotRunner::UDPSend, &_robotRunner));
  LoopFunc loop_udpRecv("udp_recv", _robotRunner.dt, 3,
                        boost::bind(&RobotRunner::UDPRecv, &_robotRunner));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_torque_control.start();

  // if (cin.get() == '\n')
  // {
  //     for (int i = 0; i < LengthOfFile; i++)
  //     {
  //         fileName[i] = "../data/" + fileName[i];
  //         file[i].open(fileName[i]);
  //     }
  //     for (int i = 0; i < LengthOfFile; i++)
  //     {
  //         for (int j = 0; j < chlc.motiontime; j++)
  //             file[i] << j*chlc.dt << " " << _mem_fprint[i][j] <<endl;
  //     }
  //     cout << "terminate" << endl;

  //     return 0;
  // }

  while (1) {
  };
  return 0;
}
