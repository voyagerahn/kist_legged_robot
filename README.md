## KIST-Quadruped-Robot-Software
This repository contains the Robot software project.  For a getting started guide, see the README.md.

The common folder contains the common library with dynamics and utilities
The resources folder will contain data files, like CAD of the robot used for the visualization
The robot folder will contain the robot program
The third-party will contain *small* third party libraries that we have modified.

## Build
To build all code:
```
mkdir build
cd build
cmake ..
make
```

## Run unitree A1 
1. Connect to A1 network (over wifi : 'UntireeRoboticsA1-846'  or wire LAN port)
2. Enter the robot program folder `cd build/robot/`
3. Run robot code `./kist_controller` 



## Dependencies:

- Eigen - http://eigen.tuxfamily.org

- RBDL 2.6.0 - https://github.com/rbdl/rbdl
```
git clone https://github.com/rbdl/rbdl
cd rbdl
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -DRBDL_BUILD_ADDON_URDFREADER=ON ..
make all
make install
sudo ldconfig
```