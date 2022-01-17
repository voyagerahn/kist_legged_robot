# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kist/templorary_kist_project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kist/templorary_kist_project/build

# Include any dependencies generated for this target.
include robot/CMakeFiles/robot.dir/depend.make

# Include the progress variables for this target.
include robot/CMakeFiles/robot.dir/progress.make

# Include the compile flags for this target's objects.
include robot/CMakeFiles/robot.dir/flags.make

robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o: robot/CMakeFiles/robot.dir/flags.make
robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o: ../robot/src/KIST_Controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist/templorary_kist_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o"
	cd /home/kist/templorary_kist_project/build/robot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robot.dir/src/KIST_Controller.cpp.o -c /home/kist/templorary_kist_project/robot/src/KIST_Controller.cpp

robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/src/KIST_Controller.cpp.i"
	cd /home/kist/templorary_kist_project/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kist/templorary_kist_project/robot/src/KIST_Controller.cpp > CMakeFiles/robot.dir/src/KIST_Controller.cpp.i

robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/src/KIST_Controller.cpp.s"
	cd /home/kist/templorary_kist_project/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kist/templorary_kist_project/robot/src/KIST_Controller.cpp -o CMakeFiles/robot.dir/src/KIST_Controller.cpp.s

robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o.requires:

.PHONY : robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o.requires

robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o.provides: robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o.requires
	$(MAKE) -f robot/CMakeFiles/robot.dir/build.make robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o.provides.build
.PHONY : robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o.provides

robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o.provides.build: robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o


robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o: robot/CMakeFiles/robot.dir/flags.make
robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o: ../robot/src/RobotRunner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist/templorary_kist_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o"
	cd /home/kist/templorary_kist_project/build/robot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robot.dir/src/RobotRunner.cpp.o -c /home/kist/templorary_kist_project/robot/src/RobotRunner.cpp

robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/src/RobotRunner.cpp.i"
	cd /home/kist/templorary_kist_project/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kist/templorary_kist_project/robot/src/RobotRunner.cpp > CMakeFiles/robot.dir/src/RobotRunner.cpp.i

robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/src/RobotRunner.cpp.s"
	cd /home/kist/templorary_kist_project/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kist/templorary_kist_project/robot/src/RobotRunner.cpp -o CMakeFiles/robot.dir/src/RobotRunner.cpp.s

robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o.requires:

.PHONY : robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o.requires

robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o.provides: robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o.requires
	$(MAKE) -f robot/CMakeFiles/robot.dir/build.make robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o.provides.build
.PHONY : robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o.provides

robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o.provides.build: robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o


robot/CMakeFiles/robot.dir/src/main.cpp.o: robot/CMakeFiles/robot.dir/flags.make
robot/CMakeFiles/robot.dir/src/main.cpp.o: ../robot/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist/templorary_kist_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object robot/CMakeFiles/robot.dir/src/main.cpp.o"
	cd /home/kist/templorary_kist_project/build/robot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robot.dir/src/main.cpp.o -c /home/kist/templorary_kist_project/robot/src/main.cpp

robot/CMakeFiles/robot.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/src/main.cpp.i"
	cd /home/kist/templorary_kist_project/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kist/templorary_kist_project/robot/src/main.cpp > CMakeFiles/robot.dir/src/main.cpp.i

robot/CMakeFiles/robot.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/src/main.cpp.s"
	cd /home/kist/templorary_kist_project/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kist/templorary_kist_project/robot/src/main.cpp -o CMakeFiles/robot.dir/src/main.cpp.s

robot/CMakeFiles/robot.dir/src/main.cpp.o.requires:

.PHONY : robot/CMakeFiles/robot.dir/src/main.cpp.o.requires

robot/CMakeFiles/robot.dir/src/main.cpp.o.provides: robot/CMakeFiles/robot.dir/src/main.cpp.o.requires
	$(MAKE) -f robot/CMakeFiles/robot.dir/build.make robot/CMakeFiles/robot.dir/src/main.cpp.o.provides.build
.PHONY : robot/CMakeFiles/robot.dir/src/main.cpp.o.provides

robot/CMakeFiles/robot.dir/src/main.cpp.o.provides.build: robot/CMakeFiles/robot.dir/src/main.cpp.o


# Object files for target robot
robot_OBJECTS = \
"CMakeFiles/robot.dir/src/KIST_Controller.cpp.o" \
"CMakeFiles/robot.dir/src/RobotRunner.cpp.o" \
"CMakeFiles/robot.dir/src/main.cpp.o"

# External object files for target robot
robot_EXTERNAL_OBJECTS =

robot/librobot.so: robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o
robot/librobot.so: robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o
robot/librobot.so: robot/CMakeFiles/robot.dir/src/main.cpp.o
robot/librobot.so: robot/CMakeFiles/robot.dir/build.make
robot/librobot.so: common/libbiomimetics.so
robot/librobot.so: third-party/inih/libinih.so
robot/librobot.so: third-party/ParamHandler/libdynacore_param_handler.so
robot/librobot.so: third-party/ParamHandler/libdynacore_yaml-cpp.so.0.5.3
robot/librobot.so: third-party/JCQP/libJCQP.so
robot/librobot.so: third-party/osqp/out/libosqp.so
robot/librobot.so: robot/CMakeFiles/robot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kist/templorary_kist_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library librobot.so"
	cd /home/kist/templorary_kist_project/build/robot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
robot/CMakeFiles/robot.dir/build: robot/librobot.so

.PHONY : robot/CMakeFiles/robot.dir/build

robot/CMakeFiles/robot.dir/requires: robot/CMakeFiles/robot.dir/src/KIST_Controller.cpp.o.requires
robot/CMakeFiles/robot.dir/requires: robot/CMakeFiles/robot.dir/src/RobotRunner.cpp.o.requires
robot/CMakeFiles/robot.dir/requires: robot/CMakeFiles/robot.dir/src/main.cpp.o.requires

.PHONY : robot/CMakeFiles/robot.dir/requires

robot/CMakeFiles/robot.dir/clean:
	cd /home/kist/templorary_kist_project/build/robot && $(CMAKE_COMMAND) -P CMakeFiles/robot.dir/cmake_clean.cmake
.PHONY : robot/CMakeFiles/robot.dir/clean

robot/CMakeFiles/robot.dir/depend:
	cd /home/kist/templorary_kist_project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kist/templorary_kist_project /home/kist/templorary_kist_project/robot /home/kist/templorary_kist_project/build /home/kist/templorary_kist_project/build/robot /home/kist/templorary_kist_project/build/robot/CMakeFiles/robot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : robot/CMakeFiles/robot.dir/depend

