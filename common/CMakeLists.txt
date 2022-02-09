# Source/Include files
include_directories(".")
include_directories("include/" )                 # common includes
include_directories("/usr/local/include/lcm/")   # lcm includes
# include_directories("../lcm-types/cpp/")
include_directories("../third-party/inih")       # inih includes
include_directories("../third-party/osqp/include")       # osqp includes
include_directories("../third-party/ParamHandler")
include_directories("../robot/include")
include_directories("/usr/include/eigen3")   # eigen includes
# include_directories("/usr/local/include/eigen3")   # eigen includes for robot computer
include_directories("FootstepPlanner")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
file(GLOB_RECURSE sources "src/*.cpp" "src/Controllers/*.cpp" "src/ControlParameters/*.cpp")                   # common library cpp files

# Library
add_library(biomimetics SHARED ${sources})       # produce a library used by sim/robot
target_link_libraries(biomimetics inih dynacore_param_handler JCQP osqp rbdl rbdl_urdfreader)


# add_test(NAME example_test COMMAND test-common)

# endif(CMAKE_SYSTEM_NAME MATCHES Linux)

# Our libraries
add_subdirectory(FootstepPlanner)

