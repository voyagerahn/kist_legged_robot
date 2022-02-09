include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("include/")
include_directories("./")
include_directories("../common/include/")
include_directories("../third-party")
include_directories("../third-party/ParamHandler")
include_directories("lib")
include_directories("../../common/include/")
include_directories("../../common/FootstepPlanner")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/include/eigen3")   # eigen includes
# include_directories("/usr/local/include/eigen3")   # eigen includes for robot computer
add_subdirectory(src/Controller/WBC)
add_subdirectory(src/Controller/WBC_Ctrl)

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp" 
)

link_directories(lib)
link_directories(./lib)
add_library(robot SHARED ${sources})
target_link_libraries(robot biomimetics pthread lcm inih dynacore_param_handler libunitree_legged_sdk_amd64.so rbdl rbdl_urdfreader)

set(CMAKE_CXX_FLAGS "-O3")

add_executable(kist_controller ${sources} src/KIST_Controller.cpp src/main.cpp)
target_link_libraries(kist_controller robot biomimetics libunitree_legged_sdk_amd64.so)
target_link_libraries(kist_controller dynacore_param_handler qpOASES)
target_link_libraries(kist_controller Goldfarb_Optimizer osqp)
target_link_libraries(kist_controller WBC_Ctrl)



# include_directories("./Controllers")

# # file(GLOB_RECURSE sources
# # "./*.cpp"
# # "FSM_States/*.cpp" 
# # "Controllers/BalanceController/*.cpp" 
# # "Controllers/convexMPC/*.cpp")
# file(GLOB_RECURSE sources "./*.cpp")

# # add_subdirectory(Controllers/WBC)
# # add_subdirectory(Controllers/WBC_Ctrl)
# # add_subdirectory(Controllers/VisionMPC)

# add_executable(kist_controller ${sources} KIST_Controller.cpp main.cpp)
# target_link_libraries(kist_controller robot biomimetics libunitree_legged_sdk_amd64.so)
# target_link_libraries(kist_controller dynacore_param_handler qpOASES)
# target_link_libraries(kist_controller Goldfarb_Optimizer osqp)
# # target_link_libraries(kist_cotroller WBC_Ctrl)
# # target_link_libraries(kist_cotroller VisionMPC)
