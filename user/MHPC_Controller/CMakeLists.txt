cmake_minimum_required(VERSION 3.10)

project(MHPCCTRL)

# set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# disable some warninigs that have been verified having no problems
set(CMAKE_CXX_FLAGS "-Wno-unused-parameter -Wno-return-type")

include_directories(${CMAKE_SOURCE_DIR}) # directory where toppest CMakeLists.txt is located
include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")
include_directories("Common/header")

include_directories("Utils")
include_directories("Controller/Heuristics")
include_directories("Controller/MHPCLocomotion")
include_directories("FSM_States/header")

add_subdirectory(CasadiGen)
add_subdirectory(Dynamics)
add_subdirectory(HSDDPSolver)

file(GLOB_RECURSE sources "FSM_States/source/*.cpp"
						  "Controller/MHPCLocomotion/*.cpp"
						  "Controller/Heuristics/*.cpp"
						  "Utils/*.cpp")

add_executable(mhpc_ctrl ${sources} main.cpp MHPC_Controller.cpp)

target_link_libraries(mhpc_ctrl casadigen dynamics hsddp)
target_link_libraries(mhpc_ctrl robot biomimetics)
target_link_libraries(mhpc_ctrl dynacore_param_handler)



