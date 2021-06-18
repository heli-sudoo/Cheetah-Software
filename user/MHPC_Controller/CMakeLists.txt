cmake_minimum_required(VERSION 3.10)

project(MHPCCTRL)

# set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}\
					 -Wno-unused -Wno-error=unused\
					 -Wno-return-type -Wno-error=return-type\
					 -Wno-sign-compare -Wno-error=sign-compare\
					 -Wno-switch -Wno-error=switch\
					 -Wno-reorder -Wno-error=reorder")					 
					 
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}\
					 -Wno-unused -Wno-error=unused\
					 -Wno-return-type -Wno-error=return-type\
					 -Wno-sign-compare -Wno-error=sign-compare\
					 -Wno-switch -Wno-error=switch")

include_directories(${CMAKE_SOURCE_DIR})
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


file(GLOB_RECURSE sources "Utils/*.cpp"
						  "Controller/Heuristics/*.cpp"
						  "Controller/MHPCLocomotion/*.cpp"
						  "FSM_States/source/*.cpp")

add_executable(mhpc_ctrl main.cpp MHPC_Controller.cpp ${sources})
target_link_libraries(mhpc_ctrl robot biomimetics)
target_link_libraries(mhpc_ctrl casadigen dynamics hsddp)
target_link_libraries(mhpc_ctrl dynacore_param_handler)


# cmake test for debugging
add_subdirectory(test)


