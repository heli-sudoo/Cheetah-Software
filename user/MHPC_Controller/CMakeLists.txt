include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")

add_executable(MHPC_ctrl main.cpp MHPC_Controller.cpp)
target_link_libraries(MHPC_ctrl robot biomimetics dynacore_param_handler qpOASES)
