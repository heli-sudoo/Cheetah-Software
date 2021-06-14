project(mhpc_test)

file(GLOB_RECURSE sources "../Controller/MHPCLocomotion/*.cpp"
						  "../Controller/Heuristics/*.cpp"
						  "../Utils/*.cpp")

add_executable(mhpc_test test_main.cpp ${sources})

target_link_libraries(mhpc_test casadigen dynamics hsddp)
target_link_libraries(mhpc_test robot biomimetics)
