project(unit_test)

# mhpc test
file(GLOB_RECURSE sources "../Controller/MHPCLocomotion/*.cpp"
						  "../Controller/Heuristics/*.cpp"
						  "../Utils/*.cpp")

add_executable(mhpc_test test_main.cpp ${sources})

target_link_libraries(mhpc_test casadigen dynamics hsddp)
target_link_libraries(mhpc_test robot biomimetics)

# Casadigen test
file(GLOB_RECURSE casadi_sources "../CasadiGen/source/*.cpp" "../CasadiGen/source/*.c")
add_executable(casadigen_test casadigen_test.cpp ${casadi_sources})
target_include_directories(casadigen_test PUBLIC "../CasadiGen/header")
