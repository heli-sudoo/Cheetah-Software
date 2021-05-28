file(GLOB_RECURSE sources "source/*.c" "source/*.cpp")

add_library(casadigen SHARED ${sources})
target_include_directories(casadigen PUBLIC "header/")
    
