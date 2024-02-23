cmake_minimum_required(VERSION 3.10 )  # Adjust version as needed

project(biped 
    LANGUAGES C CXX)

# Include SOEM source files into your build 
add_subdirectory(SOEM)  

# Create your executable
add_executable(simple_test src/simple_test.c)

# Link your executable against the SOEM library
target_link_libraries(simple_test PRIVATE soem)  # 'soem' is the target created by add_subdirectory
