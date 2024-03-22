cmake_minimum_required(VERSION 3.10 )  # Adjust version as needed

project(biped 
    LANGUAGES C CXX)
set (CMAKE_CXX_STANDARD 17)

# # Enable compiler caching
# set(CMAKE_CXX_COMPILER_LAUNCHER ccache)

# Use Ninja for faster builds
# set(CMAKE_GENERATOR ninja)
## cmake -G Ninja

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON) 
 
# need to find with python3 -c "from distutils import sysconfig; print(sysconfig.get_python_inc())"
# set(Python_EXECUTABLE "/home/grl/repo/micromamba/envs/py38/bin/python")
# set(Python3_INCLUDE_DIRS "/home/grl/repo/micromamba/envs/py38/include/python3.8/")
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/__init__.py") # Creates an empty file

# set(CMAKE_TOOLCHAIN_FILE "/home/grl/repo/vcpkg/scripts/buildsystems/vcpkg.cmake")
# cmake .. "-DCMAKE_TOOLCHAIN_FILE=/home/grl/repo/vcpkg/scripts/buildsystems/vcpkg.cmake"

find_package(msgpack-cxx CONFIG REQUIRED)
# target_link_libraries(main PRIVATE msgpack-cxx)

find_package(asio CONFIG REQUIRED)
# target_link_libraries(main PRIVATE asio::asio)

find_package(Eigen3 CONFIG REQUIRED)
# target_link_libraries(main PRIVATE Eigen3::Eigen)
# target_include_directories(main PRIVATE ${EIGEN3_INCLUDE_DIR})


find_package(pybind11 REQUIRED)
find_package(Threads REQUIRED)

# Include SOEM source files into your build 
add_subdirectory(SOEM)  

add_subdirectory(SOEM/test/linux/slaveinfo) # build slaveinfo

# Create your executable
add_executable(simple_test 
            src/simple_test.cpp
            src/ethercat_motor.h
            # src/ethercat_motor.cpp
            # src/network.cpp
            # src/network.h
            )
# target_link_libraries(simple_test PRIVATE msgpack-cxx) 
target_link_libraries(simple_test PRIVATE soem)  # 'soem' is the target created by add_subdirectory
target_include_directories(simple_test PRIVATE SOEM)

# target_link_libraries(simple_test PRIVATE asio::asio)
# target_link_libraries(simple_test PRIVATE pybind11::module)
# target_include_directories(simple_test PRIVATE ${pybind11_INCLUDE_DIRS})
target_link_libraries(simple_test PRIVATE Eigen3::Eigen)
target_include_directories(simple_test PRIVATE ${EIGEN3_INCLUDE_DIR})


pybind11_add_module(ethercat_motor_py 
    src/ethercat_motor_py.cpp
    src/ethercat_motor.h
    # src/ethercat_motor.cpp
)
# target_link_libraries(ethercat_motor_py PRIVATE msgpack-cxx) 
target_link_libraries(ethercat_motor_py PRIVATE soem) 
target_include_directories(ethercat_motor_py PRIVATE SOEM)
target_link_libraries(ethercat_motor_py PRIVATE pybind11::module)
target_include_directories(ethercat_motor_py PRIVATE ${pybind11_INCLUDE_DIRS})
target_link_libraries(ethercat_motor_py PRIVATE Eigen3::Eigen)
target_include_directories(ethercat_motor_py PRIVATE ${EIGEN3_INCLUDE_DIR})

pybind11_add_module(pybind_test src/pybind_test.cpp)
# add_executable(pybind_test src/pybind_test.cpp)
target_link_libraries(pybind_test PRIVATE pybind11::module)
target_include_directories(pybind_test PRIVATE ${pybind11_INCLUDE_DIRS})
target_link_libraries(pybind_test PRIVATE Eigen3::Eigen)
target_include_directories(pybind_test PRIVATE ${EIGEN3_INCLUDE_DIR})


add_executable(msgpack_test src/msgpack_test/msgpack_test.cpp)
target_link_libraries(msgpack_test PRIVATE msgpack-cxx) 

add_executable(msgpack_test_sender src/msgpack_test/sender.cpp src/msgpack_test/person.h)
target_link_libraries(msgpack_test_sender PRIVATE msgpack-cxx) 

add_executable(msgpack_test_receiver src/msgpack_test/receiver.cpp src/msgpack_test/person.h)
target_link_libraries(msgpack_test_receiver PRIVATE msgpack-cxx) 