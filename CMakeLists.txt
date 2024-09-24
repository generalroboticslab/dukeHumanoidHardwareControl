cmake_minimum_required(VERSION 3.10 )  # Adjust version as needed

# if(POLICY CMP0074) # needed for nanobind
#   cmake_policy(SET CMP0074 NEW)
# endif()


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
set(Python_EXECUTABLE "/home/grl/repo/micromamba/envs/py38/bin/python")
# set(Python3_INCLUDE_DIRS "/home/grl/repo/micromamba/envs/py38/include/python3.8/")
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

# #configure CMake to perform an optimized release build by default unless another build type is specified. 
# # Without this addition, binding code may run slowly and produce large binaries.
# if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
#   set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
#   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
# endif()

# # Detect the installed nanobind package and import it into CMake
# execute_process(
#   COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
#   OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
# find_package(nanobind CONFIG REQUIRED)


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

# # Detect the installed nanobind package and import it into CMake
# execute_process(
#   COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
#   OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
# find_package(nanobind CONFIG REQUIRED)

find_package(Threads REQUIRED)

# find_package(iir REQUIRED) # filters
find_package(iir REQUIRED)

# Include SOEM source files into your build 
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/SOEM)  
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/SOEM/test/linux/slaveinfo) # build slaveinfo

# # # nanobind
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/nanobind)


if (MSVC)
    add_compile_options(/W4)
  else()
    add_compile_options(-Wall)
endif()

# Create your executable

# nanobind_add_module(nanobind_test src/nanobind_test.cpp)
# target_include_directories(nanobind_test PUBLIC ${EIGEN3_INCLUDE_DIR})
# target_link_libraries(nanobind_test PUBLIC Eigen3::Eigen)



add_executable(filter_test 
            src/filter_test.cpp)
# target_link_libraries(filter_test PRIVATE irr)
target_link_libraries(filter_test PRIVATE iir::iir_static)
# target_compile_features(filter_test PRIVATE cxx_std_11)


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
target_link_libraries(simple_test PRIVATE iir::iir_static)

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
target_link_libraries(ethercat_motor_py PRIVATE iir::iir_static)




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