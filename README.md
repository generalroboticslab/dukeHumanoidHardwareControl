
## Project structure



```
├── biped_controller.py
├── build # build folder
│   ├── ethercat_motor_py.cpython-38-x86_64-linux-gnu.so
│   ├── simple_test
│   └── ...
├── checkpoint
│   ├── biped
│   └── ...
├── CMakeLists.txt
├── ext
│   ├── nanobind
│   └── SOEM
├── matchIsaacToRobot.py
├── motor_controller.py
├── numpy_ringbuffer.py
├── publisher.py
├── README.md
├── run_joint_monkey.sh
├── sensor_controller.py
├── src
│   ├── ethercat_motor.cpp
│   ├── ethercat_motor.h
│   ├── ethercat_motor_py.cpp
│   ├── filter_test.cpp
│   ├── motor_mass_current.txt
│   ├── msgpack_test
│   ├── nanobind_test.cpp
│   ├── network.cpp
│   ├── network.h
│   ├── pybind_test.cpp
│   ├── Readme.md
│   ├── simple_test.c
│   ├── simple_test.cpp
│   └── tmp.c
├── sshkeyboard_pd_test.py
├── teensy_comm
│   └── teensy_comm.ino
├── trajectory_PD_test.py
```


## vcpkg

follow the instructions in: 
https://learn.microsoft.com/en-us/vcpkg/get_started/get-started?pivots=shell-bash

```
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg && ./bootstrap-vcpkg.sh



```
./vcpkg install msgpack asio eigen3 iir1
```
export VCPKG_ROOT=/path/to/vcpkg
export PATH=$VCPKG_ROOT:$PATH
```


```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=~/repo/micromamba/envs/py38 -DPython_EXECUTABLE=~/repo/micromamba/envs/py38/bin/python -DCMAKE_TOOLCHAIN_FILE=~/repo/vcpkg/scripts/buildsystems/vcpkg.cmake -G Ninja

```

# Run

```
## validate sensors
sudo chrt -f 99 $(which python) -u sensor_controller.py

## initialize motor
sudo chrt -f 99 $(which python) -u motor_controller.py

## run biped control
sudo chrt -f 99 $(which python) -u biped_controller.py

```