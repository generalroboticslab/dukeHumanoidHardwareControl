
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h> // For passing NumPy arrays
#include "ethercat_motor.h"

namespace py = pybind11;
PYBIND11_MODULE(eathercat_motor_py, m)
{
   py::class_<Motor>(m, "Motor")
       //   .def(py::init<const std::string&, int8_t, double*, double>()) // Expose constructor
       .def(py::init())
       // ... expose other methods of your Motor class
       .def("run", &Motor::run); // Bind the run function
   ;
   m.def("do_work_in_thread", []()
         {
            double target_input_in[] = {0.5,0.5};
            double max_velocity = 0.5;
            Motor motor("enp3s0", (int8_t)CONTROL_MODE::CYCLIC_SYNC_VELOCITY, target_input_in, max_velocity);
            motor._run();
            
            // std::thread t(&Motor::_run, motor);
            // t.detach(); // Let the thread run in the background
         });
}