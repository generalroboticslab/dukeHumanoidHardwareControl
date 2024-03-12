
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h> // For passing NumPy arrays
#include "ethercat_motor.h"

namespace py = pybind11;

class PyMotor : public Motor
{
public:
   void run_in_background()
   {
      pybind11::gil_scoped_release release;
      thread2 = std::thread(&Motor::run, this);
      thread2.detach();
   }
   // for the inherited function you have to specify the constructor again for pybind11 :(
   PyMotor(const std::string &ifname, int8_t control_mode_int8, double max_velocity, double max_torque)
   {
      _init(ifname, control_mode_int8, max_velocity, max_torque);
   }
};

PYBIND11_MODULE(ethercat_motor_py, m)
{
   py::class_<PyMotor>(m, "PyMotor", py::dynamic_attr())
       .def(py::init<const std::string &, int8_t, double, double>())
       .def(py::init<const std::string &, int8_t, double, double>())
       .def("set_should_terminate", &Motor::set_should_terminate)
       .def_readonly("target_int32", &Motor::target_int32) // Bind the run function
       .def_readwrite("control_mode_int8", &Motor::control_mode_int8)
       .def_readwrite("should_print", &Motor::should_print)
       .def("set_target_input", &Motor::set_target_input)
       .def("set_target_offset", &Motor::set_target_offset)
       .def("run", &PyMotor::run_in_background);
}