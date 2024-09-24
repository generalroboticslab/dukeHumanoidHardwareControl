
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h> // For passing NumPy arrays
#include <pybind11/eigen.h>

#include "ethercat_motor.h"

namespace py = pybind11;

class PyMotor : public Motor
{
public:
   // tx_pdo_t tx_pdo_example{};

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

   py::enum_<CONTROL_MODE>(m, "CONTROL_MODE", py::arithmetic())
       .value("CYCLIC_SYNC_POSITION", CONTROL_MODE::CYCLIC_SYNC_POSITION)
       .value("CYCLIC_SYNC_VELOCITY", CONTROL_MODE::CYCLIC_SYNC_VELOCITY)
       .value("CYCLIC_SYNC_TORQUE", CONTROL_MODE::CYCLIC_SYNC_TORQUE)
       .export_values();

   py::class_<tx_pdo_t>(m, "tx_pdo_t")
       .def(py::init([]()
                     { return tx_pdo_t(); }))
       .def_readwrite("target_position", &tx_pdo_t::target_position)
       .def_readwrite("target_velocity", &tx_pdo_t::target_velocity)
       .def_readwrite("velocity_offset", &tx_pdo_t::velocity_offset)
       .def_readwrite("max_torque", &tx_pdo_t::max_torque)
       .def_readwrite("target_torque_raw", &tx_pdo_t::target_torque)
       .def_readwrite("torque_offset", &tx_pdo_t::torque_offset)
       .def_readwrite("control_word", &tx_pdo_t::control_word)
       .def_readwrite("mode_of_operation", &tx_pdo_t::mode_of_operation)
       .def("astuple",
            [](const tx_pdo_t &self)
            {
               return py::make_tuple(
                   self.target_position,
                   self.target_velocity,
                   self.velocity_offset,
                   self.max_torque,
                   self.target_torque,
                   self.torque_offset,
                   self.control_word,
                   self.mode_of_operation);
            })
       .def_static("fromtuple", [](const py::tuple &tup)
                   { if (py::len(tup) != 8) {throw py::cast_error("Invalid size");}
                        return tx_pdo_t{
                           tup[0].cast<int32>(),
                           tup[1].cast<int32>(),
                           tup[2].cast<int32>(),
                           tup[3].cast<int16>(),
                           tup[4].cast<int16>(),
                           tup[5].cast<int16>(),
                           tup[6].cast<uint16>(),
                           tup[7].cast<uint8>()}; });

   PYBIND11_NUMPY_DTYPE(tx_pdo_t,
                        target_position,
                        target_velocity,
                        velocity_offset,
                        max_torque,
                        target_torque,
                        torque_offset,
                        control_word,
                        mode_of_operation);

   py::class_<rx_pdo_t>(m, "rx_pdo_t")
       .def(py::init([]()
                     { return rx_pdo_t(); }))
       .def_readwrite("position_actual", &rx_pdo_t::position_actual)
       .def_readwrite("position_follow_err", &rx_pdo_t::position_follow_err)
       .def_readwrite("velocity_actual", &rx_pdo_t::velocity_actual)
       .def_readwrite("torque_actual", &rx_pdo_t::torque_actual)
       .def_readwrite("status_word", &rx_pdo_t::status_word)
       .def_readwrite("mode_of_operation_disp", &rx_pdo_t::mode_of_operation_disp)
       .def("astuple",
            [](const rx_pdo_t &self)
            {
               return py::make_tuple(
                   self.position_actual,
                   self.position_follow_err,
                   self.velocity_actual,
                   self.torque_actual,
                   self.status_word,
                   self.mode_of_operation_disp);
            })
       .def_static("fromtuple", [](const py::tuple &tup)
                   { if (py::len(tup) != 6) {throw py::cast_error("Invalid size");}
                        return rx_pdo_t{
                           tup[0].cast<int32>(),
                           tup[1].cast<int32>(),
                           tup[2].cast<int32>(),
                           tup[3].cast<int16>(),
                           tup[4].cast<uint16>(),
                           tup[5].cast<uint8>()}; });

   PYBIND11_NUMPY_DTYPE(rx_pdo_t,
                        position_actual,
                        position_follow_err,
                        velocity_actual,
                        torque_actual,
                        status_word,
                        mode_of_operation_disp);

   py::class_<PyMotor>(m, "PyMotor", py::dynamic_attr())
      .def(py::init<const std::string &, int8_t, double, double>())
      .def("set_should_terminate", &PyMotor::set_should_terminate)
      //  .def_readonly("target_int32", &PyMotor::target_int32) // Bind the run function
      .def_readwrite("control_mode_int8", &PyMotor::control_mode_int8)
      .def_readwrite("should_print", &PyMotor::should_print)
      .def_readwrite("debug", &PyMotor::debug)
      .def_readonly("rated_torque", &PyMotor::rated_torque)
      .def_readonly("actual_position", &PyMotor::actual_position)
      .def_readonly("actual_position_error", &PyMotor::actual_position_error)
      .def_readonly("actual_velocity", &PyMotor::actual_velocity)
      .def_readonly("actual_velocity_filtered", &PyMotor::actual_velocity_filtered)
      .def_readonly("actual_torque_raw", &PyMotor::actual_torque_raw)
      .def_readonly("actual_torque", &PyMotor::actual_torque)
      .def_readwrite("use_position_pd", &PyMotor::use_position_pd)
      .def_readwrite("rotor_current_to_torque_ratio", &PyMotor::rotor_current_to_torque_ratio)
      .def_readwrite("kp", &PyMotor::kp)
      .def_readwrite("kd", &PyMotor::kd)
      .def_readonly("dof_position", &PyMotor::dof_position)
      .def_readonly("dof_velocity", &PyMotor::dof_velocity)
      .def_readwrite("target_dof_position", &PyMotor::target_dof_position)
      .def_readonly("target_dof_torque_Nm", &PyMotor::target_dof_torque_Nm)
      .def_readwrite("torque_multiplier", &PyMotor::torque_multiplier)
      .def_readonly("target_dof_torque_Nm_filtered", &PyMotor::target_dof_torque_Nm_filtered)

      .def_readonly("actual_position_after_offset", &PyMotor::actual_position_after_offset)
      .def_readonly("position_offset", &PyMotor::position_offset)
      .def_readonly("dt_measured", &PyMotor::dt_measured)
      // .def_readonly("actual_velocity_measured", &PyMotor::actual_velocity_measured)

      .def("set_target_input", &PyMotor::set_target_input)
      .def("set_target_position", &PyMotor::set_target_position,py::arg().noconvert())
      .def("set_target_velocity", &PyMotor::set_target_velocity,py::arg().noconvert())
      .def("set_target_torque", &PyMotor::set_target_torque,py::arg().noconvert())
      .def("set_target_position_offset", &PyMotor::set_target_position_offset,py::arg().noconvert())
      .def("set_velocity_offset", &PyMotor::set_velocity_offset,py::arg().noconvert())
      .def("set_torque_offset", &PyMotor::set_torque_offset,py::arg().noconvert())
      .def("set_max_torque", &PyMotor::set_max_torque,py::arg().noconvert())
       .def("set_position_offset", &PyMotor::set_position_offset,py::arg().noconvert())
      .def("run", &PyMotor::run_in_background,R"pbdoc(Starts the motor control loop in a background thread.)pbdoc");
}