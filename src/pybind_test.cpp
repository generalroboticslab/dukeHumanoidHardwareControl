#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <atomic>

#include <pybind11/eigen.h>
#include <Eigen/Dense>

namespace py = pybind11;

#define NUM_TARGET 6 // num of motors

using MatrixXdR = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

class BackgroundWorker
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::VectorXd eigen_vec = Eigen::VectorXd::Zero(NUM_TARGET);  // Declare and initialize in one line
    // Vec12<double> eigen_vec;

    std::chrono::system_clock::time_point t;
    std::vector<double> data_vector{1, 2, 3};

    BackgroundWorker() = default; // Default constructor
    BackgroundWorker(const std::string &ifname_, int8_t control_mode_int8, double max_velocity, double max_torque)
    {
        std::cout << "BackgroundWorker(int k)" << std::endl;
        std::cout << "ifname_:" << ifname_ << std::endl;
        std::cout << "control_mode_int8:" << (int)control_mode_int8 << std::endl;
        std::cout << "max_velocity:" << max_velocity << std::endl;
        std::cout << "max_torque:" << max_torque << std::endl;
    }

    void start_background_thread(int duration)
    {
        py::gil_scoped_release release;
        worker_thread_ = std::thread(&BackgroundWorker::background_worker, this, duration);
        worker_thread_.detach();
    }

    void set_should_terminate(bool value)
    {
        should_terminate = value;
        std::cout << "set_should_terminate()" << std::endl;
    }
    void change_data_vector(const py::array_t<double> &a)
    {
        for (int i = 0; i < 3; i++)
        {
            data_vector[i] = a.at(i);
        }
    }

private:
    void background_worker(int duration)
    {
        for (int i = 0; i < duration && !should_terminate; i++)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            t = std::chrono::system_clock::now();
            eigen_vec(i%NUM_TARGET)=static_cast<double>(std::chrono::system_clock::to_time_t(t));
            data_vector[2] = i;
            std::cout << "Background thread running: " << i << std::endl;
        }
        std::cout << "Background thread terminating..." << std::endl;
    }

    std::atomic<bool> should_terminate{false};
    std::thread worker_thread_;
};

PYBIND11_MODULE(pybind_test, m)
{
    py::class_<BackgroundWorker>(m, "BackgroundWorker", py::dynamic_attr())
        .def(py::init<>()) // Expose the constructor
        .def(py::init<const std::string &, int8_t,double,double>())
        .def_readonly("t", &BackgroundWorker::t)
        .def_readwrite("data_vector", &BackgroundWorker::data_vector)
        .def("change_data_vector", &BackgroundWorker::change_data_vector)
        .def("start_background_thread", &BackgroundWorker::start_background_thread)
        .def("set_should_terminate", &BackgroundWorker::set_should_terminate)
        .def_readwrite("eigen_vec", &BackgroundWorker::eigen_vec, py::return_value_policy::reference);
}

// #include <pybind11/pybind11.h>

// namespace py = pybind11;

// class MyClass {
// public:
//     MyClass(const std::string& name) : name_(name) {}

//     std::string get_name() const { return name_; }

// private:
//     std::string name_;
// };

// PYBIND11_MODULE(mymodule, m) {
//   py::class_<MyClass>(m, "MyClass")
//       .def(py::init<const std::string&>())  // Constructor with std::string argument
//       .def("get_name", &MyClass::get_name);
// }