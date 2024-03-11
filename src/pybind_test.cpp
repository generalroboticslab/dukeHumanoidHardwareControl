// #include <pybind11/pybind11.h>

// int add(int i, int j) {
//     return i + j;
// }

// namespace py = pybind11;
// using namespace py;
// PYBIND11_MODULE(pybind_test, m) { //name has to match the name of the .cpp file
//     m.doc() = "Example pybind11 module";
//     m.def("add", &add, "A function that adds two numbers");
// }

#include <pybind11/pybind11.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <csignal>
#include <atomic>

namespace py = pybind11;

volatile sig_atomic_t should_terminate_{false}; // Atomic for thread safety
void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        should_terminate_ = true;
    }
}

class Worker
{
public:
    Worker() : is_running_(false) {}
    ~Worker() { std::cout << "deconstruction called\n"; }
    void start_work(int duration)
    {
        if (!is_running_)
        {
            is_running_ = true;
            thread_ = std::thread(&Worker::do_work, this, duration);
        }
    }

    void stop_work()
    {
        if (is_running_)
        {
            std::cout << "stop_work" << std::endl;

            should_terminate_ = true; // Set the termination flag
            if (thread_.joinable())
            {
                thread_.join();
            }
            is_running_ = false; // Reset running state after termination
        }
    }

private:
    void do_work(int duration)
    {
        // py::gil_scoped_release release;
        std::signal(SIGINT, signal_handler); // Pass the static function

        for (int i = 0; i < duration && !should_terminate_; i++)
        {
            py::gil_scoped_release release; // Temporarily release GIL
            // Check flag
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << "do_work: " << i + 1 << std::endl;

            py::gil_scoped_acquire acquire; // Reacquire GIL
        }
        is_running_ = false;

        // py::gil_scoped_acquire acquire;
    }

    std::thread thread_;
    bool is_running_;
};

PYBIND11_MODULE(pybind_test, m)
{ // Module name must match your CMake file
    m.doc() = "Example pybind11 module for the Worker class";
    py::class_<Worker, std::unique_ptr<Worker, py::nodelete>>(m, "Worker") // Bind the Worker class
        .def(py::init<>())                                                 // Constructor
        .def("start_work", &Worker::start_work)
        .def("stop_work", &Worker::stop_work);
}