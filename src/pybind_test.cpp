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

class Worker {
public:
    Worker() : is_running_(false) {} 

    void start_work(int duration) {
        if (!is_running_) {
            is_running_ = true;
            thread_ = std::thread(&Worker::do_work, this, duration);
        }
    }

    void stop_work() {
        if (is_running_) {
            is_running_ = false;
            if (thread_.joinable()) {
                thread_.join(); 
            }
        }
    }

private:
    void do_work(int duration) {
        std::this_thread::sleep_for(std::chrono::seconds(duration));
        is_running_ = false;
    }

    std::thread thread_;
    bool is_running_;
};

PYBIND11_MODULE(pybind_test, m) {
    pybind11::class_<Worker>(m, "Worker")
        .def(pybind11::init<>())
        .def("start_work", &Worker::start_work)
        .def("stop_work", &Worker::stop_work);
}
