#include <Eigen/Core>
#include <iostream>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>

//https://stackoverflow.com/questions/77305738/nanobind-fails-converting-complex-np-array-to-eigen-complex-vector
NB_MODULE(nanobind_test, m) {
  m.def("hello_world", []() { std::cout << "hello world\n"; });
  m.def("print_double_vector", [](Eigen::Vector2d &vec) {
    std::cout << "Double: " << vec.transpose() << std::endl;
  });
  m.def("print_complex_vector", [](Eigen::Vector2cd &vec) {
    std::cout << "Complex: " << vec.transpose() << std::endl;
  });
}
