/*
 * rclpy_pybind11.cpp
 *
 *  Created on: Dec 4, 2021
 *      Author: jnicho
 */


#include <pybind11/pybind11.h>

#include "global_parameter_client.hpp"

namespace rclpy_parameter_utils
{
namespace py = pybind11;

  PYBIND11_MODULE(rclpy_parameter_utils, m)
  {
    m.doc() = "rclpy_parameter_utils library for support with global parameters";
    py::class_<GlobalParameterClient >(m,"GlobalParameterClient")
        .def(py::init<const std::string&, const std::string&, int>(),"GlobalParameterClient constructor",
             py::arg("node_name"), py::arg("node_namespace"), py::arg("number_of_threads") = 1)
        .def("start", &GlobalParameterClient::start)
        .def("get_parameter",&GlobalParameterClient::getParameter)
        .def("get_parameters", &GlobalParameterClient::getParameters)
        .def("set_parameter", &GlobalParameterClient::setParameter);
  }
}
