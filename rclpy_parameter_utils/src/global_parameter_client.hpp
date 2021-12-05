/*
 * global_parameter_client.h
 *
 *  Created on: Dec 2, 2021
 *      Author: jnicho
 */

#ifndef INCLUDE_RCLPY_PARAMETER_UTILS_GLOBAL_PARAMETER_CLIENT_H_
#define INCLUDE_RCLPY_PARAMETER_UTILS_GLOBAL_PARAMETER_CLIENT_H_

#include <pybind11/pybind11.h>

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter.hpp>

#include "rclcpp_component.hpp"


namespace rclpy_parameter_utils
{

class GlobalParameterClient: RclCppComponent
{
public:
  GlobalParameterClient(const std::string& node_name, const std::string& node_namespace, int number_of_threads = 1);
  virtual ~GlobalParameterClient();

  void start();

  void stop();

  rcl_interfaces::msg::Parameter getParameter(const std::string& full_parameter_name, double timeout_secs = 0.5);

  pybind11::dict getParameters(const std::string& parameters_namespace, const std::vector<std::string>& parameter_names,
                               double timeout_secs = 0.5);

  void setParameter(const std::string &full_parameter_name, const rclcpp::ParameterValue& parameter_val, double timeout_secs = 0.5);

protected:

  rclcpp::SyncParametersClient createParameterClient(rclcpp::Node::SharedPtr node,
                                                     const std::string& remote_node_name,
                                                     double timeout_secs);

  std::vector<rclcpp::Parameter> getParametersImpl(const std::string& remote_node_name,
                                    const std::vector<std::string>& parameter_names,
                                    double timeout_secs);

  void setParametersImpl(const std::string& remote_node_name,
                         const std::vector<rclcpp::Parameter>& parameter_names,
                         double timeout_secs);


  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread executor_thread_;
};

} /* namespace rclpy */

#endif /* INCLUDE_RCLPY_PARAMETER_UTILS_GLOBAL_PARAMETER_CLIENT_H_ */
