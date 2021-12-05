/*
 * global_parameter_client.cpp
 *
 *  Created on: Dec 2, 2021
 *      Author: jnicho
 */

#include <boost/format.hpp>

#include <pybind11/stl.h>

#include "global_parameter_client.hpp"

namespace rclpy_parameter_utils
{

GlobalParameterClient::GlobalParameterClient(const std::string& node_name, const std::string& node_namespace, int number_of_threads):
    RclCppComponent(),
    executor_(rclcpp::ExecutorOptions(), number_of_threads)
{
  node_ = std::make_shared<rclcpp::Node>(node_name, node_namespace);
}

GlobalParameterClient::~GlobalParameterClient()
{
  stop();
}

void GlobalParameterClient::start()
{
  executor_thread_ = std::thread([&](){
    executor_.add_node(node_);
    executor_.spin();
  });
}

void GlobalParameterClient::stop()
{
  executor_.cancel();
  if(executor_thread_.joinable())
  {
    executor_thread_.join();
  }
}

rcl_interfaces::msg::Parameter GlobalParameterClient::getParameter(const std::string& full_parameter_name, double timeout_secs)
{
  rcl_interfaces::msg::Parameter ret_param_msg;
  ret_param_msg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;

  std::size_t last_slash_pos = full_parameter_name.find_last_not_of('/');
  if(last_slash_pos >= full_parameter_name.size()-1)
  {
    return ret_param_msg;
  }
  std::string remote_node_name = full_parameter_name.substr(0, last_slash_pos);
  std::string param_name = full_parameter_name.substr(last_slash_pos + 1, std::string::npos);

  std::vector<rclcpp::Parameter> parameters = getParametersImpl(remote_node_name, {param_name}, timeout_secs);
  if(parameters.empty())
  {
    return ret_param_msg;
  }

  ret_param_msg = parameters.front().to_parameter_msg();
  return ret_param_msg;
}

pybind11::dict GlobalParameterClient::getParameters(const std::string& parameters_namespace, const std::vector<std::string>& parameter_names, double timeout_secs)
{
  std::vector<rclcpp::Parameter> parameters = getParametersImpl(parameters_namespace, parameter_names, timeout_secs);
  pybind11::dict output_dict;
  for(const auto& p : parameters)
  {
    pybind11::object pystr = pybind11::cast(p.get_name(), pybind11::return_value_policy::copy); // TODO: May not be necessary to "copy"
    rcl_interfaces::msg::Parameter p_msg = p_msg = p.to_parameter_msg();

    output_dict[pystr] = pybind11::cast(p_msg);
  }
  return output_dict;
}

rclcpp::SyncParametersClient GlobalParameterClient::createParameterClient(rclcpp::Node::SharedPtr node,
                                                   const std::string& remote_node_name,
                                                   double timeout_secs)
{
  rclcpp::SyncParametersClient parameter_client = rclcpp::SyncParametersClient(node,remote_node_name);
  parameter_client.wait_for_service(std::chrono::duration<double>(timeout_secs));
  if(!parameter_client.service_is_ready())
  {
    const std::string error_msg = boost::str(boost::format("Failed to connect to parameter service %s") % remote_node_name);
    throw std::runtime_error(error_msg);
  }
  return parameter_client;
}

std::vector<rclcpp::Parameter> GlobalParameterClient::getParametersImpl(const std::string& remote_node_name, const std::vector<std::string>& parameter_names, double timeout_secs)
{
  rclcpp::SyncParametersClient parameter_client = createParameterClient(node_,remote_node_name, timeout_secs);
  std::vector<rclcpp::Parameter> parameters = parameter_client.get_parameters(parameter_names);
  return parameters;
}

void GlobalParameterClient::setParameter(const std::string &full_parameter_name, const rclcpp::ParameterValue& parameter_val, double timeout_secs)
{
  std::size_t last_slash_pos = full_parameter_name.find_last_not_of('/');
  if(last_slash_pos >= full_parameter_name.size()-1)
  {
    throw std::runtime_error(boost::str(boost::format("Parameter name %s is malformed") % full_parameter_name));
  }
  std::string remote_node_name = full_parameter_name.substr(0, last_slash_pos);
  std::string param_name = full_parameter_name.substr(last_slash_pos + 1, std::string::npos);
  setParametersImpl(remote_node_name, {rclcpp::Parameter(param_name,parameter_val)}, timeout_secs);
}

void GlobalParameterClient::setParametersImpl(const std::string &remote_node_name,
                                              const std::vector<rclcpp::Parameter> &parameters,
                                              double timeout_secs)
{
  rclcpp::SyncParametersClient parameter_client = createParameterClient(node_,remote_node_name, timeout_secs);
  std::vector<rcl_interfaces::msg::SetParametersResult> parameter_results = parameter_client.set_parameters(parameters);
  for(const auto& res : parameter_results)
  {
    if(!res.successful)
    {
      throw std::runtime_error(res.reason);
    }
  }
}

} /* namespace rclpy */
