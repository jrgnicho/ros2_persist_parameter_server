/*
 * rclcpp_component.cpp
 *
 *  Created on: Dec 4, 2021
 *      Author: jnicho
 */

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_component.hpp"

rclpy_parameter_utils::RclCppComponent::RclCppComponent()
{
  if(rclcpp::is_initialized())
  {
    return;
  }
  int argc = 1;
  char *argv[] = {(char*)"nameless"};
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclpy_parameter_utils"), "initialized rclcpp");

}

rclpy_parameter_utils::RclCppComponent::~RclCppComponent()
{

}
