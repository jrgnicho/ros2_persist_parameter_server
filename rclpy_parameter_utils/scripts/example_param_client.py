#!/usr/bin/env python3

import sys

import rclpy
import rclpy_parameter_utils

from rcl_interfaces.msg import Parameter as ParameterMsg

DEFAULT_PARAMETER_SERVER_NAME = 'global_parameter_server'

def main():
    
    try:
        rclpy.init(args = sys.argv)
        node = rclpy.create_node('test_global_parameters_node')        
        
        #param = rclpy.Parameter()
        #msg = rcl_interfaces.msg.Parameter()
        
        #print(type(msg.value))
        #print(str(msg))
        
        global_parameter_server_name = node.get_parameter_or('parameter_server_name',
                                                               rclpy.Parameter('parameter_server_name', value = DEFAULT_PARAMETER_SERVER_NAME)).value
        global_param_client = rclpy_parameter_utils.impl.GlobalParameterClient(node.get_name(),'', 2)
        
        
        # Getting Parameters
        nested_double_param = global_param_client.get_parameter(global_parameter_server_name, 'nested_param.another_nested_param.double_val')
        print('Got parameter "%s" : %s'%(nested_double_param.name, str(nested_double_param.value)))
        
        multiple_params_dict = global_param_client.get_parameters(global_parameter_server_name,['str_list_val',
                                                                                                 'nested_param.another_nested_param.double_val',
                                                                                                 'str_val',
                                                                                                 'double_val'])
        print('Got several parameters')
        print(dict(map(lambda p: (p[0], p[1].value), multiple_params_dict.items())))
        
        # Setting Parameters
        new_param = rclpy.Parameter(nested_double_param.name,type_ = nested_double_param.type_, value = -1 *nested_double_param.value)
        global_param_client.set_parameter(global_parameter_server_name,new_param)
        print('Set parameter "%s" to value %s'%(new_param.name, str(new_param.value)))
        
        rclpy.spin(node)
    except KeyboardInterrupt as ex:
        pass
    
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    
    main()