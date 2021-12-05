#!/usr/bin/env python3

import sys

import rclpy
import rclpy_parameter_utils

def main():
    
    try:
        rclpy.init(args = sys.argv)
        node = rclpy.create_node('test_parameter_client')
        global_param_client = rclpy_parameter_utils.impl.GlobalParameterClient(node.get_name(),'client', 1)
        global_param_client.start()
        
        print('spinning node now ...')
        rclpy.spin(node)
    except KeyboardInterrupt as ex:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    
    main()