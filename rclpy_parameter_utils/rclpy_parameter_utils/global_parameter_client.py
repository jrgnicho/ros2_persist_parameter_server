
from typing import List

from rclpy.parameter import Parameter

import rcl_interfaces
from rcl_interfaces.srv import *

from rclpy.parameter import Parameter
import rclpy_parameter_utils
import rclpy
from builtins import isinstance

class GlobalParameterClient:
    
    def __init__(self, node):
        
        self.__node = node
        self.__parameter_server_name = ''
        self.__get_param_client = None
        self.__set_param_client = None
        
    def __del__(self):
        pass
        
    def set_parameter(self, param_server_name : str, param: Parameter):
        return self._set_parameter_impl(param_server_name, [param]) 
    
    def set_parameters(self, param_server_name : str, params_list: List[Parameter]):
        return self._set_parameter_impl(param_server_name, params_list)
        
    def get_parameter(self,param_server_name : str, param_name : str):
        
        params = self._get_parameters_impl(param_server_name, [param_name])  
        if len(params) == 0:
            self.__node.get_logger().error('Failed to get parameter "%s" from parameter server "%s"'%(param_name, param_server_name))
        return params[0]  
    
    def get_parameter_or(self,param_server_name, param_name, param_default):
        if not isinstance(param_default, Parameter):
            raise RuntimeError('Default parameter passed is not of %s type', str(type(Parameter)))  
        
        params = self._get_parameters_impl(param_server_name, [param_name])  
        if len(params) == 0:
            return param_default
        
        return params[0]
    
    def get_parameters(self, param_server_name: str, param_names : List[str]):
        params_list = self._get_parameters_impl(param_server_name, param_names) 
        if len(params_list) == 0:
            self.__node.get_logger().error('Failed to get parameter "%s" from parameter server "%s"'%(str(param_names), param_server_name))
            return {}
        
        params_dict = dict(map(lambda p : (p.name, p), params_list))
        return params_dict 
    
    def _get_parameters_impl(self,param_server_name, param_names: List[str]):
        if self.__parameter_server_name != param_server_name:
            self._create_clients(param_server_name)
        
        req = GetParameters.Request(names = param_names)
        
        res = self.__get_param_client.call(req) 
        if res is None:
            return []  
        
        params : List[Parameter] = []
        for i, param_val in enumerate(res.values):
            param_msg = rcl_interfaces.msg.Parameter()
            param_msg.name = param_names[i]
            param_msg.value = param_val
            p = Parameter.from_parameter_msg(param_msg)
            params.append(p)
            
        return params    
    
    def _set_parameter_impl(self, param_server_name, params_list : List[Parameter]):
        if self.__parameter_server_name != param_server_name:
            self._create_clients(parameter_server_name)
            
        req = SetParameters.Request()
        for param in params_list:
            req.parameters.append(param.to_parameter_msg())
            
        res = self.__set_param_client.call(req)
        succeeded = True
        for set_result in res.results:
            succeeded = succeeded and set_result.successful
            if not succeeded:
                self.__node.get_logger().error('Set parameter failed: %s'%(set_result.reason))
                return False
        
        return True            
        
    def _create_clients(self, parameter_server_name):        
        if (not isinstance(parameter_server_name, str)) or parameter_server_name == '':
            raise RuntimeError('Parameter service name can not be empty string')
            
        self.__parameter_server_name = parameter_server_name
        self.__get_param_client = self.__node.create_client(GetParameters, parameter_server_name + '/get_parameters')
        self.__set_param_client = self.__node.create_client(SetParameters, parameter_server_name + '/set_parameters')
        
    @property
    def impl(self):
        return self.__cobj
    