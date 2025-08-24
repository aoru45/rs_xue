"""
RealSense LiDAR Python Interface

This package provides Python bindings for RealSense LiDAR devices.
"""

import os
import sys

# 获取当前模块的目录
current_dir = os.path.dirname(os.path.abspath(__file__))

# 查找.so文件
so_files = [f for f in os.listdir(current_dir) if f.endswith('.so')]

if so_files:
    # 导入编译好的模块
    so_file = so_files[0]  # 使用第一个找到的.so文件
    module_name = so_file.split('.')[0]  # 获取模块名（去掉扩展名）
    
    # 动态导入.so模块
    import importlib.util
    spec = importlib.util.spec_from_file_location(module_name, os.path.join(current_dir, so_file))
    rs_xue_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rs_xue_module)
    
    # 导出主要类和函数
    Client = rs_xue_module.Client
    
    # 导出其他可能的函数
    if hasattr(rs_xue_module, 'processCloud'):
        processCloud = rs_xue_module.processCloud
    if hasattr(rs_xue_module, 'processCloudWithCalib'):
        processCloudWithCalib = rs_xue_module.processCloudWithCalib
    if hasattr(rs_xue_module, 'convert_pcap'):
        convert_pcap = rs_xue_module.convert_pcap
    if hasattr(rs_xue_module, 'convert_pcap_with_calib'):
        convert_pcap_with_calib = rs_xue_module.convert_pcap_with_calib
        
    __all__ = ['Client']
    
    # 添加其他导出的函数到__all__
    if 'processCloud' in locals():
        __all__.append('processCloud')
    if 'processCloudWithCalib' in locals():
        __all__.append('processCloudWithCalib')
    if 'convert_pcap' in locals():
        __all__.append('convert_pcap')
    if 'convert_pcap_with_calib' in locals():
        __all__.append('convert_pcap_with_calib')
else:
    raise ImportError("No compiled .so file found in the package")

__version__ = '1.0.0'