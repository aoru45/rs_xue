#!/usr/bin/env python3
"""
测试安装后的rs_xue包
"""

try:
    import rs_xue
    print(f"Successfully imported rs_xue version {rs_xue.__version__}")
    print(f"Available classes/functions: {rs_xue.__all__}")
    
    # 测试创建客户端
    client = rs_xue.RealtimeLidarClient()
    print("Successfully created RealtimeLidarClient")
    
except ImportError as e:
    print(f"Failed to import rs_xue: {e}")
except Exception as e:
    print(f"Error: {e}")