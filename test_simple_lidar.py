#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RoboSense LiDAR 简化接口测试

这个脚本演示了简化后的RoboSense LiDAR实时点云获取接口：
- 只需要配置LiDAR的IP地址
- 只有三个核心方法：initialize(), get(), stop()
- 自动使用默认参数（端口6699/7788，RS16类型）
"""

import numpy as np
import time
import sys
import os
import signal
import atexit

# 添加模块路径

try:
    import rs_xue
except ImportError as e:
    print(f"导入rs_xue模块失败: {e}")
    print("请确保模块已正确编译")
    sys.exit(1)

# 全局客户端变量，用于信号处理
global_client = None

def signal_handler(signum, frame):
    """
    信号处理函数，确保程序被中断时正确清理资源
    """
    print("\n收到中断信号，正在安全停止LiDAR客户端...")
    if global_client is not None:
        try:
            global_client.stop()
            print("LiDAR客户端已安全停止")
        except Exception as e:
            print(f"停止客户端时发生错误: {e}")
    sys.exit(0)

def cleanup_on_exit():
    """
    程序退出时的清理函数
    """
    if global_client is not None:
        try:
            global_client.stop()
        except:
            pass

# 注册信号处理器和退出清理函数
signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
signal.signal(signal.SIGTERM, signal_handler)  # 终止信号
atexit.register(cleanup_on_exit)               # 程序退出时清理

def test_simple_lidar():
    """
    测试简化的LiDAR接口
    """
    global global_client
    print("=== RoboSense LiDAR 简化接口测试 ===")
    
    # 创建客户端
    client = rs_xue.RealtimeLidarClient()
    global_client = client  # 设置全局变量用于信号处理
    
    try:
        # 只需要配置LiDAR IP地址，其他参数使用默认值
        lidar_ip = "192.168.1.200"  # 修改为你的LiDAR IP地址
        
        print(f"正在初始化LiDAR客户端 (IP: {lidar_ip})...")
        if not client.initialize(lidar_ip):
            print("初始化失败")
            return False
        
        print("初始化成功！开始获取点云数据...")
        print("提示：get()方法会自动启动LiDAR连接")
        
        # 获取点云数据（自动启动连接）
        for i in range(10):
            print(f"\n获取第 {i+1} 帧点云数据...")
            
            # get()现在直接返回numpy数组或None
            points = client.get()
            if points is not None:
                point_count = len(points)
                
                print(f"  成功获取点云数据")
                print(f"  点数: {point_count}")
                print(f"  点云数组形状: {points.shape}")
                
                if len(points) > 0:
                    print(f"  前3个点坐标:")
                    for j in range(min(3, len(points))):
                        x, y, z = points[j]
                        print(f"    点{j+1}: ({x:.3f}, {y:.3f}, {z:.3f})")
                    
                    # 检查是否有NaN值
                    nan_count = np.isnan(points).sum()
                    if nan_count > 0:
                        print(f"  警告: 发现 {nan_count} 个NaN值")
                        # 过滤NaN值
                        valid_mask = ~np.isnan(points).any(axis=1)
                        valid_points = points[valid_mask]
                        print(f"  有效点数: {len(valid_points)}")
                        
                        if len(valid_points) > 0:
                            print(f"  X范围: [{np.min(valid_points[:, 0]):.3f}, {np.max(valid_points[:, 0]):.3f}]")
                            print(f"  Y范围: [{np.min(valid_points[:, 1]):.3f}, {np.max(valid_points[:, 1]):.3f}]")
                            print(f"  Z范围: [{np.min(valid_points[:, 2]):.3f}, {np.max(valid_points[:, 2]):.3f}]")
                        else:
                            print("  所有点都是无效的")
                    else:
                        # 显示点云统计信息
                        print(f"  X范围: [{np.min(points[:, 0]):.3f}, {np.max(points[:, 0]):.3f}]")
                        print(f"  Y范围: [{np.min(points[:, 1]):.3f}, {np.max(points[:, 1]):.3f}]")
                        print(f"  Z范围: [{np.min(points[:, 2]):.3f}, {np.max(points[:, 2]):.3f}]")
                else:
                    print("  点云数组为空")
            else:
                print(f"  获取失败")
            
        
        print("\n测试完成！")
        return True
        
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        return False
    finally:
        # 停止客户端
        print("\n正在停止客户端...")
        try:
            client.stop()
            print("客户端已停止")
        except Exception as e:
            print(f"停止客户端时发生错误: {e}")
        global_client = None  # 清除全局引用

def main():
    """
    主函数
    """
    print("RoboSense LiDAR 简化接口测试程序")
    print("按Ctrl+C可以随时退出")
    print("="*50)
    
    print("\n简化接口说明:")
    print("1. initialize(lidar_ip) - 只需要LiDAR IP地址")
    print("2. get() - 获取点云数据（返回numpy数组）")
    print("3. stop() - 停止LiDAR客户端")
    print("\n默认参数:")
    print("- MSOP端口: 6699")
    print("- DIFOP端口: 7788")
    print("- LiDAR类型: RSEM4")
    print("- 主机IP: 自动检测")
    
    try:
        # 运行测试
        success = test_simple_lidar()
        
        if success:
            print("\n🎉 测试成功！")
            print("\n使用示例:")
            print("```python")
            print("import rs_xue")
            print("import numpy as np")
            print("")
            print("# 创建客户端")
            print("client = rs_xue.RealtimeLidarClient()")
            print("")
            print("# 初始化（只需要IP地址）")
            print("client.initialize('192.168.1.200')")
            print("")
            print("# 获取点云数据（直接返回numpy数组）")
            print("points = client.get()")
            print("if points is not None:")
            print("    print(f'获取到 {len(points)} 个点')")
            print("    print(f'点云形状: {points.shape}')")
            print("    # points是形状为(N, 3)的numpy数组，每行为[x, y, z]")
            print("")
            print("# 停止")
            print("client.stop()")
            print("```")
        else:
            print("\n❌ 测试失败")
            print("\n请检查:")
            print("1. LiDAR IP地址是否正确")
            print("2. 网络连接是否正常")
            print("3. LiDAR是否已启动")
            print("4. 防火墙是否阻止了UDP端口6699/7788")
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"\n程序执行过程中发生错误: {e}")
    
    print("\n测试程序结束")

if __name__ == "__main__":
    main()