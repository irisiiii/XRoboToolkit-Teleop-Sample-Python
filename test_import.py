#!/usr/bin/env python3
import os
import sys

# 设置 ROS2 环境变量
workspace_path = "/root/jwq/xr/XRoboToolkit-Teleop-Sample-Python"
install_path = os.path.join(workspace_path, "install")

required_lib_paths = [
    os.path.join(install_path, "jaka_robot_interfaces/lib"),
    os.path.join(install_path, "jiazhua_interfaces/lib"),
]

ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
needs_restart = False
for lib_path in required_lib_paths:
    if os.path.exists(lib_path) and lib_path not in ld_library_path:
        needs_restart = True
        ld_library_path = f"{lib_path}:{ld_library_path}"

if needs_restart and "ROS_ENV_RESTARTED" not in os.environ:
    print("🔄 重新启动以加载环境...")
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = ld_library_path
    env["ROS_ENV_RESTARTED"] = "1"
    os.execve(sys.executable, [sys.executable] + sys.argv, env)

jaka_msg_path = os.path.join(install_path, "jaka_robot_interfaces/lib/python3.10/site-packages")
jiazhua_msg_path = os.path.join(install_path, "jiazhua_interfaces/lib/python3.10/site-packages")
if os.path.exists(jaka_msg_path):
    sys.path.insert(0, jaka_msg_path)
if os.path.exists(jiazhua_msg_path):
    sys.path.insert(0, jiazhua_msg_path)

print("✅ 环境已设置")
print(f"LD_LIBRARY_PATH: {os.environ.get('LD_LIBRARY_PATH', '')[:200]}...")

try:
    from jaka_robot_interfaces.msg import ServoJointCommand, JointValue
    print("✅ ServoJointCommand 导入成功")
    
    from jiazhua_interfaces.msg import JiaZhuaDualCmd
    print("✅ JiaZhuaDualCmd 导入成功")
    
    print("\n测试创建消息实例...")
    cmd = ServoJointCommand()
    cmd.joint_pos_left = JointValue()
    cmd.joint_pos_left.joint_values = [0.0] * 7
    print("✅ 消息创建成功")
    
except Exception as e:
    print(f"❌ 导入失败: {e}")
    import traceback
    traceback.print_exc()

