不本程序是根据亚伯智能机械臂2次开发：

关于厂家提供的程序：
厂商 SDK + Demo 程序：dofbot_pro/
ROS2 封装层：dofbot_pro_ws/
用户层 / 实验层：colcon_ws

在这里记录robot Arm调试需要的keypoints：

<img width="333" height="426" alt="image" src="https://github.com/user-attachments/assets/481b318f-7ade-4323-a22e-602480077bc4" />
pin1: 3.3v
pin3: SDA
pin5: SCL pin6: GND
按照这个顺序和载版的I2C接口连接（位于左上角和一片OLED在一起）

上电后可以首先运行测试程序yahboom_oled.py测试I2C是否通信正常

然后可以开启dofbot_pro_ws/里的关键控制节点
ros2 run dofbot_pro_driver arm_driver

可以使用ros topic控制机械臂：
ros2 topic pub --once /TargetAngle dofbot_pro_interface/msg/ArmJoint "
run_time: 4000
joints: [90, 90, 90, 0, 90, 90]"


本程序的改进：
原来的arm_driver假定机器人姿态正常，如果在机械臂被认为移动的情况下可能导致线缆卡住无法回位

## Workspace Structure

This repository contains multiple workspaces and vendor components.

- [`dofbot_pro_ws/src`](dofbot_pro_ws/src)  
  ROS 2 packages for DOFBOT Pro robotic arm  
  → See detailed package breakdown:  
  [`dofbot_pro_ws/src/Content Overview.md`](dofbot_pro_ws/src/Content Overview.md)


