# 左右手双节点启动文件（ROS 2）
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rh15_ctrl',
            executable='rh_ctrl',
            namespace='rh15_left',
            name='left',
            output='screen',
            arguments=['can0']   # argv[1] = 'can0'
        ),
        Node(
            package='rh15_ctrl',
            executable='rh_ctrl',
            namespace='rh15_right',
            name='right',
            output='screen',
            arguments=['can1']   # argv[1] = 'can0'
        ),
        # ros2 run rh6_ctrl rh_ctrl can0
    ])


# ros2 launch rh15_ctrl BothHands.launch.py

# 运行该脚本应该能看到如下输出：

# $ ros2 topic list
# *
# /rh6_left/ryhand15_cmd
# /rh6_left/ryhand15_status
# /rh6_right/ryhand15_cmd
# /rh6_right/ryhand15_status
# *

# $ ros2 node list
# /rh15_left/left
# /rh15_right/right