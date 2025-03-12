import pybullet as p
import time
import math
import asyncio


joint_angle_ids = []
joint_lower_limits = []
joint_upper_limits = []

pre_angles = [0] *15
anglecmds = [0] *15

hand_lr = 0



def map_angle90_to_value(angle):
    # Ensure the angle is within the valid range
    if angle < 0:
        angle = 0
    elif angle > math.pi / 2:
        angle = math.pi / 2

    # Map the angle from 0 to pi/2 to 4095 to 0
    value = 4095 - (angle / (math.pi / 2)) * 4095
    return int(value)

def map_angle75_to_value(angle):
    # Ensure the angle is within the valid range
    if angle < 0:
        angle = 0
    elif angle > math.pi * 75 / 180:
        angle = math.pi  * 75 / 180

    # Map the angle from 0 to math.pi  * 75 / 180 to 4095 to 0
    value = 4095 - (angle / (math.pi  * 75 / 180)) * 4095
    return int(value)


def map_angle_to_value_full_range(angle):
    # Ensure the angle is within the valid range
    if angle < -math.pi / 2:
        angle = -math.pi / 2
    elif angle > math.pi / 2:
        angle = math.pi / 2

    # Map the angle from -pi/2 to pi/2 to -4095 to 4095
    value = (angle / (math.pi / 2)) * 4095
    return int(value)




async def connect_and_run_simulation():

        cnt = 0

        # 运行仿真
        while True:
            p.stepSimulation()
            ptxtang = "\r"
            ptxtcmd = "\r"
            angles = []
        
            for joint_index, joint_angle_id in enumerate(joint_angle_ids):
                joint_angle = p.readUserDebugParameter(joint_angle_id)
                ptxtang += f"J{joint_index}:{joint_angle:.2f} "
                angles.append(joint_angle)

                # anglecmds[joint_index] = map_angle_to_value(angles[joint_index])

            for i in range( len(angles) ):
                pre_angles[i] = angles[i]
                if i % 3 != 0:
                    if i % 3 == 1:
                        anglecmds[i] = map_angle90_to_value(pre_angles[i])
                    else:
                        anglecmds[i] = map_angle75_to_value(pre_angles[i])
                else:
                    anglecmds[i] = map_angle_to_value_full_range(pre_angles[i])

            j = 0
            ang = 0
            for i in range( len(angles) ):

                if i % 3 == 0:
                    ang = anglecmds[i]

                    if hand_lr == 0:
                        anglecmds[i] = anglecmds[i+1] - ang
                    else:
                        anglecmds[i] = anglecmds[i+1] + ang

                elif i % 3 == 1:
                    if hand_lr == 0:
                        anglecmds[i] = anglecmds[i] + ang
                    else:
                        anglecmds[i] = anglecmds[i] - ang
                else:
                    anglecmds[i] = anglecmds[i]

                
                ptxtcmd += f"M{j+1}:{anglecmds[i]} " 
                j += 1          


            for joint_index, joint_angle_id in enumerate(joint_angle_ids): 
                mod_i = joint_index % 3
                k = (joint_index // 3) * 5 + mod_i

                p.setJointMotorControl2(bodyUniqueId=robot_id,
                                            jointIndex=k,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=pre_angles[joint_index])
                
                if mod_i == 2:
                    p.setJointMotorControl2(bodyUniqueId=robot_id,
                                            jointIndex=k+1,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=pre_angles[joint_index])

            
                
            
            # if cnt % 10 == 0:
            #     print( ptxtcmd + " " * 20, end="" )
            #     cmds = [anglecmds[i] for i in [0,1,2, 4,5,6, 8,9,10, 12,13,14, 16,17,18]]

                # if hand_lr == 0:
                #    await SendCmdData15(client, cmds, "left")
                # else:
                #    await SendCmdData15(client, cmds, "right")

            cnt += 1

            time.sleep(1.0 / 240.0)




if __name__ == "__main__":

    
    # 连接到PyBullet仿真环境
    physicsClient = p.connect(p.GUI)

    # 加载URDF文件
    urdf_path = "hand_left/ruihand15z.urdf" 
    robot_id = p.loadURDF(urdf_path)

    # 获取机器人中的关节数量
    num_joints = p.getNumJoints(robot_id)
    print(f"Number of joints: {num_joints}")

    # 添加调试参数来显示关节角度
    for joint_index in range(num_joints):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_lower_limit = joint_info[8]
        joint_upper_limit = joint_info[9]
        joint_lower_limits.append(joint_lower_limit)
        joint_upper_limits.append(joint_upper_limit)

        if joint_index %5 < 3:
            joint_angle_id = p.addUserDebugParameter(f"Joint {joint_index} Angle", joint_lower_limit, joint_upper_limit, 0)
            joint_angle_ids.append(joint_angle_id)

    print( len(joint_angle_ids) )



    # 创建一个固定约束，将模型固定在当前位置
    base_position, base_orientation = p.getBasePositionAndOrientation(robot_id)
    constraint_id = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=-1,
        childBodyUniqueId=-1,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=base_position,
        parentFrameOrientation=base_orientation,
        childFramePosition=[0, 0, 0],
        childFrameOrientation=[0, 0, 0]
    )

    # 使模型x轴正对屏幕
    p.resetDebugVisualizerCamera(
        cameraDistance=0.3,
        cameraYaw=90,
        cameraPitch=-35,
        cameraTargetPosition=base_position
    )


    # # 显示小坐标系
    # for joint_index in range(num_joints):
    #     p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=robot_id, parentLinkIndex=joint_index)
    #     p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=robot_id, parentLinkIndex=joint_index)
    #     p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=robot_id, parentLinkIndex=joint_index)


    


    # 设置仿真步长
    p.setTimeStep(1.0 / 240.0)


    asyncio.run(connect_and_run_simulation())




# 断开连接
p.disconnect()