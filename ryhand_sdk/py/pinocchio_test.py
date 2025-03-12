# NOTE: this example needs gepetto-gui to be installed
# usage: launch gepetto-gui and then run this test
 
import sys
from pathlib import Path
 
import pinocchio as pin
from pinocchio.visualize import GepettoVisualizer
 
# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython
pinocchio_model_dir = Path(__file__).parent.parent / "py"
 
model_path = pinocchio_model_dir 
mesh_dir = pinocchio_model_dir
urdf_filename = "hand_left.urdf"
urdf_model_path = model_path / urdf_filename
 
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)
viz = GepettoVisualizer(model, collision_model, visual_model)
 
# Initialize the viewer.
try:
    viz.initViewer()
except ImportError as err:
    print(
        "Error while initializing the viewer. "
        "It seems you should install gepetto-viewer"
    )
    print(err)
    sys.exit(0)
 
try:
    viz.loadViewerModel("pinocchio")
except AttributeError as err:
    print(
        "Error while loading the viewer model. "
        "It seems you should start gepetto-viewer"
    )
    print(err)
    sys.exit(0)
 
# Display a robot configuration.
q0 = pin.neutral(model)
viz.display(q0)
 
# Display another robot.
viz2 = GepettoVisualizer(model, collision_model, visual_model)
viz2.initViewer(viz.viewer)
viz2.loadViewerModel(rootNodeName="pinocchio2")
q = q0.copy()
q[1] = 1.0
viz2.display(q)















from pathlib import Path
from sys import argv
from math import pi
import numpy as np
import pinocchio as pin

# This path refers to Pinocchio source code but you can define your own directory here.
pinocchio_model_dir = Path(__file__).parent.parent / "py"
 
# You should change here to set up your own URDF file or just pass it as an argument of
# this example.
urdf_filename = (
    pinocchio_model_dir / "hand_left.urdf"
    if len(argv) < 2
    else argv[1]
)

mesh_dir = pinocchio_model_dir
 
# Load the urdf model
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_filename, mesh_dir
)
print("model name: " + model.name)
 
# Load the urdf model
# model = pin.buildModelFromUrdf(urdf_filename)
# print("model name: " + model.name)
 
# Create data required by the algorithms
data = model.createData()

# Define joint configuration
q = pin.randomConfiguration(model)
print(f"before q: {q.T}")
for i in range(len(q)):
    if i%4 == 3:
        q[i] = q[i-1]
print(f"after q: {q.T}")
 
# Perform the forward kinematics over the kinematic tree
pin.forwardKinematics(model, data, q)
 
# Print out the placement of each joint of the kinematic tree
print("{:<24} : {:^10} {:^10} {:^10} | {:^8} {:^8} {:^8}".format(
    "Joint Name", "X", "Y", "Z", "Roll", "Pitch", "Yaw"))
print("-" * 80)

for name, oMi in zip(model.names, data.oMi):
    # 提取位置信息
    translation = oMi.translation
    
    # 提取姿态信息并转换为欧拉角(RPY)
    rotation = oMi.rotation
    rpy = pin.rpy.matrixToRpy(rotation) * 180 / pi # 转换为roll-pitch-yaw欧拉角
    
    # 格式化输出位置和姿态
    print("{:<24} : {: .2f} {: .2f} {: .2f} | {: .2f} {: .2f} {: .2f}".format(
        name,
        translation[0], translation[1], translation[2],  # 位置
        rpy[0], rpy[1], rpy[2]  # 姿态(Roll, Pitch, Yaw)
    ))
    
# Create data required by the algorithms
data, collision_data, visual_data = pin.createDatas(
    model, collision_model, visual_model
)

# Update Geometry models
pin.updateGeometryPlacements(model, data, collision_model, collision_data)
pin.updateGeometryPlacements(model, data, visual_model, visual_data)
 
# Print out the placement of each joint of the kinematic tree
print("\nJoint placements:")
for name, oMi in zip(model.names, data.oMi):
    print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))
 
# Print out the placement of each collision geometry object
print("\nCollision object placements:")
for k, oMg in enumerate(collision_data.oMg):
    print("{:d} : {: .2f} {: .2f} {: .2f}".format(k, *oMg.translation.T.flat))
 
# Print out the placement of each visual geometry object
print("\nVisual object placements:")
for k, oMg in enumerate(visual_data.oMg):
    print("{:d} : {: .2f} {: .2f} {: .2f}".format(k, *oMg.translation.T.flat))




    
# --------------------------添加逆运动学部分--------------------------
print("\n---------------- 逆运动学求解 ----------------")

# 选择末端执行器的框架ID（这里假设使用最后一个关节/链接作为末端执行器）
# 你可能需要根据你的模型结构选择正确的ID
JOINT_ID = model.getFrameId(model.frames[-1].name)  # 选择最后一个框架作为末端执行器
print(f"末端执行器框架ID: {JOINT_ID}, 名称: {model.frames[JOINT_ID].name}")

# 获取当前末端执行器的位置和姿态作为目标
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)
current_pose = data.oMf[JOINT_ID].copy()  # 当前末端执行器位姿

# 设置目标位置（稍微修改当前位置，以测试求解器）
target_position = current_pose.translation.copy()
target_position[0] += 0.005  # X方向移动5cm
target_position[1] -= 0.002  # Y方向移动-2cm

# 设置目标姿态（保持当前姿态）
target_rotation = current_pose.rotation.copy()

# 创建目标变换矩阵
target_pose = pin.SE3(target_rotation, target_position)

print("当前末端执行器位置:", current_pose.translation)
print("目标末端执行器位置:", target_position)

# 初始化求解器参数
q_init = q.copy()  # 使用当前配置作为初始猜测
DT = 1e-1          # 时间步长
MAX_IT = 100       # 最大迭代次数
EPSILON = 1e-4     # 收敛阈值

# 逆运动学求解
print("\n开始逆运动学求解...")
q_ik = q_init.copy()
  
success = False
for i in range(MAX_IT):
    pin.forwardKinematics(model, data, q_ik)
    pin.updateFramePlacements(model, data)
    
    current_pose = data.oMf[JOINT_ID]
    
    # 计算位置误差
    error = pin.log6(current_pose.inverse() * target_pose).vector
    error_norm = np.linalg.norm(error)
    
    if error_norm < EPSILON:
        success = True
        print(f"求解成功! 迭代 {i+1} 次, 误差: {error_norm:.6f}")
        break
    
    # 计算雅可比矩阵
    J = pin.computeFrameJacobian(model, data, q_ik, JOINT_ID)
    
    # 计算关节增量
    v = np.linalg.pinv(J) @ error
    
    # 更新关节角度
    q_ik = pin.integrate(model, q_ik, v * DT)
    
    if (i+1) % 10 == 0:
        print(f"迭代 {i+1}, 误差: {error_norm:.6f}")

if not success:
    print(f"未收敛! 最终误差: {error_norm:.6f}")

# 打印结果
print("\n逆运动学求解结果:")
print(f"初始关节角: {q_init.T}")
print(f"求解关节角: {q_ik.T}")

# 验证结果
print("\n验证逆运动学结果...")
pin.forwardKinematics(model, data, q_ik)
pin.updateFramePlacements(model, data)
final_pose = data.oMf[JOINT_ID]

print(f"目标位置: {target_position}")
print(f"最终位置: {final_pose.translation}")
print(f"位置误差: {np.linalg.norm(final_pose.translation - target_position):.6f}")

# 添加约束条件验证
for i in range(len(q_ik)):
    if i%4 == 3 and abs(q_ik[i] - q_ik[i-1]) > 1e-6:
        print(f"警告: 约束条件未满足! q[{i}]={q_ik[i]:.6f} != q[{i-1}]={q_ik[i-1]:.6f}")