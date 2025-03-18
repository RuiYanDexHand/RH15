#include "rh15_ctrl.hpp"
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>
#include <string>
#include <thread>
#include <pthread.h>
#include <sched.h>
#include <chrono>
#include <pthread.h>
#include <stdio.h>
#include <sched.h>
#include <stdlib.h>

extern "C" {
   #include <ryhandlib_port.h>
   #include <can_socket.h>
}





pthread_t thread_id;
volatile int  thread_go = 1;

using namespace std::chrono_literals;

std::string exec(const char* cmd) 
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}


std::string get_pkg_directory(std::string package_name)
{
    std::string command = "ros2 pkg prefix " + package_name;
    std::string package_path = exec(command.c_str());
    // std::cout << "The path of the package '" << package_name << "' is: " << package_path << std::endl;

    return package_path;
}



std::string extract_path_before_delimiter(const std::string &path, std::string delimiter) 
{
    // std::string delimiter = "/install";
    size_t pos = path.find(delimiter);
    if (pos != std::string::npos) 
    {
        return path.substr(0, pos);
    } 
    else 
    {
        return path; // 如果没有找到/install，返回原始路径
    }
}






// using namespace std::chrono_literals;

namespace ruiyan::rh15
{
    rh15_ctrl::rh15_ctrl( std::string name ) : Node( name )
    {
        RCLCPP_INFO(this->get_logger(), "hello %s",name.c_str());

        si = 0;
        rh15msg = rh15_msg::msg::Rh15Msg();
        rh15cmd = rh15_cmd::msg::Rh15Cmd();

        // 拇指
        poly_coeff[0][0] = -0.000091;
        poly_coeff[0][1] = 0.008974;
        poly_coeff[0][2] = 0.836899;
        poly_coeff[0][3] = 0.288791;

        // 食指
        poly_coeff[1][0] = -0.000091;
        poly_coeff[1][1] = 0.008974;
        poly_coeff[1][2] = 0.836899;
        poly_coeff[1][3] = 0.288791;

        // 中指
        poly_coeff[2][0] = -0.000071;
        poly_coeff[2][1] = 0.006173;
        poly_coeff[2][2] = 0.882956;
        poly_coeff[2][3] = 0.207410;

        // 无名指
        poly_coeff[3][0] = -0.000091;
        poly_coeff[3][1] = 0.008974;
        poly_coeff[3][2] = 0.836899;
        poly_coeff[3][3] = 0.288791;

        // 小指
        poly_coeff[4][0] = -0.000110;
        poly_coeff[4][1] = 0.011289;
        poly_coeff[4][2] = 0.801033;
        poly_coeff[4][3] = 0.363636;

        // std::string package_share_dir = get_pkg_directory("rh15_ctrl");
        // urdf_path = extract_path_before_delimiter( package_share_dir,"/install");

        // 加载 URDF 模型
        urdf_path = ament_index_cpp::get_package_share_directory("rh15_ctrl") + "/urdf";

        urdf_filename_l = urdf_path + "/ruihand15z.urdf";
        fingertip_l_[0] = "fz15";
        fingertip_l_[1] = "fz25";
        fingertip_l_[2] = "fz35";
        fingertip_l_[3] = "fz45";
        fingertip_l_[4] = "fz55";

        pinocchio::urdf::buildModel(urdf_filename_l, model_l_);
        data_l_ = pinocchio::Data(model_l_);

        urdf_filename_r = urdf_path + "/ruihand15y.urdf";
        fingertip_r_[0] = "fy15";
        fingertip_r_[1] = "fy25";
        fingertip_r_[2] = "fy35";
        fingertip_r_[3] = "fy45";
        fingertip_r_[4] = "fy55";

        pinocchio::urdf::buildModel(urdf_filename_r, model_r_);
        data_r_ = pinocchio::Data(model_r_);


        // 初始化关节状态，确保尺寸与模型匹配
        q_.resize(model_l_.nq);
        q_.setZero();

        q_fk_.resize(model_l_.nq);
        q_fk_.setZero();

        q_ik_.resize(model_l_.nq);
        q_ik_.setZero();

        RCLCPP_INFO(this->get_logger(), "joint_num: %d.",model_l_.nq);
        RCLCPP_INFO(this->get_logger(), "urdf_l_path: %s.",urdf_filename_l.c_str());
        RCLCPP_INFO(this->get_logger(), "urdf_r_path: %s.",urdf_filename_r.c_str());


        // interfaces_ptr_ = std::make_shared<InterfacesThread>(urdf_urdf_filenamepath,this->declare_parameter("handhand_can_id", "can0"), end_type);
        auto pub_name = this->declare_parameter("ryhand_pub_topic_name", "ryhand_status");
        auto sub_name = this->declare_parameter("ryhand_sub_topic_name", "ryhand_cmd");


        std::string control_type = this->declare_parameter("control_type", "normal");
        RCLCPP_INFO(this->get_logger(), "control_type = %s",control_type.c_str());

        if (control_type == "normal")
        {
            // 创建发布器 - 状态
            ryhand_state_publisher_ = this->create_publisher<rh15_msg::msg::Rh15Msg>(pub_name, 10);

            // 创建订阅器 - 命令
            ryhand_cmd_subscriber_ = this->create_subscription<rh15_cmd::msg::Rh15Cmd>( sub_name, 10, std::bind(&rh15_ctrl::CmdCallback, this, std::placeholders::_1) );

            // 创建定时器 10ms 
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&rh15_ctrl::PubState, this));
        }

        // // 创建服务
        callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        server_rh15fk_ = this->create_service<rh15_cmd::srv::Rh15fk>("rh15_fk", 
            std::bind(&rh15_ctrl::rh15fk_callback, this, std::placeholders::_1, std::placeholders::_2) , 
            rmw_qos_profile_services_default, callback_group_service_ );

        server_rh15ik_ = this->create_service<rh15_cmd::srv::Rh15ik>("rh15_ik", 
            std::bind(&rh15_ctrl::rh15ik_callback, this, std::placeholders::_1, std::placeholders::_2), 
            rmw_qos_profile_services_default, callback_group_service_ );
    


        
    }


    // 运动学正解
    void rh15_ctrl::rh15fk_callback(const rh15_cmd::srv::Rh15fk::Request::SharedPtr request, const rh15_cmd::srv::Rh15fk::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "rh15fk");

        if( request->lr )
        {
            model_fk_ = model_r_;
            data_fk_ = data_r_;
            fingertip[0] = fingertip_r_[0];
            fingertip[1] = fingertip_r_[1];
            fingertip[2] = fingertip_r_[2];
            fingertip[3] = fingertip_r_[3];
            fingertip[4] = fingertip_r_[4];
        }
        else
        {
            model_fk_ = model_l_;
            data_fk_ = data_l_;
            fingertip[0] = fingertip_l_[0];
            fingertip[1] = fingertip_l_[1];
            fingertip[2] = fingertip_l_[2];
            fingertip[3] = fingertip_l_[3];
            fingertip[4] = fingertip_l_[4];
        }

        q_fk_.resize(model_fk_.nq);
        q_fk_.setZero();
        for (int i = 0; i < 20; i += 4 )
        {
            q_fk_[ i/4*5 + 0 ] = request->j_ang[ i + 0 ];
            q_fk_[ i/4*5 + 1 ] = request->j_ang[ i + 1 ];
            q_fk_[ i/4*5 + 2 ] = request->j_ang[ i + 2 ];
            q_fk_[ i/4*5 + 3 ] = request->j_ang[ i + 3 ];
        }

        // Perform the forward kinematics over the kinematic tree
        forwardKinematics(model_fk_, data_fk_, q_fk_);
    
        // 获取末 关节坐标系的位姿
        pinocchio::SE3 end_effector_pose1 = data_fk_.oMi[ model_fk_.getJointId( fingertip[0] ) ];
        pinocchio::SE3 end_effector_pose2 = data_fk_.oMi[ model_fk_.getJointId( fingertip[1] ) ];
        pinocchio::SE3 end_effector_pose3 = data_fk_.oMi[ model_fk_.getJointId( fingertip[2] ) ];
        pinocchio::SE3 end_effector_pose4 = data_fk_.oMi[ model_fk_.getJointId( fingertip[3] ) ];
        pinocchio::SE3 end_effector_pose5 = data_fk_.oMi[ model_fk_.getJointId( fingertip[4] ) ];

        // 创建平移向量
        Eigen::Vector3d translation(request->x_base, request->y_base, request->z_base);

        // 创建旋转矩阵
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(request->roll_base, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(request->pitch_base, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(request->yaw_base, Eigen::Vector3d::UnitZ());
    
        // 设置基坐标系空间姿态
        pinocchio::SE3 base_to_world(rotation, translation);

        // 计算末端执行器相对于世界坐标系的位姿
        pinocchio::SE3 end1_effector_to_world = base_to_world * end_effector_pose1;
        pinocchio::SE3 end2_effector_to_world = base_to_world * end_effector_pose2;
        pinocchio::SE3 end3_effector_to_world = base_to_world * end_effector_pose3;
        pinocchio::SE3 end4_effector_to_world = base_to_world * end_effector_pose4;
        pinocchio::SE3 end5_effector_to_world = base_to_world * end_effector_pose5;

        // 构建一个 SE3 向量
        std::vector<pinocchio::SE3> end_effector_poses = {
            end1_effector_to_world,
            end2_effector_to_world,
            end3_effector_to_world,
            end4_effector_to_world,
            end5_effector_to_world
        };


        for (int i = 0; i < 5; i++)
        {
            response->x[i] = end_effector_poses[i].translation().x();
            response->y[i] = end_effector_poses[i].translation().y();
            response->z[i] = end_effector_poses[i].translation().z();
            Eigen::Quaterniond quat(end_effector_poses[i].rotation());
            response->w[i] = quat.w();
            response->i[i] = quat.x();
            response->j[i] = quat.y();
            response->k[i] = quat.z();
            response->roll[i]  = end_effector_poses[i].rotation().eulerAngles(0, 1, 2)[0];
            response->pitch[i] = end_effector_poses[i].rotation().eulerAngles(0, 1, 2)[1];
            response->yaw[i]   = end_effector_poses[i].rotation().eulerAngles(0, 1, 2)[2];
        }

    }


    // 运动学逆解
    void rh15_ctrl::rh15ik_callback(const rh15_cmd::srv::Rh15ik::Request::SharedPtr request, const rh15_cmd::srv::Rh15ik::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "rh15ik");

        Eigen::Vector3d target_pos[5];
        Eigen::Matrix3d target_rot[5];
        Eigen::Quaterniond target_quat[5];
        pinocchio::SE3 target_poses[5];

        if (request->lr)
        {
            model_ik_ = model_r_;
            data_ik_ = data_r_;
            fingertip[0] = fingertip_r_[0];
            fingertip[1] = fingertip_r_[1];
            fingertip[2] = fingertip_r_[2];
            fingertip[3] = fingertip_r_[3];
            fingertip[4] = fingertip_r_[4];
        }
        else
        {
            model_ik_ = model_l_;
            data_ik_ = data_l_;
            fingertip[0] = fingertip_l_[0];
            fingertip[1] = fingertip_l_[1];
            fingertip[2] = fingertip_l_[2];
            fingertip[3] = fingertip_l_[3];
            fingertip[4] = fingertip_l_[4];
        }

        // 创建平移向量
        Eigen::Vector3d translation(request->x_base, request->y_base, request->z_base);
        // 创建旋转矩阵
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(request->roll_base, Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(request->pitch_base, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(request->yaw_base, Eigen::Vector3d::UnitZ());
        // 设置基坐标系空间姿态
        pinocchio::SE3 base_to_world(rotation, translation);

        // 构建目标姿态
        for (int i = 0; i < 5; i++)
        {
            Eigen::Vector3d pos(request->x[i], request->y[i], request->z[i]);

            rotation = Eigen::AngleAxisd(request->roll[i], Eigen::Vector3d::UnitX()) *
                        Eigen::AngleAxisd(request->pitch[i], Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(request->yaw[i], Eigen::Vector3d::UnitZ());

            pinocchio::SE3 pose(rotation, pos);
            target_pos[i] = pose.translation();
            target_rot[i] = rotation;
            target_quat[i] = Eigen::Quaterniond(pose.rotation());
            target_poses[i] = pose;
        }

        #if 1

        // 计算 target_pose 相对于 base 坐标系的位姿
        for (int i = 0; i < 5; i++)
        {
            target_poses[i] = base_to_world.actInv(target_poses[i]);
        }

        #endif

        // 定义每个手指的关节索引（示例为单指，需扩展为多指）
        std::vector<int> active_joints  = { 2, 7, 12, 17, 22 }; // 主动关节 q2 的模型索引
        std::vector<int> passive_joints = { 3, 8, 13, 18, 23 }; // 随动关节 q3 的模型索引

        // 定义关节限制
        Eigen::VectorXd q_min = model_ik_.lowerPositionLimit;
        Eigen::VectorXd q_max = model_ik_.upperPositionLimit;

        double tolerance = 1e-2;
        int max_iter = 100;
        int iter = 0;
        double dt = 1e-1;
        double damping = 1e-2; // 定义阻尼项
        q_ik_ = q_;

        // 迭代求解 IK
        for (iter = 0; iter < max_iter; ++iter)
        {
            pinocchio::forwardKinematics(model_ik_, data_ik_, q_ik_);

            Eigen::VectorXd total_error(6 * 5);             // 5 个手指，每个手指 6 个自由度
            Eigen::MatrixXd total_J(6 * 5, model_ik_.nv);   // 5 个手指，每个手指 6 个自由度

            for (size_t finger_idx = 0; finger_idx < 5; ++finger_idx)
            {
                const auto& tip_name = fingertip[finger_idx];
                const auto& target_pose = target_poses[finger_idx];

                pinocchio::SE3 current_pose = data_ik_.oMi[model_ik_.getJointId(tip_name)];
                pinocchio::Motion error = pinocchio::log6(target_pose.actInv(current_pose));
                Eigen::VectorXd e = error.toVector();

                pinocchio::Data::Matrix6x J(6, model_ik_.nv);
                J.setZero();
                pinocchio::computeFrameJacobian(model_ik_, data_ik_, q_ik_, model_ik_.getJointId(tip_name), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

                // 合并随动关节的雅可比到主动关节
                for (size_t i = 0; i < active_joints.size(); ++i)
                {
                    int active = active_joints[i];
                    int passive = passive_joints[i];
                    J.col(active) += J.col(passive); // 叠加雅可比
                    J.col(passive).setZero();        // 随动关节不再独立
                }

                total_error.segment<6>(finger_idx * 6) = e;
                total_J.block<6, Eigen::Dynamic>(finger_idx * 6, 0, 6, model_ik_.nv) = J;
            }

            if (total_error.norm() < tolerance) break;

            // 伪逆求解（带阻尼项）
            Eigen::MatrixXd J_pinv = (total_J.transpose() * total_J + 
                                damping * Eigen::MatrixXd::Identity(model_ik_.nv, model_ik_.nv)).ldlt().solve(total_J.transpose());
            Eigen::VectorXd dq = -J_pinv * total_error * dt;

            // 更新关节角度
            q_ik_ = pinocchio::integrate(model_ik_, q_ik_, dq);

            // 裁剪主动关节角度
            for (int j = 0; j < model_ik_.nv; ++j)
            {
                q_ik_[j] = std::clamp(q_ik_[j], q_min[j], q_max[j]);
            }

            // 同步随动关节（例如 q3 = q2）
            for (size_t i = 0; i < active_joints.size(); ++i)
            {
                int active = active_joints[i];
                int passive = passive_joints[i];
                // 强制同步
                q_ik_[passive] = deg_to_rad(evaluatePolynomial(poly_coeff[i], 3, rad_to_deg(q_ik_[active])));
            }

            // 再次裁剪（确保同步后的随动关节不越界）
            for (int j = 0; j < model_ik_.nv; ++j)
            {
                q_ik_[j] = std::clamp(q_ik_[j], q_min[j], q_max[j]);
            }
        }
        // q_ = q_ik_;

        // 输出迭代次数和差值
        // RCLCPP_INFO(this->get_logger(), "Iteration: %d, Error Norm: %f", iter, total_error.norm());

        // 返回结果
        for (int i = 0; i < q_ik_.size(); i += 5 )
        {
            response->j_ang[ i/5 * 4 + 0 ] = q_ik_[ i + 0 ];
            response->j_ang[ i/5 * 4 + 1 ] = q_ik_[ i + 1 ];
            response->j_ang[ i/5 * 4 + 2 ] = q_ik_[ i + 2 ];
            response->j_ang[ i/5 * 4 + 3 ] = q_ik_[ i + 3 ];
        }


    }



    void rh15_ctrl::UpdataMotor( void )
    {
        for(int i = 0; i < 15; i++)
        {
            switch(i % 3)
            {
                case 0:
                    if( rh15msg.lr )
                    {
                        RyMotion_ServoMove_Mix( &stuServoCan, i+1, sutServoDataW[i+1].stuCmd.usTp, sutServoDataW[i+1].stuCmd.usTv, sutServoDataW[i+1].stuCmd.usTc, &sutServoDataR[i], 1);
                    }
                    else
                    {
                        RyMotion_ServoMove_Mix( &stuServoCan, i+1, sutServoDataW[i].stuCmd.usTp, sutServoDataW[i].stuCmd.usTv, sutServoDataW[i].stuCmd.usTc, &sutServoDataR[i], 1);
                    }
                    break;
        
                case 1:
                    if( rh15msg.lr )
                    {
                        RyMotion_ServoMove_Mix( &stuServoCan, i+1, sutServoDataW[i-1].stuCmd.usTp, sutServoDataW[i-1].stuCmd.usTv, sutServoDataW[i-1].stuCmd.usTc, &sutServoDataR[i], 1);
                    }
                    else
                    {
                        RyMotion_ServoMove_Mix( &stuServoCan, i+1, sutServoDataW[i].stuCmd.usTp, sutServoDataW[i].stuCmd.usTv, sutServoDataW[i].stuCmd.usTc, &sutServoDataR[i], 1);
                    }
                    break;
        
                case 2:
                    RyMotion_ServoMove_Mix( &stuServoCan, i+1, sutServoDataW[i].stuCmd.usTp, sutServoDataW[i].stuCmd.usTv, sutServoDataW[i].stuCmd.usTc, &sutServoDataR[i], 1);
                    break;
        
                default:
                    break;
            }
        } 

    }


    void rh15_ctrl::CmdCallback(const rh15_cmd::msg::Rh15Cmd::SharedPtr msg)
    {
        rh15_cmd::msg::Rh15Cmd cmd = *msg;
        int p1, p2, p3, p4, p5, p6;
    
        rh15msg.lr = cmd.lr; // left_or_right
    
        if (memcmp(&rh15cmd, &cmd, sizeof(rh15_cmd::msg::Rh15Cmd)))
        {
            for (int i = 0; i < 15; i++)
            {
                sutServoDataW[i].stuCmd.usTp = cmd.m_pos[i];
                sutServoDataW[i].stuCmd.usTv = cmd.m_spd[i];
                sutServoDataW[i].stuCmd.usTc = cmd.m_curlimit[i];
            }
    
            switch (cmd.mode)
            {
                // end pos cmd
                case 2:
                {
                    Eigen::Vector3d target_pos[5];
                    Eigen::Matrix3d target_rot[5];
                    Eigen::Quaterniond target_quat[5];
                    pinocchio::SE3 target_poses[5];
    
                    if (rh15msg.lr)
                    {
                        model_ = model_r_;
                        data_ = data_r_;
                        fingertip[0] = fingertip_r_[0];
                        fingertip[1] = fingertip_r_[1];
                        fingertip[2] = fingertip_r_[2];
                        fingertip[3] = fingertip_r_[3];
                        fingertip[4] = fingertip_r_[4];
                    }
                    else
                    {
                        model_ = model_l_;
                        data_ = data_l_;
                        fingertip[0] = fingertip_l_[0];
                        fingertip[1] = fingertip_l_[1];
                        fingertip[2] = fingertip_l_[2];
                        fingertip[3] = fingertip_l_[3];
                        fingertip[4] = fingertip_l_[4];
                    }
    
                    // 设置基坐标系空间姿态
                    rh15msg.x_base = cmd.x_base;
                    rh15msg.y_base = cmd.y_base;
                    rh15msg.z_base = cmd.z_base;
                    rh15msg.roll_base = cmd.roll_base;
                    rh15msg.pitch_base = cmd.pitch_base;
                    rh15msg.yaw_base = cmd.yaw_base;
    
                    // 创建平移向量
                    Eigen::Vector3d translation(cmd.x_base, cmd.y_base, cmd.z_base);
                    // 创建旋转矩阵
                    Eigen::Matrix3d rotation;
                    rotation = Eigen::AngleAxisd(cmd.roll_base, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(cmd.pitch_base, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(cmd.yaw_base, Eigen::Vector3d::UnitZ());
                    // 设置基坐标系空间姿态
                    pinocchio::SE3 base_to_world(rotation, translation);
    
                    // 构建目标姿态
                    for (int i = 0; i < 5; i++)
                    {
                        Eigen::Vector3d pos(cmd.x[i], cmd.y[i], cmd.z[i]);
    
                        rotation = Eigen::AngleAxisd(cmd.roll[i], Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(cmd.pitch[i], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(cmd.yaw[i], Eigen::Vector3d::UnitZ());
    
                        pinocchio::SE3 pose(rotation, pos);
                        target_pos[i] = pose.translation();
                        target_rot[i] = rotation;
                        target_quat[i] = Eigen::Quaterniond(pose.rotation());
                        target_poses[i] = pose;
                    }
    
                    #if 1
    
                    // 计算 target_pose 相对于 base 坐标系的位姿
                    for (int i = 0; i < 5; i++)
                    {
                        target_poses[i] = base_to_world.actInv(target_poses[i]);
                    }
    
                    #endif
    
                    // 定义每个手指的关节索引（示例为单指，需扩展为多指）
                    std::vector<int> active_joints  = { 2,  7, 12, 17, 22 }; // 主动关节 q2 的模型索引
                    std::vector<int> passive_joints = { 3,  8, 13, 18, 23 }; // 随动关节 q3 的模型索引
    
                    // 定义关节限制
                    Eigen::VectorXd q_min = model_.lowerPositionLimit;
                    Eigen::VectorXd q_max = model_.upperPositionLimit;
    
                    double tolerance = 1e-2;
                    int max_iter = 1000;
                    double dt = 1e-1;
                    double damping = 1e-2; // 定义阻尼项
                    Eigen::VectorXd q_ik = q_;
    
                    // 迭代求解 IK
                    for (int iter = 0; iter < max_iter; ++iter)
                    {
                        pinocchio::forwardKinematics(model_, data_, q_ik);
    
                        Eigen::VectorXd total_error(6 * 5); // 5 个手指，每个手指 6 个自由度
                        Eigen::MatrixXd total_J(6 * 5, model_.nv); // 5 个手指，每个手指 6 个自由度
    
                        for (size_t finger_idx = 0; finger_idx < 5; ++finger_idx)
                        {
                            const auto& tip_name = fingertip[finger_idx];
                            const auto& target_pose = target_poses[finger_idx];
    
                            pinocchio::SE3 current_pose = data_.oMi[model_.getJointId(tip_name)];
                            pinocchio::Motion error = pinocchio::log6(target_pose.actInv(current_pose));
                            Eigen::VectorXd e = error.toVector();
    
                            pinocchio::Data::Matrix6x J(6, model_.nv);
                            J.setZero();
                            pinocchio::computeFrameJacobian(model_, data_, q_ik, model_.getJointId(tip_name), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
    
                            // 合并随动关节的雅可比到主动关节
                            for (size_t i = 0; i < active_joints.size(); ++i)
                            {
                                int active = active_joints[i];
                                int passive = passive_joints[i];
                                J.col(active) += J.col(passive); // 叠加雅可比
                                J.col(passive).setZero();        // 随动关节不再独立
                            }
    
                            total_error.segment<6>(finger_idx * 6) = e;
                            total_J.block<6, Eigen::Dynamic>(finger_idx * 6, 0, 6, model_.nv) = J;
                        }
    
                        if (total_error.norm() < tolerance) break;
    
                        // 伪逆求解（带阻尼项）
                        Eigen::MatrixXd J_pinv = (total_J.transpose() * total_J +
                                                    damping * Eigen::MatrixXd::Identity(model_.nv, model_.nv))
                                                    .ldlt().solve(total_J.transpose());
                        Eigen::VectorXd dq = -J_pinv * total_error * dt;
    
                        // 更新关节角度
                        q_ik = pinocchio::integrate(model_, q_ik, dq);
    
                        // 裁剪主动关节角度
                        for (int j = 0; j < model_.nv; ++j)
                        {
                            q_ik[j] = std::clamp(q_ik[j], q_min[j], q_max[j]);
                        }
    
                        // 同步随动关节（例如 q3 = q2）
                        for (size_t i = 0; i < active_joints.size(); ++i)
                        {
                            int active = active_joints[i];
                            int passive = passive_joints[i];
                            // 强制同步
                            q_ik[passive] = deg_to_rad(evaluatePolynomial(poly_coeff[i], 3, rad_to_deg(q_ik[active])));
                        }
    
                        // 再次裁剪（确保同步后的随动关节不越界）
                        for (int j = 0; j < model_.nv; ++j)
                        {
                            q_ik[j] = std::clamp(q_ik[j], q_min[j], q_max[j]);
                        }
                    }
    
                    q_ = q_ik;
    
                    for ( int i = 0; i < q_.size(); i += 5 )
                    {
                        cmd.j_ang[ i/5 * 4 + 0 ] = q_[ i + 0 ];
                        cmd.j_ang[ i/5 * 4 + 1 ] = q_[ i + 1 ];
                        cmd.j_ang[ i/5 * 4 + 2 ] = q_[ i + 2 ];
                        cmd.j_ang[ i/5 * 4 + 3 ] = q_[ i + 3 ];
                    }
    
                    for (int i = 0; i < 20; i++)
                    {
                        if (i % 4 == 3)
                        {
                            p1 = map_rad_to_value_full_range(cmd.j_ang[i / 4 * 4 + 0]);
                            p2 = map_rad90_to_value(cmd.j_ang[i / 4 * 4 + 1]);
                            p3 = map_rad75_to_value(cmd.j_ang[i / 4 * 4 + 2]);
                            p4 = map_rad75_to_value(cmd.j_ang[i / 4 * 4 + 3]);
                            (void)p4;
    
                            if (p1 > 4095 / 4) p1 = 4095 / 4;
                            if (p1 < -4095 / 4) p1 = -4095 / 4;
    
                            // if( !cmd.lr )
                            // {
                            p5 = p2 - p1 / 2;
                            p6 = p2 + p1 / 2;
                            // }
                            // else
                            // {
                            //     p5 = p2 + p1 / 2;
                            //     p6 = p2 - p1 / 2;
                            // }
    
                            if (p5 > 4095) p5 = 4095;
                            else if (p5 < 0) p5 = 0;
    
                            if (p6 > 4095) p6 = 4095;
                            else if (p6 < 0) p6 = 0;
    
                            if (p3 > 4095) p3 = 4095;
                            else if (p3 < 0) p3 = 0;
    
                            sutServoDataW[(i / 4 * 3) + 0].stuCmd.usTp = p5;
                            sutServoDataW[(i / 4 * 3) + 1].stuCmd.usTp = p6;
                            sutServoDataW[(i / 4 * 3) + 2].stuCmd.usTp = p3;
                        }
                    }
                    UpdataMotor();
                }
                break;
    
                // rad cmd
                case 1:
                {
                    for (int i = 0; i < 20; i++)
                    {
                        if (i % 4 == 3)
                        {
                            p1 = map_rad_to_value_full_range(cmd.j_ang[i / 4 * 4 + 0]);
                            p2 = map_rad90_to_value(cmd.j_ang[i / 4 * 4 + 1]);
                            p3 = map_rad75_to_value(cmd.j_ang[i / 4 * 4 + 2]);
                            p4 = map_rad75_to_value(cmd.j_ang[i / 4 * 4 + 3]);
                            (void)p4;
    
                            if (p1 > 4095 / 4) p1 = 4095 / 4;
                            if (p1 < -4095 / 4) p1 = -4095 / 4;
    
                            // if( !cmd.lr )
                            // {
                            p5 = p2 - p1 / 2;
                            p6 = p2 + p1 / 2;
                            // }
                            // else
                            // {
                            //     p5 = p2 + p1 / 2;
                            //     p6 = p2 - p1 / 2;
                            // }
    
                            if (p5 > 4095) p5 = 4095;
                            else if (p5 < 0) p5 = 0;
    
                            if (p6 > 4095) p6 = 4095;
                            else if (p6 < 0) p6 = 0;
    
                            if (p3 > 4095) p3 = 4095;
                            else if (p3 < 0) p3 = 0;
    
                            sutServoDataW[(i / 4 * 3) + 0].stuCmd.usTp = p5;
                            sutServoDataW[(i / 4 * 3) + 1].stuCmd.usTp = p6;
                            sutServoDataW[(i / 4 * 3) + 2].stuCmd.usTp = p3;
                        }
                    }
                    UpdataMotor();
                }
                break;
    
                // raw cmd
                case 0:
                {
                    UpdataMotor();
                }
                break;    
    
                default: break;
            }
    
            rh15cmd = cmd;
        }
    }

   
    void rh15_ctrl::PubState()
    {
        // RCLCPP_INFO(this->get_logger(), "cnt = %d",cnt++);
        int p, v, t, pcom, perr;

        // RyFunc_GetServoInfo( &stuServoCan, si+1, &sutServoDataR[si], 0 );
        // si = (si+1)%15;

        for (int i = 0; i < 15; i++)
        {
            rh15msg.status[i] = sutServoDataR[i].stuInfo.ucStatus;
            
            p = sutServoDataR[i].stuInfo.ub_P;
            v = sutServoDataR[i].stuInfo.ub_V;
            t = sutServoDataR[i].stuInfo.ub_I;
            if(v>2047) v -= 4096;
            if(t>2047) t -= 4096;

            rh15msg.m_pos[i] = p;
            rh15msg.m_spd[i] = v;
            rh15msg.m_cur[i] = t;
            rh15msg.m_force[i] = sutServoDataR[i].stuInfo.ub_F;

            switch (i % 3)
            {
            case 2:
                if( rh15msg.lr )
                {
                    p = rh15msg.m_pos[i - 2];
                    rh15msg.m_pos[i - 2] = rh15msg.m_pos[i - 1];
                    rh15msg.m_pos[i - 1] = p;

                    v = rh15msg.m_spd[i - 2];
                    rh15msg.m_spd[i - 2] = rh15msg.m_spd[i - 1];
                    rh15msg.m_spd[i - 1] = v;

                    t = rh15msg.m_cur[i - 2];
                    rh15msg.m_cur[i - 2] = rh15msg.m_cur[i - 1];
                    rh15msg.m_cur[i - 1] = t;
                }

                pcom  = (rh15msg.m_pos[i-1] + rh15msg.m_pos[i-2]) / 2;
                perr  = (rh15msg.m_pos[i-1] - rh15msg.m_pos[i-2]);
                rh15msg.j_ang[ (i/3 * 4) + 0] = value_to_rad_full_range(perr);
                rh15msg.j_ang[ (i/3 * 4) + 1] = value_to_rad90(pcom);
                p = rh15msg.m_pos[i];
                rh15msg.j_ang[ (i/3 * 4) + 2] = value_to_rad75(p);
                rh15msg.j_ang[ (i/3 * 4) + 3] = deg_to_rad( evaluatePolynomial( poly_coeff[i/3], 3,  rad_to_deg( value_to_rad75(p) ) ) ); 
                break;

            default:
                break;
            }
        }


 

        // 正解FK
        if( rh15msg.lr )
        {
            model_ = model_r_;
            data_ = data_r_;
            fingertip[0] = fingertip_r_[0];
            fingertip[1] = fingertip_r_[1];
            fingertip[2] = fingertip_r_[2];
            fingertip[3] = fingertip_r_[3];
            fingertip[4] = fingertip_r_[4];
        }
        else
        {
            model_ = model_l_;
            data_ = data_l_;
            fingertip[0] = fingertip_l_[0];
            fingertip[1] = fingertip_l_[1];
            fingertip[2] = fingertip_l_[2];
            fingertip[3] = fingertip_l_[3];
            fingertip[4] = fingertip_l_[4];
        }


        for (int i = 0; i < 20; i += 4 )
        {
            q_[ i/4*5 + 0 ] = rh15msg.j_ang[ i + 0 ];
            q_[ i/4*5 + 1 ] = rh15msg.j_ang[ i + 1 ];
            q_[ i/4*5 + 2 ] = rh15msg.j_ang[ i + 2 ];
            q_[ i/4*5 + 3 ] = rh15msg.j_ang[ i + 3 ];
        }


        // Perform the forward kinematics over the kinematic tree
        forwardKinematics(model_, data_, q_);

        // 获取末 关节坐标系的位姿
        pinocchio::SE3 end_effector_pose1 = data_.oMi[ model_.getJointId( fingertip[0] ) ];
        pinocchio::SE3 end_effector_pose2 = data_.oMi[ model_.getJointId( fingertip[1] ) ];
        pinocchio::SE3 end_effector_pose3 = data_.oMi[ model_.getJointId( fingertip[2] ) ];
        pinocchio::SE3 end_effector_pose4 = data_.oMi[ model_.getJointId( fingertip[3] ) ];
        pinocchio::SE3 end_effector_pose5 = data_.oMi[ model_.getJointId( fingertip[4] ) ];


        rh15msg.x_base = rh15cmd.x_base;
        rh15msg.y_base = rh15cmd.y_base;
        rh15msg.z_base = rh15cmd.z_base;
        rh15msg.roll_base = rh15cmd.roll_base;
        rh15msg.pitch_base = rh15cmd.pitch_base;
        rh15msg.yaw_base = rh15cmd.yaw_base;

        // 创建平移向量
        Eigen::Vector3d translation(rh15msg.x_base, rh15msg.y_base, rh15msg.z_base);

        // 创建旋转矩阵
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(rh15msg.roll_base, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(rh15msg.pitch_base, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rh15msg.yaw_base, Eigen::Vector3d::UnitZ());
    
        // 设置基坐标系空间姿态
        pinocchio::SE3 base_to_world(rotation, translation);


        // 计算末端执行器相对于世界坐标系的位姿
        pinocchio::SE3 end1_effector_to_world = base_to_world * end_effector_pose1;
        pinocchio::SE3 end2_effector_to_world = base_to_world * end_effector_pose2;
        pinocchio::SE3 end3_effector_to_world = base_to_world * end_effector_pose3;
        pinocchio::SE3 end4_effector_to_world = base_to_world * end_effector_pose4;
        pinocchio::SE3 end5_effector_to_world = base_to_world * end_effector_pose5;

        // 构建一个 SE3 向量
        std::vector<pinocchio::SE3> end_effector_poses = {
            end1_effector_to_world,
            end2_effector_to_world,
            end3_effector_to_world,
            end4_effector_to_world,
            end5_effector_to_world
        };


        for (int i = 0; i < 5; i++)
        {
            rh15msg.x[i] = end_effector_poses[i].translation().x();
            rh15msg.y[i] = end_effector_poses[i].translation().y();
            rh15msg.z[i] = end_effector_poses[i].translation().z();
            Eigen::Quaterniond quat(end_effector_poses[i].rotation());
            rh15msg.w[i] = quat.w();
            rh15msg.i[i] = quat.x();
            rh15msg.j[i] = quat.y();
            rh15msg.k[i] = quat.z();

            rh15msg.roll[i]  = end_effector_poses[i].rotation().eulerAngles(0, 1, 2)[0];
            rh15msg.pitch[i] = end_effector_poses[i].rotation().eulerAngles(0, 1, 2)[1];
            rh15msg.yaw[i]   = end_effector_poses[i].rotation().eulerAngles(0, 1, 2)[2];

        }

        // 发布消息
        ryhand_state_publisher_->publish(rh15msg);

    }


}




void check_thread_priority() 
{
    pthread_t thread_handle = pthread_self(); // 获取当前线程句柄
    int policy;
    struct sched_param sch_params;

    // 获取线程的调度参数
    if (pthread_getschedparam(thread_handle, &policy, &sch_params) == 0) 
    {
        // 打印调度策略
        if (policy == SCHED_FIFO) 
        {
            printf("线程调度策略: SCHED_FIFO\n");
        } 
        else if (policy == SCHED_RR) 
        {
            printf("线程调度策略: SCHED_RR\n");
        } 
        else if (policy == SCHED_OTHER) 
        {
            printf("线程调度策略: SCHED_OTHER\n");
        } 
        else 
        {
            printf("线程调度策略: 未知\n");
        }

        // 打印线程优先级
        printf("线程优先级: %d\n", sch_params.sched_priority);
    }
    else 
    {
        perror("无法获取线程优先级");

    }
}



// 用一个高优先级的线程来模拟用来生成 ms 系统时间节排 uwTick
void BusReadAnduwTickTask()
{

    check_thread_priority();


    while ( thread_go ) 
    {
        // 在此处放置高优先级线程要执行的工作
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        uwTick = static_cast<unsigned int>(now_ms) % 1000;


#if 0
        // 读取CAN总线数据
        TPCANMsg pcan_msg;
        TPCANTimestamp pcan_timestamp;
        int result = objPCANBasic_Read(PcanHandle, &pcan_msg, &pcan_timestamp);

        if (result == 0) 
        {   
            // PCAN_ERROR_OK
            // 打印接收到的消息 ID 和数据（实际使用时关闭该打印）
            printf("Rx> %04X ：", pcan_msg.ID);
            for (int i = 0; i < pcan_msg.LEN; i++) 
            {
                printf("%02X ", pcan_msg.DATA[i]);
            }
            printf("\n");

            // 将接收的消息转换为 CanMsg 格式
            CanMsg_t received_msg;
            received_msg.ulId = pcan_msg.ID;
            received_msg.ucLen = pcan_msg.LEN;
            memcpy(received_msg.pucDat, pcan_msg.DATA, pcan_msg.LEN);

            // 调用处理函数
            RyCanServoLibRcvMsg( &stuServoCan, received_msg);
        } 

#else

        struct can_frame frame;
        if( (sock>0) && receive_can_message(sock, &frame) )
        {
            // 将接收的消息转换为 CanMsg 格式
            CanMsg_t received_msg;
            memset (&received_msg, 0, sizeof(CanMsg_t));
            received_msg.ulId = frame.can_id;
            received_msg.ucLen = frame.can_dlc;
            memcpy(received_msg.pucDat,frame.data, frame.can_dlc);

            // 调用处理函数
            RyCanServoLibRcvMsg( &stuServoCan, received_msg);
        }

#endif

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}





// 定义线程函数
void* CanRx_and_uwTick_thread(void* arg) 
{
    (void)arg;

#ifdef __linux__
    pthread_t thread_handle = pthread_self();
    struct sched_param sch_params;

    // 设置线程优先级
    sch_params.sched_priority = 80; // 根据需要设置优先级
    if (pthread_setschedparam(thread_handle, SCHED_FIFO, &sch_params)) 
    {
        perror("高优先级线程创建失败");
    } 
    else 
    {
        printf("高优先级线程设置成功\n");
    }
#endif

    // 执行目标任务
    BusReadAnduwTickTask();


    return NULL;

}




int main(int argc, char *argv[])
{
    u8_t ret = 0;
    u8_t i = 0;


    // 打开 CAN 设备
    if( !open_can_socket(&sock, &addr, &ifr) )
    {
        printf("open can socket failed\n");
        sock = 0;
    }  
    printf("sock = %d\n",sock);

	// 复位stuServoCan内容
	memset( &stuServoCan ,0,sizeof(RyCanServoBus_t));

	// 指定最大支持的Hook数，由用户根据实际应用情况来指定个数，建议给定值大于2 ，（一个总线至少需一个Hook，用户可以不给定，库内部最少会申请一个Hook）
	stuServoCan.usHookNum = 5;                                                 	 		 
	
	// 申请并指定所需的Hook数据空间，下面二行操作用户可以不做，RyCanServoBusInit 会自动申请，但程序栈必需足够
	stuServoCan.pstuHook = (MsgHook_t *)malloc(stuServoCan.usHookNum * sizeof(MsgHook_t));  		   
	memset(stuServoCan.pstuHook, 0, stuServoCan.usHookNum * sizeof(MsgHook_t));      
		
	// 指定最大支持的listen数,由用户根据实际应用情况来指定个数，如果需要实现伺服电机主动上报功能，则需给定足够的listen，一个伺服电机需要一个listen	
	stuServoCan.usListenNum = 31+1;                                                  
	// 申请并指定所需的listen数据空间，下面二行操作用户可以不做，RyCanServoBusInit 会自动申请，但程序栈必需足够
	stuServoCan.pstuListen = (MsgListen_t *)malloc(stuServoCan.usListenNum * sizeof(MsgListen_t));  
	memset(stuServoCan.pstuListen, 0, stuServoCan.usListenNum * sizeof(MsgListen_t)); 
	
	// 初始化库, 内部会使用 malloc 需保证有足够的栈空间，请查看栈空间设置
	ret = RyCanServoBusInit( &stuServoCan, BusWrite, (volatile u16_t *)&uwTick, 1000 );
	
	if( ret == 0 )
	{
        for( i=0;i<15;i++ )
        {
            // 这里要注意，每个Listen或Hook都要为分配一个CanMsg_t对象，如果对个Listen或Hook 用同一个CanMsg_t对象，
            // 效果等同于只有一个 Listen或Hook
            stuListenMsg[i].ulId = SERVO_BACK_ID(i+1);
            stuListenMsg[i].pucDat[0] = 0xAA;

            // 添加监听，也可以为每个监听对象添加一个回调函数，这样每个监听对象的回调函数可以不同
            ret = AddListen( &stuServoCan,stuListenMsg + i, CallBck0 );
        }

        for( i=0;i<15;i++ )
        {
            // 这里要注意，每个Listen或Hook都要为分配一个CanMsg_t对象，如果对个Listen或Hook 用同一个CanMsg_t对象，
            // 效果等同于只有一个 Listen或Hook
            stuListenMsg[15+i].ulId = SERVO_BACK_ID(i+1);
            stuListenMsg[15+i].pucDat[0] = 0xA0;

            // 添加监听，也可以为每个监听对象添加一个回调函数，这样每个监听对象的回调函数可以不同
            ret = AddListen( &stuServoCan,stuListenMsg + i + 15, CallBck0 );
        }
	}

    sutServoDataW[0].pucDat[0] = 0xaa;
	sutServoDataW[0].stuCmd.usTp = 4095;
	sutServoDataW[0].stuCmd.usTv = 1000;
	sutServoDataW[0].stuCmd.usTc = 80;		
	for( i = 1; i < 15; i++ )
	{
		sutServoDataW[i] = sutServoDataW[0];
	}

    // ROS2 初始化
    rclcpp::init(argc, argv);

    // 创建线程
    thread_go = 1;
    if (pthread_create(&thread_id, NULL, CanRx_and_uwTick_thread, NULL)) {
        perror("线程创建失败");
        return EXIT_FAILURE;
    }

    // 分离线程
    if (pthread_detach(thread_id)) {
        perror("线程分离失败");
        return EXIT_FAILURE;
    }

    // 清除错误
    RyParam_ClearFault( &stuServoCan, 0, 1 );

    auto node =  std::make_shared<ruiyan::rh15::rh15_ctrl>("rh15");
    rclcpp::executors::MultiThreadedExecutor  exector;
    exector.add_node(node);
    exector.spin();

    // 关闭 ROS2
    rclcpp::shutdown();

    thread_go = 0;

    // 关闭 CAN 套接字
    if( sock > 0 ) close_can_socket(sock);


    return 0;
}





