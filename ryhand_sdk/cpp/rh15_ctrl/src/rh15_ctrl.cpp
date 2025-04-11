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
#include <bitset>

extern "C" {
   #include <ryhandlib_port.h>
   #include <can_socket.h>
}





pthread_t thread_id;
volatile int  thread_go = 1;

using namespace std::chrono_literals;



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

        q_iik_.resize(model_l_.nq);
        q_iik_.setZero();


        RCLCPP_INFO(this->get_logger(), "joint_num: %d.",model_l_.nq);
        RCLCPP_INFO(this->get_logger(), "urdf_l_path: %s.",urdf_filename_l.c_str());
        RCLCPP_INFO(this->get_logger(), "urdf_r_path: %s.",urdf_filename_r.c_str());
        RCLCPP_INFO(this->get_logger(), "end: %s. id is %d", fingertip_r_[0].c_str(), model_r_.getJointId( fingertip_r_[0] )); 


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
            fingertip_fk[0] = fingertip_r_[0];
            fingertip_fk[1] = fingertip_r_[1];
            fingertip_fk[2] = fingertip_r_[2];
            fingertip_fk[3] = fingertip_r_[3];
            fingertip_fk[4] = fingertip_r_[4];
        }
        else
        {
            model_fk_ = model_l_;
            data_fk_ = data_l_;
            fingertip_fk[0] = fingertip_l_[0];
            fingertip_fk[1] = fingertip_l_[1];
            fingertip_fk[2] = fingertip_l_[2];
            fingertip_fk[3] = fingertip_l_[3];
            fingertip_fk[4] = fingertip_l_[4];
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
        pinocchio::SE3 end_effector_pose1 = data_fk_.oMi[ model_fk_.getJointId( fingertip_fk[0] ) ];
        pinocchio::SE3 end_effector_pose2 = data_fk_.oMi[ model_fk_.getJointId( fingertip_fk[1] ) ];
        pinocchio::SE3 end_effector_pose3 = data_fk_.oMi[ model_fk_.getJointId( fingertip_fk[2] ) ];
        pinocchio::SE3 end_effector_pose4 = data_fk_.oMi[ model_fk_.getJointId( fingertip_fk[3] ) ];
        pinocchio::SE3 end_effector_pose5 = data_fk_.oMi[ model_fk_.getJointId( fingertip_fk[4] ) ];

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


        req_fk = *request;
        res_fk = *response;


    }



    // 运动学逆解
    void rh15_ctrl::rh15ik_callback(const rh15_cmd::srv::Rh15ik::Request::SharedPtr request, const rh15_cmd::srv::Rh15ik::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "rh15ik");

        if (request->lr)
        {
            model_ik_ = model_r_;
            data_ik_ = data_r_;
            fingertip_ik[0] = fingertip_r_[0];
            fingertip_ik[1] = fingertip_r_[1];
            fingertip_ik[2] = fingertip_r_[2];
            fingertip_ik[3] = fingertip_r_[3];
            fingertip_ik[4] = fingertip_r_[4];
        }
        else
        {
            model_ik_ = model_l_;
            data_ik_ = data_l_;
            fingertip_ik[0] = fingertip_l_[0];
            fingertip_ik[1] = fingertip_l_[1];
            fingertip_ik[2] = fingertip_l_[2];
            fingertip_ik[3] = fingertip_l_[3];
            fingertip_ik[4] = fingertip_l_[4];
        }

        rh15ik( model_ik_, data_ik_, q_ik_, fingertip_ik, request, response);
        
    }


    void rh15_ctrl::rh15ik( pinocchio::Model& model, pinocchio::Data& data, Eigen::VectorXd &q_ik, std::string ftip[5], const rh15_cmd::srv::Rh15ik::Request::SharedPtr request, const rh15_cmd::srv::Rh15ik::Response::SharedPtr response)
    {

        Eigen::Vector3d target_pos[5];
        Eigen::Matrix3d target_rot[5];
        pinocchio::SE3 target_poses[5];

        // 创建基坐标系位姿
        Eigen::Vector3d translation(request->x_base, request->y_base, request->z_base);
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(request->roll_base, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(request->pitch_base, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(request->yaw_base, Eigen::Vector3d::UnitZ());
        pinocchio::SE3 base_to_world(rotation, translation);

        // 构建目标姿态
        for (int i = 0; i < 5; i++)
        {
            Eigen::Vector3d pos(request->x[i], request->y[i], request->z[i]);
            rotation = Eigen::AngleAxisd(request->roll[i], Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(request->pitch[i], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(request->yaw[i], Eigen::Vector3d::UnitZ());

            // Eigen::Vector3d pos(res_fk.x[i], res_fk.y[i], res_fk.z[i]);
            // rotation = Eigen::AngleAxisd(res_fk.roll[i], Eigen::Vector3d::UnitX()) *
            //     Eigen::AngleAxisd(res_fk.pitch[i], Eigen::Vector3d::UnitY()) *
            //     Eigen::AngleAxisd(res_fk.yaw[i], Eigen::Vector3d::UnitZ());


            pinocchio::SE3 pose(rotation, pos);
            target_pos[i] = pose.translation();
            target_rot[i] = rotation;
            target_poses[i] = base_to_world.actInv(pose); // 转换为基坐标系下的位姿
        }


        // 灵巧手参数配置
        const std::vector<std::pair<int, int>> coupled_joints = {{2,3}, {7,8}, {12,13}, {17,18}, {22,23}};
        const std::vector<int> fixed_joints = {4,9,14,19,24};
        Eigen::VectorXd minq = model.lowerPositionLimit;
        Eigen::VectorXd maxq = model.upperPositionLimit;


        // 初始化关节角度
        q_ik = pinocchio::neutral(model);
        const double eps = 1e-4;    // 收敛误差阈值
        const int IT_MAX = 50;      // 最大迭代次数
        const double DT = 0.5;      // 时间步长
        const double damp = 1e-6;   // 阻尼系数

        // 定义逆运动学求解函数
        auto solve_finger_ik = [&](int finger_idx, const pinocchio::SE3& oMdes, int target_joint_id, Eigen::VectorXd& q_ik, bool * success) 
        {
            // 为每个线程创建独立的 Pinocchio 数据对象，避免线程冲突
            pinocchio::Data local_data(model);
            pinocchio::Data::Matrix6x J(6, model.nv);
            J.setZero();
            Eigen::VectorXd v(model.nv);
            Eigen::Matrix<double, 6, 1> err;

            // 迭代求解
            for (int i = 0;; i++)
            {
                pinocchio::forwardKinematics(model, local_data, q_ik);
                const pinocchio::SE3 iMd = local_data.oMi[target_joint_id].actInv(oMdes);
                err = pinocchio::log6(iMd).toVector();

                if (err.norm() < eps)
                {
                    *success = true;
                    break;
                }
                if (i >= IT_MAX)
                {
                    *success = false;
                    break;
                }

                pinocchio::computeJointJacobian(model, local_data, q_ik, target_joint_id, J);
                for (auto [master, slave] : coupled_joints)
                {
                    J.col(master) += J.col(slave);
                    J.col(slave).setZero();
                }
                for (int id : fixed_joints) J.col(id).setZero();

                pinocchio::Data::Matrix6 Jlog;
                pinocchio::Jlog6(iMd.inverse(), Jlog);
                J = -Jlog * J;

                pinocchio::Data::Matrix6 JJt = J * J.transpose();
                JJt.diagonal().array() += damp;
                v.noalias() = -J.transpose() * JJt.ldlt().solve(err);

                for (auto [master, slave] : coupled_joints) v[slave] = v[master];
                for (int id : fixed_joints) v[id] = 0.0;

                q_ik = pinocchio::integrate(model, q_ik, v * DT);
                for (int i = 0; i < q_ik.size(); i++)
                    q_ik[i] = std::clamp(q_ik[i], minq[i], maxq[i]);

                for (auto [master, slave] : coupled_joints) 
                    q_ik[slave] = deg_to_rad( evaluatePolynomial( poly_coeff[finger_idx], 3,  rad_to_deg( q_ik[master] ) ) ); 

                for (int id : fixed_joints) q_ik[id] = 0.0;

                for (int i = 0; i < q_ik.size(); i++)
                    q_ik[i] = std::clamp(q_ik[i], minq[i], maxq[i]);
            }
        };

        // 创建线程并行求解
        std::vector<std::thread> threads;
        bool successes[5];
        std::vector<Eigen::VectorXd> q_ik_per_finger(5, q_ik); // 为每个手指分配独立的关节角度向量

        auto start_time = std::chrono::high_resolution_clock::now();
        for (int finger_idx = 0; finger_idx < 5; ++finger_idx)
        {
            int target_joint_id = model.getJointId( ftip[finger_idx] );
            const pinocchio::SE3& oMdes = target_poses[finger_idx];
            threads.emplace_back( solve_finger_ik, finger_idx, std::cref(oMdes), target_joint_id, std::ref(q_ik_per_finger[finger_idx]), &successes[finger_idx] );
        }

        // 等待所有线程完成
        for (auto& thread : threads)
        {
            thread.join();
        }

        // 合并结果到 q_ik
        for (int finger_idx = 0; finger_idx < 5; ++finger_idx)
        {
            int offset = finger_idx * 5; // 假设每个手指有5个关节
            for (int j = 0; j < 5 && offset + j < q_ik.size(); ++j)
            {
                q_ik[offset + j] = q_ik_per_finger[finger_idx][offset + j];
            }
        }

        // 计算执行时间
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        // 检查求解成功状态
        char finger_done = 0;
        for (int i = 0; i < 5; ++i)
        {
            if (successes[i])
            {
                finger_done |= (1 << i);
            }
        }

        // 输出结果
        // std::cout << "Execution time: " << duration << " ms" << std::endl;
        // std::cout << "finger_done (binary): " << std::bitset<5>(finger_done) << std::endl;

        // 返回结果
        for (int i = 0; i < q_ik.size(); i += 5)
        {
            response->j_ang[i/5 * 4 + 0] = q_ik[i + 0];
            response->j_ang[i/5 * 4 + 1] = q_ik[i + 1];
            response->j_ang[i/5 * 4 + 2] = q_ik[i + 2];
            response->j_ang[i/5 * 4 + 3] = q_ik[i + 3];
        }

        // req_ik = *request;
        // res_ik = *response;

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

                    req_ik.lr = rh15msg.lr;
                    req_ik.x_base = rh15msg.x_base;
                    req_ik.y_base = rh15msg.y_base;
                    req_ik.z_base = rh15msg.z_base;
                    req_ik.roll_base = rh15msg.roll_base;
                    req_ik.pitch_base = rh15msg.pitch_base;
                    req_ik.yaw_base = rh15msg.yaw_base;
                    for (int i = 0; i < 5; i++)
                    {
                        req_ik.x[i] = cmd.x[i];
                        req_ik.y[i] = cmd.y[i];
                        req_ik.z[i] = cmd.z[i];
                        req_ik.roll[i] = cmd.roll[i];
                        req_ik.pitch[i] = cmd.pitch[i];
                        req_ik.yaw[i] = cmd.yaw[i];
                    }
                    rh15ik(model_, data_, q_iik_, fingertip, std::make_shared<rh15_cmd::srv::Rh15ik::Request>(req_ik), std::make_shared<rh15_cmd::srv::Rh15ik::Response>(res_ik));
                    for ( int i = 0; i < 20; i ++ )
                    {
                        cmd.j_ang[ i ] = res_ik.j_ang[ i ];
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





