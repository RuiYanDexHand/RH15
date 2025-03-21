#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <array>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp> 
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Dense>

#include "rh15_cmd/msg/rh15_cmd.hpp"
#include "rh15_msg/msg/rh15_msg.hpp"
#include "rh15_cmd/srv/rh15fk.hpp"
#include "rh15_cmd/srv/rh15ik.hpp"




namespace ruiyan::rh15
{
    class rh15_ctrl : public rclcpp::Node
    {
    public:
        rh15_ctrl( std::string name );

        void CmdCallback(const rh15_cmd::msg::Rh15Cmd::SharedPtr msg);
        void PubState();
        void UpdataMotor( void );

        void rh15fk_callback(const rh15_cmd::srv::Rh15fk::Request::SharedPtr request,const rh15_cmd::srv::Rh15fk::Response::SharedPtr response);
        void rh15ik( pinocchio::Model& model, pinocchio::Data& data, Eigen::VectorXd &q_ik, std::string ftip[5], const rh15_cmd::srv::Rh15ik::Request::SharedPtr request, const rh15_cmd::srv::Rh15ik::Response::SharedPtr response);
        void rh15ik_callback(const rh15_cmd::srv::Rh15ik::Request::SharedPtr request,const rh15_cmd::srv::Rh15ik::Response::SharedPtr response);

    private:

        pinocchio::Model model_fk_,model_ik_,model_,model_l_,model_r_;
        pinocchio::Data data_fk_,data_ik_,data_,data_l_,data_r_;
        Eigen::VectorXd q_fk_,q_ik_, q_iik_, q_;
        std::string urdf_path,urdf_filename_l,urdf_filename_r;
        std::string fingertip_l_[5], fingertip_r_[5];
        std::string fingertip[5];
        std::string fingertip_fk[5];
        std::string fingertip_ik[5];

        double poly_coeff[5][4];
        int si;

        rh15_msg::msg::Rh15Msg rh15msg;
        rh15_cmd::msg::Rh15Cmd rh15cmd;


        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<rh15_msg::msg::Rh15Msg>::SharedPtr ryhand_state_publisher_;
        rclcpp::Subscription<rh15_cmd::msg::Rh15Cmd>::SharedPtr ryhand_cmd_subscriber_;


        // 声明服务回调组
        rclcpp::CallbackGroup::SharedPtr callback_group_service_;

        // 声明服务端
        rclcpp::Service<rh15_cmd::srv::Rh15fk>::SharedPtr server_rh15fk_;
        rclcpp::Service<rh15_cmd::srv::Rh15ik>::SharedPtr server_rh15ik_;


        rh15_cmd::srv::Rh15fk::Request req_fk;
        rh15_cmd::srv::Rh15fk::Response res_fk;

        rh15_cmd::srv::Rh15ik::Request req_ik;
        rh15_cmd::srv::Rh15ik::Response res_ik;


    };
}