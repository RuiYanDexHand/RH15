#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>


#include "rh15_cmd/msg/rh15_cmd.hpp"
#include "rh15_msg/msg/rh15_msg.hpp"



namespace ruiyan::rh15
{
    class rh15_ctrl : public rclcpp::Node
    {
    public:
        rh15_ctrl( std::string name );

        void CmdCallback(const rh15_cmd::msg::Rh15Cmd::SharedPtr msg);
        void PubState();
        void UpdataMotor( void );


    private:

        std::string urdf_path,urdf_filename;
        int si;

        double poly_coeff[4];

        rh15_msg::msg::Rh15Msg rh15msg;
        rh15_cmd::msg::Rh15Cmd rh15cmd;


        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<rh15_msg::msg::Rh15Msg>::SharedPtr ryhand_state_publisher_;
        rclcpp::Subscription<rh15_cmd::msg::Rh15Cmd>::SharedPtr ryhand_cmd_subscriber_;

    };
}