#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>


#include "rh15_cmd/msg/rh15_cmd.hpp"



namespace ruiyan::rh15
{
    class rh15_test : public rclcpp::Node
    {
    public:

        rh15_test( std::string name );

        void PubCmd();

        float rad_to_deg(float rad);
        float deg_to_rad(float deg);

    private:

        int tick;
        int tspan_ms;

        rh15_cmd::msg::Rh15Cmd rh15cmd;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<rh15_cmd::msg::Rh15Cmd>::SharedPtr ryhand_cmd_publisher_;


    };
}