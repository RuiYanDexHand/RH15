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
        poly_coeff[0] = 0.000504;
        poly_coeff[1] = -0.043255;
        poly_coeff[2] = 2.561668;
        poly_coeff[3] = 0.148494;

        
        std::string package_share_dir = get_pkg_directory("rh15_ctrl");
        urdf_path = extract_path_before_delimiter( package_share_dir,"/install");

        rh15msg.lr = this->declare_parameter("left_right", 0);
        if( rh15msg.lr == 0 )
            urdf_filename = urdf_path + "/hand_left.urdf";
        else
            urdf_filename = urdf_path + "/hand_right.urdf";

        RCLCPP_INFO(this->get_logger(), "urdf_path: %s.",urdf_filename.c_str());


        // interfaces_ptr_ = std::make_shared<InterfacesThread>(urdf_urdf_filenamepath,this->declare_parameter("handhand_can_id", "can0"), end_type);
        auto pub_name = this->declare_parameter("ryhand_pub_topic_name", "ryhand_status");
        auto sub_name = this->declare_parameter("ryhand_sub_topic_name", "ryhand_cmd");


        std::string control_type = this->declare_parameter("control_type", "normal");
        RCLCPP_INFO(this->get_logger(), "control_type = %s",control_type.c_str());

        if (control_type == "normal")
        {
            // 创建发布器 - 状态
            ryhand_state_publisher_ = this->create_publisher<rh15_msg::msg::Rh15Msg>(pub_name, 1);

            // 创建订阅器 - 命令
            ryhand_cmd_subscriber_ = this->create_subscription<rh15_cmd::msg::Rh15Cmd>( sub_name, 10, std::bind(&rh15_ctrl::CmdCallback, this, std::placeholders::_1) );

            rclcpp::Publisher<rh15_msg::msg::Rh15Msg>::SharedPtr ryhand_state_publisher_;
            rclcpp::Subscription<rh15_cmd::msg::Rh15Cmd>::SharedPtr ryhand_cmd_subscriber_;
            
            // 创建定时器 10ms 
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&rh15_ctrl::PubState, this));
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


    void rh15_ctrl::CmdCallback( const rh15_cmd::msg::Rh15Cmd::SharedPtr msg )
    {
        rh15_cmd::msg::Rh15Cmd cmd = *msg;
        int p1,p2,p3,p4,p5,p6; 

        rh15msg.lr = cmd.lr;  // left_or_right
        if( rh15msg.lr == 0 ) urdf_filename = urdf_path + "/hand_left.urdf";
        else urdf_filename = urdf_path + "/hand_right.urdf";

        if( memcmp(&rh15cmd, &cmd, sizeof(rh15_cmd::msg::Rh15Cmd)) )
        {
            for(int i = 0; i < 15; i++)
            {
                sutServoDataW[i].stuCmd.usTp = cmd.m_pos[i];
                sutServoDataW[i].stuCmd.usTv = cmd.m_spd[i];
                sutServoDataW[i].stuCmd.usTc = cmd.m_curlimit[i];
            }  

            switch (cmd.mode)
            {
                // raw cmd
                case 0:
                    UpdataMotor();  
                    break;

                // rad cmd
                case 1:
                    for( int i = 0; i < 20; i++ )
                    {
                        if( i%4 == 3 )
                        {
                            p1 = map_rad_to_value_full_range( cmd.j_ang[ i/4 * 4 + 0 ] );
                            p2 = map_rad90_to_value( cmd.j_ang[ i/4 * 4 + 1 ] );
                            p3 = map_rad75_to_value( cmd.j_ang[ i/4 * 4 + 2 ] );
                            p4 = map_rad75_to_value( cmd.j_ang[ i/4 * 4 + 3 ] );
                            (void)p4;

                            if( p1 > 4095/4 ) p1 = 4095/4;
                            if( p1 < -4095/4 ) p1 = -4095/4;

                            // if( !cmd.lr ) 
                            // {
                                p5 = p2 - p1/2;
                                p6 = p2 + p1/2;
                            // }
                            // else
                            // {
                            //     p5 = p2 + p1/2;
                            //     p6 = p2 - p1/2; 
                            // }

                            if( p5 > 4095 ) p5 = 4095;
                            else if( p5 < 0 ) p5 = 0;

                            if( p6 > 4095 ) p6 = 4095;
                            else if( p6 < 0 ) p6 = 0;

                            if( p3 > 4095 ) p3 = 4095;
                            else if( p3 < 0 ) p3 = 0;

                            sutServoDataW[ (i/4 *3) + 0 ].stuCmd.usTp = p5;
                            sutServoDataW[ (i/4 *3) + 1 ].stuCmd.usTp = p6;
                            sutServoDataW[ (i/4 *3) + 2 ].stuCmd.usTp = p3;
                        }
                    }

                    UpdataMotor();    
                    break;

                // end pos cmd    
                case 2:

                    // float64 x_base
                    // float64 y_base
                    // float64 z_base

                    // float64 roll_base
                    // float64 pitch_base
                    // float64 yaw_base

                    // float64[5] x
                    // float64[5] y
                    // float64[5] z

                    // float64[5] roll
                    // float64[5] pitch
                    // float64[5] yaw

                    // double end_pos[6] = {msg->end_pos[0], msg->end_pos[1], msg->end_pos[2], msg->end_pos[3], msg->end_pos[4], msg->end_pos[5]};
                    // Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(end_pos);
                    // interfaces_ptr_->setEndPose(transform);
                    // std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0};
            
                    // for (int i = 0; i < 6; i++)
                    // {
                    //     joint_positions[i] = msg->joint_pos[i];
                    // }

                    break;

                default:
                    break;
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
                rh15msg.j_ang[ (i/3 * 4) + 3] = value_to_rad75(p); // deg_to_rad( evaluatePolynomial( poly_coeff, 3,  rad_to_deg( value_to_rad75(p) ) ) ); 
                break;

            default:
                break;
            }
        }

        rh15msg.x_base = rh15cmd.x_base;
        rh15msg.y_base = rh15cmd.y_base;
        rh15msg.z_base = rh15cmd.z_base;
        rh15msg.roll_base = rh15cmd.roll_base;
        rh15msg.pitch_base = rh15cmd.pitch_base;
        rh15msg.yaw_base = rh15cmd.yaw_base;

        for (int i = 0; i < 5; i++)
        {
            rh15msg.x[i] = 0;
            rh15msg.y[i] = 0;  
            rh15msg.z[i] = 0;
    
            rh15msg.roll[i]   = 0;
            rh15msg.pitch[i]  = 0;
            rh15msg.yaw[i]    = 0;
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
    rclcpp::spin(node);
    rclcpp::shutdown();


    thread_go = 0;

    // 关闭 CAN 套接字
    if( sock > 0 ) close_can_socket(sock);


    return 0;
}














#if 0

#include <rclcpp/rclcpp.hpp>

/*
    创建一个类节点，名字叫做 Rh15,继承自Node.
*/
class Rh15 : public rclcpp::Node
{

public:
    // 构造函数,有一个参数为节点名称
    Rh15(std::string name) : Node(name)
    {
        // 打印一句自我介绍
        RCLCPP_INFO(this->get_logger(), "hello %s.",name.c_str());
    }

private:
   
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    /*产生一个rh15_ctrl的节点*/
    auto node = std::make_shared<Rh15>("rh15_ctrl");
    
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);


    rclcpp::shutdown();
    return 0;
}

#endif