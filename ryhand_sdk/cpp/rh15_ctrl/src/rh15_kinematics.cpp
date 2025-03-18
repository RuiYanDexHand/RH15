#include <rclcpp/rclcpp.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp> // 添加这一行
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class PinocchioRh15 : public rclcpp::Node 
{
  
public:
  PinocchioRh15( std::string name ) : Node( name )
  {

    RCLCPP_INFO(this->get_logger(), "hello z13 %s.",name.c_str());

    // 加载 URDF 模型
    std::string urdf_path = ament_index_cpp::get_package_share_directory("rh15_ctrl") + "/urdf/ruihand15z.urdf";

    RCLCPP_INFO(this->get_logger(), "urdf_path %s.",urdf_path.c_str());

    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);


    for (std::size_t i = 0; i < data_.oMi.size(); ++i)
    {
      std::stringstream ss;
      ss << "Joint " << i << " pose: translation = " 
         << data_.oMi[i].translation().transpose()
         << ", rotation =\n" << data_.oMi[i].rotation();
      RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    // RCLCPP_INFO(this->get_logger(), "z13");

    // 初始化关节状态，确保尺寸与模型匹配
    q_.resize(model_.nq);
    q_.setZero();

    // 正向运动学结果发布器
    fk_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("fk_result", 10);

    // 逆向运动学目标订阅器
    ik_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "ik_target", 10, std::bind(&PinocchioRh15::ik_callback, this, std::placeholders::_1));

    // 定时执行正向运动学
    timer_ = this->create_wall_timer(50ms, std::bind(&PinocchioRh15::fk_callback, this));

  }

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr fk_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ik_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 正向运动学回调：计算并发布末端执行器位姿
  void fk_callback()
  {

    // 修改 q_ 3的值按正弦变化，范围是 0~70度
    static double time = 0.0;
    time += 0.05; // 每次回调增加时间
    q_(2) = 35 * M_PI / 180.0 + (35.0 * M_PI / 180.0) * std::sin(time); // 将角度转换为弧度


    // Sample a random configuration
    // Eigen::VectorXd q = randomConfiguration(model_);
    std::cout << "q: " << q_.transpose() << std::endl;

    // Perform the forward kinematics over the kinematic tree
    forwardKinematics(model_, data_, q_);
  
    // 打印各个关节的位姿
    for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
    {
      std::cout << std::setw(24) << std::left << model_.names[joint_id] << ": " << std::fixed
                << std::setprecision(2) << data_.oMi[joint_id].translation().transpose() << std::endl;
    }

    // 获取末 关节坐标系的位姿
    // RCLCPP_INFO(this->get_logger(), "fz15 %d", model_.getJointId("fz15") );
    pinocchio::SE3 end_effector_pose1 = data_.oMi[ model_.getJointId("fz15") ];
    pinocchio::SE3 end_effector_pose2 = data_.oMi[ model_.getJointId("fz25") ];
    pinocchio::SE3 end_effector_pose3 = data_.oMi[ model_.getJointId("fz35") ];
    pinocchio::SE3 end_effector_pose4 = data_.oMi[ model_.getJointId("fz45") ];
    pinocchio::SE3 end_effector_pose5 = data_.oMi[ model_.getJointId("fz55") ];


    // 设置基坐标系空间姿态
    double x = 0.1, y = 0.1, z = 0.1; // 设置 x, y, z 坐标
    double roll = 0.0, pitch = 0.0, yaw = 0.0; // 设置 roll, pitch, yaw 角度

    // 创建平移向量
    Eigen::Vector3d translation(x, y, z);

    // 创建旋转矩阵
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
   
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

    // for (size_t i = 0; i < end_effector_poses.size(); ++i) {
    //   std::stringstream ss;
    //   ss << "End effector " << i + 1 << " pose: translation = " 
    //      << end_effector_poses[i].translation().transpose()
    //      << ", rotation =\n" << end_effector_poses[i].rotation();
    //   RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    // }
 
    // 转换为 ROS PoseArray 消息
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.stamp = this->now();
    pose_array_msg.header.frame_id = "world";

    for (const auto& end_effector_pose : end_effector_poses) 
    {
      geometry_msgs::msg::Pose pose_msg;
      pose_msg.position.x = end_effector_pose.translation().x();
      pose_msg.position.y = end_effector_pose.translation().y();
      pose_msg.position.z = end_effector_pose.translation().z();
      Eigen::Quaterniond quat(end_effector_pose.rotation());
      pose_msg.orientation.w = quat.w();
      pose_msg.orientation.x = quat.x();
      pose_msg.orientation.y = quat.y();
      pose_msg.orientation.z = quat.z();
      pose_array_msg.poses.push_back(pose_msg);
    }

    fk_publisher_->publish(pose_array_msg);
    // RCLCPP_INFO(this->get_logger(), "Published FK result");
  }

  // 逆向运动学回调：根据目标位姿迭代求解关节角度
  void ik_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    // 解析目标位姿
    Eigen::Vector3d target_pos(msg->position.x, msg->position.y, msg->position.z);
    Eigen::Quaterniond target_quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    pinocchio::SE3 target_pose(target_quat.toRotationMatrix(), target_pos);

    double tolerance = 1e-4;
    int max_iter = 1000;
    double dt = 1e-1;
    Eigen::VectorXd q_ik = q_;

    // 迭代求解 IK
    for (int i = 0; i < max_iter; ++i)
    {
      pinocchio::forwardKinematics(model_, data_, q_ik);
      pinocchio::SE3 current_pose = data_.oMi[model_.getJointId("fz15")];

      // 计算误差（误差在李代数空间内）
      pinocchio::Motion error = pinocchio::log6(target_pose.actInv(current_pose));

      if (error.toVector().norm() < tolerance)
      {
        break;
      }

      // 计算雅可比矩阵
      pinocchio::Data::Matrix6x J(6, model_.nv);
      J.setZero();
      // 计算各关节雅可比
      pinocchio::computeJointJacobians(model_, data_, q_ik);

      // 获取末端相对于 LOCAL_WORLD_ALIGNED 的雅可比
      pinocchio::computeFrameJacobian(model_, data_, q_ik, model_.getJointId("fz15"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

      // 伪逆更新关节角度
      Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
      q_ik = pinocchio::integrate(model_, q_ik, -J_pinv * error.toVector() * dt);
    }

    q_ = q_ik;
    RCLCPP_INFO(this->get_logger(), "IK solution: [%f, %f]", q_(0), q_(1));
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PinocchioRh15>("rh15_pinocchio");

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}