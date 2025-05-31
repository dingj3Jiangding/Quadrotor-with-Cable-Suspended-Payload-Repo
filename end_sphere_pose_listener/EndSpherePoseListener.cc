#include <rclcpp/rclcpp.hpp>
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
#include <Eigen/Geometry>  // 用于四元数和向量

class EndSpherePosePublisher : public rclcpp::Node
{
public:
  EndSpherePosePublisher()
    : Node("end_sphere_publisher")
  {
    pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
      "/fmu/in/vehicle_mocap_odometry", 10);

    if (!gz_node_.Subscribe("/world/default/pose/info", &EndSpherePosePublisher::OnPoseInfo, this))
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to subscribe to /world/default/pose/info");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "✅ Subscribed to /world/default/pose/info");
    }
  }

private:
  void OnPoseInfo(const gz::msgs::Pose_V &msg)
  {
    Eigen::Vector3f base_pos;
    Eigen::Quaternionf base_quat;
    Eigen::Vector3f rel_pos;
    Eigen::Quaternionf rel_quat;
    Eigen::Vector3f x500_pos;
    Eigen::Quaternionf x500_quat;

    bool found_base = false;
    bool found_sphere = false;


    bool found_x500 = false;


    for (int i = 0; i < msg.pose_size(); ++i)
    {
      const auto &pose = msg.pose(i);
      const std::string &name = pose.name();

      if (name == "x500_cable_0")
      {
        x500_pos = Eigen::Vector3f(pose.position().x(), pose.position().y(), pose.position().z());
        x500_quat = Eigen::Quaternionf(pose.orientation().w(),
                                      pose.orientation().x(),
                                      pose.orientation().y(),
                                      pose.orientation().z());
        found_x500 = true;

        RCLCPP_INFO(this->get_logger(), "🟢 x500_cable_0 pos: [%.3f %.3f %.3f]",
                    pose.position().x(), pose.position().y(), pose.position().z());
        RCLCPP_INFO(this->get_logger(), "🟢 x500_cable_0 quat: [w=%.3f x=%.3f y=%.3f z=%.3f]",
                    pose.orientation().w(), pose.orientation().x(),
                    pose.orientation().y(), pose.orientation().z());
      }

      if (name == "cable_base_link")
      {
        RCLCPP_INFO(this->get_logger(), "GET cable base link");
        base_pos = Eigen::Vector3f(pose.position().x(), pose.position().y(), pose.position().z());
        base_quat = Eigen::Quaternionf(pose.orientation().w(),
                                       pose.orientation().x(),
                                       pose.orientation().y(),
                                       pose.orientation().z());
        found_base = true;
      
        // ✅ 输出原始位置与姿态
        RCLCPP_INFO(this->get_logger(), "🔵 cable_base_link pos: [%.3f %.3f %.3f]",
                    pose.position().x(), pose.position().y(), pose.position().z());
        RCLCPP_INFO(this->get_logger(), "🔵 cable_base_link quat: [w=%.3f x=%.3f y=%.3f z=%.3f]",
                    pose.orientation().w(), pose.orientation().x(),
                    pose.orientation().y(), pose.orientation().z());
      }
      
      if (name == "end_sphere")
      {
        RCLCPP_INFO(this->get_logger(), "GET end sphere");
        rel_pos = Eigen::Vector3f(pose.position().x(), pose.position().y(), pose.position().z());
        rel_quat = Eigen::Quaternionf(pose.orientation().w(),
                                      pose.orientation().x(),
                                      pose.orientation().y(),
                                      pose.orientation().z());
        found_sphere = true;
      
        // ✅ 输出原始位置与姿态
        RCLCPP_INFO(this->get_logger(), "🔴 end_sphere pos: [%.3f %.3f %.3f]",
                    pose.position().x(), pose.position().y(), pose.position().z());
        RCLCPP_INFO(this->get_logger(), "🔴 end_sphere quat: [w=%.3f x=%.3f y=%.3f z=%.3f]",
                    pose.orientation().w(), pose.orientation().x(),
                    pose.orientation().y(), pose.orientation().z());
      }
      
    }

    if (found_base && found_sphere)
    {
      // 世界坐标 = base_link 位置 + base_quat * 相对位置
      Eigen::Vector3f world_base_pos = x500_pos + x500_quat * base_pos;
      Eigen::Quaternionf world_base_quat = x500_quat * base_quat;

      Eigen::Vector3f world_pos = world_base_pos + world_base_quat * rel_pos;
      Eigen::Quaternionf world_quat = world_base_quat * rel_quat;

      px4_msgs::msg::VehicleOdometry odom_msg;
      odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

      odom_msg.position[0] = world_pos.x();
      odom_msg.position[1] = world_pos.y();
      odom_msg.position[2] = world_pos.z();

      odom_msg.q[0] = world_quat.w();
      odom_msg.q[1] = world_quat.x();
      odom_msg.q[2] = world_quat.y();
      odom_msg.q[3] = world_quat.z();

      odom_msg.velocity[0] = 0.0f;
      odom_msg.velocity[1] = 0.0f;
      odom_msg.velocity[2] = 0.0f;

      pub_->publish(odom_msg);

      RCLCPP_INFO(this->get_logger(), "📤 Published world pose: [%.3f %.3f %.3f]",
                  world_pos.x(), world_pos.y(), world_pos.z());
    }
  }

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
  gz::transport::Node gz_node_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EndSpherePosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}