#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  //--- ROS2 初始化 ---
  rclcpp::init(argc, argv);

  // 重要！放在 /low_cost_robot namespace 內
  auto node = std::make_shared<rclcpp::Node>(
      "low_cost_robot_endeffector_control",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true)
          .arguments({"--ros-args", "-r", "__ns:=/low_cost_robot"}));

  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "Starting Interbotix Low Cost Robot End Effector Control...");

  //--- Publish 狀態 ---
  auto status_pub = node->create_publisher<std_msgs::msg::String>(
      "/low_cost_robot/endeffector_status", 10);

  auto publish_status = [&](std::string s) {
    std_msgs::msg::String msg;
    msg.data = s;
    status_pub->publish(msg);
  };

  publish_status("Node started.");

  //--- 建立 MoveGroupInterface ---
  using moveit::planning_interface::MoveGroupInterface;

  // ⚠️ Interbotix MoveIt 的 group name 是 "arm"
  MoveGroupInterface move_group(node, "arm");

  RCLCPP_INFO(logger, "MoveGroupInterface created with group 'arm'.");

  //--- 顯示目前 EE Pose ---
  auto current_pose = move_group.getCurrentPose();
  RCLCPP_INFO(logger, "Current EE Position: [%.3f, %.3f, %.3f]",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

  //--- 設定目標 Pose ---
  geometry_msgs::msg::Pose target;
  target.orientation.w = 1.0;  // 直向朝下 (identity quaternion)
  target.position.x = 0.18;    // Forward ~18cm
  target.position.y = 0.00;    // Center
  target.position.z = 0.15;    // Height ~15cm

  RCLCPP_INFO(logger, "Target Pose: [%.3f, %.3f, %.3f]",
              target.position.x,
              target.position.y,
              target.position.z);

  move_group.setPoseTarget(target);

  publish_status("Planning...");

  //--- 建立規劃 ---
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!success)
  {
    RCLCPP_ERROR(logger, "Planning failed!");
    publish_status("Planning failed.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Planning succeeded!");
  publish_status("Executing plan...");

  //--- 執行 ---
  auto exec_status = move_group.execute(plan);

  if (exec_status != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Execution failed!");
    publish_status("Execution failed.");
  }
  else
  {
    RCLCPP_INFO(logger, "Motion execution complete!");
    publish_status("Motion complete!");
  }

  rclcpp::shutdown();
  return 0;
}
