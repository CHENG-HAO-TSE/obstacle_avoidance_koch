#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  //--- ROS2 初始化 ---
  rclcpp::init(argc, argv);

  // 創建節點選項，自動聲明來自 launch 文件的參數
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.allow_undeclared_parameters(true);

  // 創建節點
  auto node = std::make_shared<rclcpp::Node>(
      "endeffector_koch",
      "/low_cost_robot",  // 命名空間
      node_options);

  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "===========================================");
  RCLCPP_INFO(logger, "Low Cost Robot End Effector Control");
  RCLCPP_INFO(logger, "===========================================");

  //--- Publish 狀態 ---
  auto status_pub = node->create_publisher<std_msgs::msg::String>(
      "endeffector_status", 10);

  auto publish_status = [&](const std::string& s) {
    std_msgs::msg::String msg;
    msg.data = s;
    status_pub->publish(msg);
    RCLCPP_INFO(logger, "Status: %s", s.c_str());
  };

  publish_status("Node started");

  // 等待一下確保所有 ROS 連接建立
  rclcpp::sleep_for(std::chrono::seconds(1));

  //--- 建立 MoveGroupInterface ---
  RCLCPP_INFO(logger, "Creating MoveGroupInterface...");
  publish_status("Creating MoveGroupInterface");

  using moveit::planning_interface::MoveGroupInterface;

  // 直接使用節點和組名創建 MoveGroupInterface
  auto move_group = std::make_shared<MoveGroupInterface>(node, "arm");

  RCLCPP_INFO(logger, "MoveGroupInterface created successfully!");
  RCLCPP_INFO(logger, "Planning frame: %s", move_group->getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group->getEndEffectorLink().c_str());

  publish_status("MoveGroupInterface ready");

  //--- 顯示目前 EE Pose ---
  try {
    auto current_pose = move_group->getCurrentPose();
    RCLCPP_INFO(logger, "Current EE Position: [%.3f, %.3f, %.3f]",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);
  } catch (const std::exception& ex) {
    RCLCPP_WARN(logger, "Could not get current pose: %s", ex.what());
  }

  //--- 設定目標 Pose ---
  geometry_msgs::msg::Pose target;
  target.orientation.w = 1.0;  // 直向朝下 (identity quaternion)
  target.position.x = 0.18;    // Forward ~18cm
  target.position.y = 0.00;    // Center
  target.position.z = 0.15;    // Height ~15cm

  RCLCPP_INFO(logger, "-------------------------------------------");
  RCLCPP_INFO(logger, "Target Pose: [%.3f, %.3f, %.3f]",
              target.position.x,
              target.position.y,
              target.position.z);
  RCLCPP_INFO(logger, "-------------------------------------------");

  move_group->setPoseTarget(target);
  publish_status("Planning to target pose");

  //--- 建立規劃 ---
  RCLCPP_INFO(logger, "Planning trajectory...");
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!success)
  {
    RCLCPP_ERROR(logger, "Planning FAILED!");
    publish_status("Planning failed");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Planning SUCCEEDED!");
  publish_status("Plan ready");

  //--- 等待用戶確認（可選） ---
  RCLCPP_INFO(logger, "-------------------------------------------");
  RCLCPP_INFO(logger, "Ready to execute motion.");
  RCLCPP_INFO(logger, "Starting execution in 2 seconds...");
  RCLCPP_INFO(logger, "-------------------------------------------");
  rclcpp::sleep_for(std::chrono::seconds(2));

  //--- 執行 ---
  RCLCPP_INFO(logger, "Executing motion...");
  publish_status("Executing motion");

  auto exec_status = move_group->execute(plan);

  if (exec_status != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Execution FAILED!");
    publish_status("Execution failed");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "===========================================");
  RCLCPP_INFO(logger, "Motion execution COMPLETE!");
  RCLCPP_INFO(logger, "===========================================");
  publish_status("Motion complete - SUCCESS");

  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::shutdown();
  return 0;
}
