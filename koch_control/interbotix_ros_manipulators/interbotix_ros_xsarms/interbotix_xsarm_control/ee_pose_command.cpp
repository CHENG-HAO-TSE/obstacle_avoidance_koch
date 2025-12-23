#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class EEPoseCommander : public rclcpp::Node
{
public:
  EEPoseCommander()
  : Node("ee_pose_commander", rclcpp::NodeOptions()
                                 .automatically_declare_parameters_from_overrides(true)
                                 .arguments({"--ros-args", "-r", "__ns:=/low_cost_robot"}))
  {
    RCLCPP_INFO(this->get_logger(), "EE Pose Commander started.");

    // MoveIt interface for the "arm" group
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");

    // subscriber
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/low_cost_robot/ee_target_pose",
      10,
      std::bind(&EEPoseCommander::poseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to /low_cost_robot/ee_target_pose");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "Received new EE target pose: (%.3f, %.3f, %.3f)",
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z);

    move_group_->setPoseTarget(msg->pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed! Cannot reach target.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Plan succeeded. Executing...");
    auto exec_status = move_group_->execute(plan);

    if (exec_status == moveit::core::MoveItErrorCode::SUCCESS)
      RCLCPP_INFO(this->get_logger(), "Motion complete!");
    else
      RCLCPP_ERROR(this->get_logger(), "Execution failed.");
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EEPoseCommander>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
