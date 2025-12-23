#include <memory>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class SimpleEndEffectorControl : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  SimpleEndEffectorControl() : Node("simple_endeffector_control")
  {
    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "Simple End Effector Control");
    RCLCPP_INFO(this->get_logger(), "===========================================");

    // 創建 action client 連接到 arm_controller
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/low_cost_robot/arm_controller/follow_joint_trajectory");

    // 訂閱當前關節狀態
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/low_cost_robot/joint_states", 10,
        std::bind(&SimpleEndEffectorControl::joint_state_callback, this, std::placeholders::_1));

    // IK 服務客戶端
    ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>(
        "/low_cost_robot/compute_ik");

    RCLCPP_INFO(this->get_logger(), "Waiting for action server and IK service...");
    
    // 等待 action server
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available!");
      return;
    }

    // 等待 IK 服務
    if (!ik_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "IK service not available!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Connected! Ready to move.");
  }

  void move_to_pose(double x, double y, double z, double qw = 1.0, double qx = 0.0, double qy = 0.0, double qz = 0.0)
  {
    RCLCPP_INFO(this->get_logger(), "-------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Moving to target pose:");
    RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f]", x, y, z);
    RCLCPP_INFO(this->get_logger(), "  Orientation: [%.3f, %.3f, %.3f, %.3f]", qw, qx, qy, qz);
    RCLCPP_INFO(this->get_logger(), "-------------------------------------------");

    // 等待關節狀態
    while (current_joint_positions_.empty() && rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 準備 IK 請求
    auto ik_request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    
    // 設定目標 pose
    ik_request->ik_request.group_name = "arm";
    ik_request->ik_request.pose_stamped.header.frame_id = "world";
    ik_request->ik_request.pose_stamped.pose.position.x = x;
    ik_request->ik_request.pose_stamped.pose.position.y = y;
    ik_request->ik_request.pose_stamped.pose.position.z = z;
    ik_request->ik_request.pose_stamped.pose.orientation.w = qw;
    ik_request->ik_request.pose_stamped.pose.orientation.x = qx;
    ik_request->ik_request.pose_stamped.pose.orientation.y = qy;
    ik_request->ik_request.pose_stamped.pose.orientation.z = qz;

    // 設定當前機器人狀態作為起始點
    ik_request->ik_request.robot_state.joint_state.name = joint_names_;
    ik_request->ik_request.robot_state.joint_state.position = current_joint_positions_;

    RCLCPP_INFO(this->get_logger(), "Computing IK solution...");

    // 呼叫 IK 服務
    auto ik_future = ik_client_->async_send_request(ik_request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), ik_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call IK service!");
      return;
    }

    auto ik_response = ik_future.get();
    
    if (ik_response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "IK solution not found! Error code: %d", ik_response->error_code.val);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "IK solution found! Sending trajectory...");

    // 創建軌跡
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names_;
    
    // 添加軌跡點
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = ik_response->solution.joint_state.position;
    point.velocities.resize(joint_names_.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);  // 2秒到達
    
    goal_msg.trajectory.points.push_back(point);

    // 發送 goal
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&SimpleEndEffectorControl::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal!");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted! Waiting for result...");

    // 等待結果
    auto result_future = action_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get result!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "Motion COMPLETE!");
    RCLCPP_INFO(this->get_logger(), "===========================================");
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (joint_names_.empty()) {
      // 只儲存 arm 關節，不包括 gripper
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] != "joint_gripper") {
          joint_names_.push_back(msg->name[i]);
          current_joint_positions_.push_back(msg->position[i]);
        }
      }
      
      if (!joint_names_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Received initial joint states:");
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          RCLCPP_INFO(this->get_logger(), "  %s: %.3f", 
                      joint_names_[i].c_str(), current_joint_positions_[i]);
        }
      }
    } else {
      // 更新關節位置
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        for (size_t j = 0; j < msg->name.size(); ++j) {
          if (msg->name[j] == joint_names_[i]) {
            current_joint_positions_[i] = msg->position[j];
            break;
          }
        }
      }
    }
  }

  void result_callback(const GoalHandleFJT::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "✓ Trajectory execution succeeded!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "✗ Trajectory execution aborted!");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Trajectory execution canceled!");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code!");
        break;
    }
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  
  std::vector<std::string> joint_names_;
  std::vector<double> current_joint_positions_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SimpleEndEffectorControl>();

  // 等待初始化
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 移動到目標位置
  node->move_to_pose(0.18, 0.0, 0.15);  // x, y, z

  rclcpp::shutdown();
  return 0;
}
