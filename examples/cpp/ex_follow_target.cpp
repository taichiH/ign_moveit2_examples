/// Example that uses MoveIt 2 to follow a target inside Ignition Gazebo

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

const std::string MOVE_GROUP = "arm";

class MoveItFollowTarget : public rclcpp::Node
{
public:
  /// Constructor
  MoveItFollowTarget();
  moveit::planning_interface::MoveGroupInterface move_group;

  /* const moveit::core::JointModelGroup * joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(MOVE_GROUP); */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pre_pick_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pre_put_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reset_manip_sub_;
  geometry_msgs::msg::Pose previous_target_pose_;

private:
  void pre_pick_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void pre_put_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void reset_manip_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
};

MoveItFollowTarget::MoveItFollowTarget()
: Node("ex_follow_target"), move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
  RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);

  pre_pick_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/pre_pick", rclcpp::QoS(1),
    std::bind(&MoveItFollowTarget::pre_pick_callback, this, std::placeholders::_1));

  pre_put_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/pre_put", rclcpp::QoS(1),
    std::bind(&MoveItFollowTarget::pre_put_callback, this, std::placeholders::_1));

  reset_manip_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/reset_manip", rclcpp::QoS(1),
    std::bind(&MoveItFollowTarget::reset_manip_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MoveItFollowTarget::reset_manip_callback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  // reset
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_group.setMaxVelocityScalingFactor(0.1);
  geometry_msgs::msg::Pose reset_manip_pose;
  reset_manip_pose.orientation.x = 1.0;
  reset_manip_pose.orientation.y = 0.0;
  reset_manip_pose.orientation.z = 0.0;
  reset_manip_pose.orientation.w = 0.0;
  reset_manip_pose.position.x = 0.4;
  reset_manip_pose.position.y = 0.0;
  reset_manip_pose.position.z = 0.5;
  move_group.setPoseTarget(reset_manip_pose);
  moveit::planning_interface::MoveGroupInterface::Plan reset_manip_pose_plan;
  bool success =
    (move_group.plan(reset_manip_pose_plan) ==
     moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(this->get_logger(), "reset_manip_pose: %s", success ? "SUCCESS" : "FAILED");
  move_group.move();
}

void MoveItFollowTarget::pre_pick_callback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  // pre pick
  move_group.setMaxAccelerationScalingFactor(0.05);
  move_group.setMaxVelocityScalingFactor(0.1);
  geometry_msgs::msg::Pose pre_pick_pose;
  pre_pick_pose.orientation.x = -0.70710681187;
  pre_pick_pose.orientation.y = 0.70710681187;
  pre_pick_pose.orientation.z = 0.0;
  pre_pick_pose.orientation.w = 0.0;
  pre_pick_pose.position.x = -0.2;
  pre_pick_pose.position.y = -0.65;
  pre_pick_pose.position.z = 0.2;
  move_group.setPoseTarget(pre_pick_pose);

  moveit::planning_interface::MoveGroupInterface::Plan pre_pick_pose_plan;
  bool success =
    (move_group.plan(pre_pick_pose_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(this->get_logger(), "pre_pick_pose: %s", success ? "SUCCESS" : "FAILED");
  move_group.move();
}

void MoveItFollowTarget::pre_put_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  // pre put
  move_group.setMaxAccelerationScalingFactor(0.05);
  move_group.setMaxVelocityScalingFactor(0.1);
  geometry_msgs::msg::Pose pre_put_pose;
  pre_put_pose.orientation.x = 0.70710681187;
  pre_put_pose.orientation.y = 0.70710681187;
  pre_put_pose.orientation.z = 0.0;
  pre_put_pose.orientation.w = 0.0;
  pre_put_pose.position.x = -0.2;
  pre_put_pose.position.y = 0.65;
  pre_put_pose.position.z = 0.28;
  move_group.setPoseTarget(pre_put_pose);

  moveit::planning_interface::MoveGroupInterface::Plan pre_put_pose_plan;
  bool success =
    (move_group.plan(pre_put_pose_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(this->get_logger(), "pre_put_pose: %s", success ? "SUCCESS" : "FAILED");
  move_group.move();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto target_follower = std::make_shared<MoveItFollowTarget>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(target_follower);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
