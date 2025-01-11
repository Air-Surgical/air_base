#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <math.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "scene_creator", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("scene_creator");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "body";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = 1.58;
    primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.1;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose body_pose;
    tf2::Quaternion tf2_rotate;
    tf2_rotate.setRPY(0., M_PI/2, 0.0);
    // body_pose.orientation.w = 1.0;
    body_pose.orientation = tf2::toMsg(tf2_rotate);
    body_pose.position.x = 0.2;
    body_pose.position.y = 0.2;
    body_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(body_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}