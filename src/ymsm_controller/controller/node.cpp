#include "ymsm_controller/controller/node.h"

#include <algorithm>

#include "geometry_msgs/Twist.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace ymsm_controller::controller
{

Node::Node() :
  ros::NodeHandle(),
  tf_buffer_(),
  tf_listner_(tf_buffer_),
  cmd_vel_publisher_(this->advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1, false)),
  path_subscriber_(this->subscribe("path", 1, &Node::initialize_path, this))
{
}


void Node::initialize_path(nav_msgs::Path::ConstPtr path_msg)
{
  this->update_path(path_msg);
  publish_cmd_vel_timer_ = this->createTimer(ros::Duration(1.0 / 60), &Node::publish_cmd_vel, this);
  path_subscriber_ = this->subscribe("path", 1, &Node::update_path, this);
}

void Node::update_path(nav_msgs::Path::ConstPtr path_msg)
{
  path_msg_ = path_msg;
  path_index_ = 0;
}

void Node::publish_cmd_vel_zero(const ros::TimerEvent&)
{
  geometry_msgs::Twist twist;
  cmd_vel_publisher_.publish(twist);
}

void Node::publish_cmd_vel(const ros::TimerEvent& timer_event)
{
  auto tf_msg_ = tf_buffer_.lookupTransform(
    "odom", "base_link", ros::Time(0));
  
  double error_x, error_y;
  while(true) {
    if (path_index_ >= path_msg_->poses.size()) {
      this->publish_cmd_vel_zero(timer_event);
      return;
    }

    const auto target_msg = path_msg_->poses[path_index_];

    error_x = target_msg.pose.position.x - tf_msg_.transform.translation.x;
    error_y = target_msg.pose.position.y - tf_msg_.transform.translation.y;

    if (std::hypot(error_x, error_y) >= 0.2) {
      break;
    }

    ++path_index_;
  }

  tf2::Quaternion quat;
  tf2::fromMsg(tf_msg_.transform.rotation, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  
  auto error_yaw = std::atan2(error_y, error_x) - yaw;
  while (error_yaw > +M_PI) error_yaw -= 2 * M_PI;
  while (error_yaw < -M_PI) error_yaw += 2 * M_PI;
  
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = std::min(std::hypot(error_x, error_y), 0.2);
  cmd_vel_msg.angular.z = error_yaw * 0.2;

  cmd_vel_publisher_.publish(cmd_vel_msg);
}

}