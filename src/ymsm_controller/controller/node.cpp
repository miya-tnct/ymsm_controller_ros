#include "ymsm_controller/controller/node.h"

#include <algorithm>
#include <string>
#include <set>
#include <unordered_map>

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
  target_itr_ = path_msg_->poses.begin();
  frame_ids_.clear();
  frame_ids_.emplace(path_msg_->header.frame_id);
  for (const auto & pose : path_msg->poses) {
    auto frame_id = pose.header.frame_id;
    if (!frame_id.empty()) {
      frame_ids_.emplace(std::move(frame_id));
    }
  }
}

void Node::publish_cmd_vel_zero(const ros::TimerEvent&)
{
  geometry_msgs::Twist twist;
  cmd_vel_publisher_.publish(twist);
}

void Node::publish_cmd_vel(const ros::TimerEvent& timer_event)
{
  std::unordered_map<std::string, geometry_msgs::TransformStamped> tf_msgs;
  for (const auto & frame_id : frame_ids_) {
    try {
      tf_msgs[frame_id] = tf_buffer_.lookupTransform(
        frame_id, "base_link", ros::Time(0));
    }
    catch (...) {
      this->publish_cmd_vel_zero(timer_event);
      return;
    }
  }
  
  double error_x, error_y;
  geometry_msgs::TransformStamped tf_msg;
  while(true) {
    if (target_itr_ == path_msg_->poses.end()) {
      this->publish_cmd_vel_zero(timer_event);
      return;
    }

    if (!target_itr_->header.frame_id.empty()) {
      tf_msg = tf_msgs[target_itr_->header.frame_id];
    }
    else {
      tf_msg = tf_msgs[path_msg_->header.frame_id];
    }
    error_x = target_itr_->pose.position.x - tf_msg.transform.translation.x;
    error_y = target_itr_->pose.position.y - tf_msg.transform.translation.y;

    if (std::hypot(error_x, error_y) >= 0.2) {
      break;
    }

    ++target_itr_;
  }

  tf2::Quaternion quat;
  tf2::fromMsg(tf_msg.transform.rotation, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  
  auto error_yaw = std::atan2(error_y, error_x) - yaw;
  while (error_yaw > +M_PI) error_yaw -= 2 * M_PI;
  while (error_yaw < -M_PI) error_yaw += 2 * M_PI;
  
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = std::min(std::hypot(error_x, error_y), 0.1);
  cmd_vel_msg.angular.z = error_yaw * 0.6;

  cmd_vel_publisher_.publish(cmd_vel_msg);
}

}