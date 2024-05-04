#ifndef YMSM_CONTROLLER_CONTROLLER_NODE_H_
#define YMSM_CONTROLLER_CONTROLLER_NODE_H_

#include <cstddef>

#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace ymsm_controller::controller
{

class Node : public ros::NodeHandle
{
public:
  Node();

private:
  void initialize_path(nav_msgs::Path::ConstPtr path_msg);

  void update_path(nav_msgs::Path::ConstPtr path_msg);

  void publish_cmd_vel(const ros::TimerEvent& timer_event);

  void publish_cmd_vel_zero(const ros::TimerEvent&);

  std::size_t path_index_;

  nav_msgs::Path::ConstPtr path_msg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listner_;

  ros::Publisher cmd_vel_publisher_;
  ros::Subscriber path_subscriber_;
  ros::Timer publish_cmd_vel_timer_;
};

}

#endif