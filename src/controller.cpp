#include "ymsm_controller/controller/node.h"

int main(int argc, char **argv)
{
  //ノードの初期化
  ros::init(argc, argv, "controller");
  ymsm_controller::controller::Node node;
  ros::spin();
  return 0;
}