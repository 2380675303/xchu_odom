//
// Created by xchu on 2021/3/22.
//

#include "xchu_mapping/log_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "xchu_mapping_node");
  ros::NodeHandle privateHandle;
  ros::NodeHandle nh("~");

  LidarMapping mapping(nh);
  //mapping.run();

//  std::thread viewer(&LidarMapping::ViewerThread, &mapping);

  ros::Rate rate(100);
  while (ros::ok()) {
    //mapping.run();

    ros::spinOnce();
    rate.sleep();
  }
//  viewer.join();

  return 0;
}