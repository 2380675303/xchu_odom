
/**
 * @file xchu_mapping_node.cpp
 * @author xiangcheng hu (2022087641@qq.com)
 * @brief
 * @version 1.0
 * @date 2020-09-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <xchu_mapping/xchu_mapping.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "xchu_mapping_node");

  LidarMapping mapping;

  mapping.run();

  return 0;
}











