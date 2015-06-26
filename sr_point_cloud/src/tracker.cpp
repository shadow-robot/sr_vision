/*
 * Copyright (c) 2015 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

#include "sr_point_cloud/tracker.hpp"

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_tracker");
  sr_point_cloud::Tracker<pcl::PointXYZ> node;
  node.run();
  return 0;
}
