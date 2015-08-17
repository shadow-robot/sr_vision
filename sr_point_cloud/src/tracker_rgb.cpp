/*
 * Copyright (c) 2015 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

#include "sr_point_cloud/tracker_rgb.hpp"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_tracker_rgb");
  sr_point_cloud::TrackerRGB node;
  node.run();
  return 0;
}
