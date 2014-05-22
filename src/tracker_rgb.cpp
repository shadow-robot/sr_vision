#include "sr_point_cloud/tracker_rgb.hpp"

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "point_cloud_tracker_rgb");
  sr_point_cloud::TrackerRGB node;
  node.run();
  return 0;
}
