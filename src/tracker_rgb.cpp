#include "sr_point_cloud/tracker_rgb.hpp"

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "sr_point_cloud");
  sr_point_cloud::TrackerRGB node;
  node.run();
  return 0;
}
