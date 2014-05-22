#include "sr_point_cloud/tracker.hpp"

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "point_cloud_tracker");
  sr_point_cloud::Tracker<pcl::PointXYZ> node;
  node.run();
  return 0;
}
