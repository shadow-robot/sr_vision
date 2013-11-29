#pragma once

#include <ros/ros.h>
#include <shape_msgs/Mesh.h>
#include <dynamic_reconfigure/server.h>
#include <sr_point_cloud/TriangulatorConfig.h>

// ROS PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h> // Allow use of PCL cloud types for pubs/subs
#include <pcl_msgs/PolygonMesh.h>

// PCL specific includes
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

//-------------------------------------------------------------------------------

namespace sr_point_cloud {

class Triangulator {

protected:
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> Cloud;

  ros::NodeHandle nh_;

  // Use a Moving Least Squares (MLS) surface reconstruction method
  // to smooth and resample noisy data?
  bool resample_;

  dynamic_reconfigure::Server<TriangulatorConfig> config_server_;
  double mu_;
  int maximum_nearest_neighbors_;
  double maximum_surface_angle_;
  double minimum_angle_;
  double maximum_angle_;

  double gp3_search_radius_;
  double mls_search_radius_;

  ros::Subscriber input_sub_;
  ros::Publisher pcl_output_pub_;
  ros::Publisher shape_output_pub_;

public:
  Triangulator();
  ~Triangulator();

  void run(void);

protected:
  void config_cb(TriangulatorConfig &config, uint32_t level);

  void cloud_cb (const sensor_msgs::PointCloud2 &sensor_pc2);

  void fromPCLPolygonMesh(const pcl::PolygonMesh &pclMesh,
                          const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_with_normals,
                          shape_msgs::Mesh &shapeMesh);

}; // Triangulator

} // End of namespace sr_point_cloud

//-------------------------------------------------------------------------------
