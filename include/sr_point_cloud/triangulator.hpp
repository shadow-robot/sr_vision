#pragma once

#include <ros/ros.h>
#include <shape_msgs/Mesh.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <sr_point_cloud/TriangulatorConfig.h>
#include <sr_grasp_msgs/TriangulateAction.h>

// ROS PCL specific includes
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
  ros::Publisher pcl_output_pub_;   // Not defined in the architecture.
  ros::Publisher shape_output_pub_; // Officially defined in the architecture.

  std::string action_name_;
  actionlib::SimpleActionServer<sr_grasp_msgs::TriangulateAction> as_tri_;
  boost::shared_ptr<sr_grasp_msgs::TriangulateFeedback> feedback_tri_;
  boost::shared_ptr<sr_grasp_msgs::TriangulateResult> result_tri_;

  // Auto start the action server?
  static const bool auto_start_;

public:
  Triangulator(std::string node_name);
  ~Triangulator();

  void run(void);

protected:
  void config_cb_(TriangulatorConfig &config, uint32_t level);

  void cloud_cb_(const Cloud::ConstPtr &cloud);

  void goal_cb_(const sr_grasp_msgs::TriangulateGoalConstPtr &goal);

  void triangulate(const Cloud::ConstPtr &cloud,
                   pcl_msgs::PolygonMesh &pclMesh,
                   shape_msgs::Mesh &shapeMesh);

  void from_PCLPolygonMesh_(const pcl::PolygonMesh &pclMesh,
                            const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_with_normals,
                            shape_msgs::Mesh &shapeMesh);

}; // Triangulator

} // End of namespace sr_point_cloud

//-------------------------------------------------------------------------------
