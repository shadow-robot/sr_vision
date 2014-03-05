#include "sr_point_cloud/triangulator.hpp"

//-------------------------------------------------------------------------------

using namespace sr_point_cloud;

//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------

Triangulator::Triangulator()
  : nh_("~")
  , resample_(true)
  , mu_(2.5)
  , maximum_nearest_neighbors_(100)
  , maximum_surface_angle_(M_PI/4) // 45 degs
  , minimum_angle_(M_PI/18) // 10 degs
  , maximum_angle_(2*M_PI/3) // 120 degs
  , gp3_search_radius_(0.025)
  , mls_search_radius_(0.03)
  , action_name_("triangulate")
  , as_tri_(nh_,
            action_name_,
            boost::bind(&Triangulator::goal_cb_, this, _1),
            false)
{
  // Setup ROS topics and services
  config_server_.setCallback( boost::bind(&Triangulator::config_cb_, this, _1, _2) );
  input_sub_ = nh_.subscribe("input/points", 1, &Triangulator::cloud_cb_, this);
  pcl_output_pub_ = nh_.advertise<pcl_msgs::PolygonMesh>("output/pcl/mesh", 1);
  shape_output_pub_ = nh_.advertise<shape_msgs::Mesh>("output/shape/mesh", 1);

  feedback_tri_.reset(new sr_grasp_msgs::TriangulateFeedback);
  result_tri_.reset(new sr_grasp_msgs::TriangulateResult);

  as_tri_.start();
  ROS_INFO_STREAM("Action server " << action_name_ << " just started.");
}

//-------------------------------------------------------------------------------

Triangulator::~Triangulator()
{
}

//-------------------------------------------------------------------------------

void Triangulator::run(void)
{
  ros::spin();
}

//-------------------------------------------------------------------------------

void Triangulator::config_cb_(TriangulatorConfig &config, uint32_t level)
{
  resample_ = config.resample;
  mu_ = config.mu;
  maximum_nearest_neighbors_ = config.maximum_nearest_neighbors;
  maximum_surface_angle_ = config.maximum_surface_angle;
  minimum_angle_ = config.minimum_angle;
  maximum_angle_ = config.maximum_angle;
}

//-------------------------------------------------------------------------------

/*
 * Runs the triangulation. Based on:
 * http://www.pointclouds.org/documentation/tutorials/greedy_projection.php
 * http://pointclouds.org/documentation/tutorials/resampling.php
 *
 * Note that the input type is sensor_msgs::PointCloud2 according to the documentation.
 * However, ROS converts automatically from that type to pcl::PointCloud<pcl::PointXYZ>.
 */
void Triangulator::cloud_cb_(const Cloud::ConstPtr &cloud)
{
  // Triangulate.
  pcl_msgs::PolygonMesh pclMesh;
  shape_msgs::Mesh shapeMesh;
  this->triangulate(cloud, pclMesh, shapeMesh);

  // Publish the mesh.
  pcl_output_pub_.publish(pclMesh);
  shape_output_pub_.publish(shapeMesh);
}

//-------------------------------------------------------------------------------

void Triangulator::goal_cb_(const sr_grasp_msgs::TriangulateGoalConstPtr &goal)
{
  // http://wiki.ros.org/hydro/Migration#PCL
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(goal->point_cloud, pcl_pc2);

  // Convert to pcl::PointCloud.
  boost::shared_ptr<Cloud> cloud(new Cloud);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  // Publish info to the console for the user.
  ROS_INFO_STREAM("Action " << action_name_ << ": Executing Triangulator::goal_cb_");

  // Triangulate.
  pcl_msgs::PolygonMesh pclMesh;
  shape_msgs::Mesh shapeMesh;
  this->triangulate(cloud, pclMesh, shapeMesh, goal->mirror_mesh);
  result_tri_->pcl_mesh = pclMesh;
  result_tri_->shape_mesh = shapeMesh;

  // Set the action state to succeeded.
  as_tri_.setSucceeded(*result_tri_);
  ROS_INFO_STREAM("Action " << action_name_ << ": Succeeded");
}

//-------------------------------------------------------------------------------

void Triangulator::triangulate(const Cloud::ConstPtr &cloud,
                               pcl_msgs::PolygonMesh &pclMesh,
                               shape_msgs::Mesh &shapeMesh,
                               bool mirror_mesh)
{
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);

  // Concatenate the XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals;
  if (!resample_)
  {
    // Follow "Fast triangulation of unordered point clouds" @
    // http://www.pointclouds.org/documentation/tutorials/greedy_projection.php

    // Normal estimation
    pcl::NormalEstimation<PointType, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    // normals should contain the point normals + surface curvatures

    cloud_with_normals.reset (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  }
  else
  {
    // Use MSL surface reconstruction method to smooth and resample noisy data.
    // URL: http://pointclouds.org/documentation/tutorials/resampling.php

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointType, pcl::PointNormal> mls;

    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (mls_search_radius_);

    // Reconstruct.
    pcl::PointCloud<pcl::PointNormal> mls_points;
    mls.process (mls_points);

    // Set shared_ptr cloud_with_normals
    cloud_with_normals = boost::make_shared< pcl::PointCloud<pcl::PointNormal> > (mls_points);
  }
  // cloud_with_normals = cloud + normals

  // Create search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (gp3_search_radius_);

  // Set typical values for the parameters
  gp3.setMu (mu_);
  gp3.setMaximumNearestNeighbors (maximum_nearest_neighbors_);
  gp3.setMaximumSurfaceAngle(maximum_surface_angle_);
  gp3.setMinimumAngle(minimum_angle_);
  gp3.setMaximumAngle(maximum_angle_);
  // The triangles generatea by GreedyProjectionTriangulation are often misdirected.
  // Solution: Use gp3.setNormalConsistency(true).
  // URL: http://www.pcl-users.org/Help-GreedyProjectionTriangulation-td3637361.html
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // Convert to ROS type
  pcl_conversions::fromPCL(triangles, pclMesh);

  // Convert to shape_msgs::Mesh type
  // The given cloud should NOT be used after resampling.
  this->from_PCLPolygonMesh_(triangles, cloud_with_normals, shapeMesh);

  // Debug
  // pcl::io::saveVTKFile("mesh.vtk", triangles);
}

//-------------------------------------------------------------------------------

void Triangulator::from_PCLPolygonMesh_(const pcl::PolygonMesh &pclMesh,
                                        const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_with_normals,
                                        shape_msgs::Mesh &shapeMesh)
{
  const std::vector<pcl::Vertices> &polygons = pclMesh.polygons;

  // Set the actual vertices that make up the mesh.
  for (size_t i = 0; i < cloud_with_normals->size(); i++)
  {
    geometry_msgs::Point vertex;
    const pcl::PointNormal &curr_point = cloud_with_normals->at(i);
    vertex.x = curr_point.x;
    vertex.y = curr_point.y;
    vertex.z = curr_point.z;
    shapeMesh.vertices.push_back(vertex);
  }

  // Set the list of triangles.
  for (size_t i = 0; i < polygons.size(); i++)
  {
    shape_msgs::MeshTriangle triangle;
    const pcl::Vertices &curr_poly = polygons[i];
    ROS_ASSERT(curr_poly.vertices.size() == 3);
    triangle.vertex_indices[0] = curr_poly.vertices[0];
    triangle.vertex_indices[1] = curr_poly.vertices[1];
    triangle.vertex_indices[2] = curr_poly.vertices[2];
    shapeMesh.triangles.push_back(triangle);
  }
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

int main (int argc, char** argv)
{
  ros::init (argc, argv, "point_cloud_triangulator");
  sr_point_cloud::Triangulator tri_node;
  tri_node.run();
  return 0;
}

//-------------------------------------------------------------------------------
