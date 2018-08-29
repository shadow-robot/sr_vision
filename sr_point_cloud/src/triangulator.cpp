/*
 * Copyright (c) 2015 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

#include "sr_point_cloud/triangulator.hpp"
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <cmath>
#include <vector>


//-------------------------------------------------------------------------------

using sr_point_cloud::Triangulator;

//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------

Triangulator::Triangulator()
  : nh_("~")
  , resample_(true)
  , mu_(2.5)
  , maximum_nearest_neighbors_(100)
  , maximum_surface_angle_(M_PI/4)  // 45 degs
  , minimum_angle_(M_PI/18)  // 10 degs
  , maximum_angle_(2*M_PI/3)  // 120 degs
  , gp3_search_radius_(0.025)
  , mls_search_radius_(0.03)
  , action_name_("triangulate")
  , as_tri_(nh_,
            action_name_,
            boost::bind(&Triangulator::goal_cb_, this, _1),
            false)
{
  // Setup ROS topics and services
  config_server_.setCallback(boost::bind(&Triangulator::config_cb_, this, _1, _2));
  input_sub_ = nh_.subscribe("input/points", 1, &Triangulator::cloud_cb_, this);
  pcl_output_pub_ = nh_.advertise<pcl_msgs::PolygonMesh>("output/pcl/mesh", 1);
  shape_output_pub_ = nh_.advertise<shape_msgs::Mesh>("output/shape/mesh", 1);

  feedback_tri_.reset(new sr_vision_msgs::TriangulateFeedback);
  result_tri_.reset(new sr_vision_msgs::TriangulateResult);

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

void Triangulator::config_cb_(const TriangulatorConfig &config, uint32_t level)
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
  this->triangulate(cloud, &pclMesh, &shapeMesh);

  // Publish the mesh.
  pcl_output_pub_.publish(pclMesh);
  shape_output_pub_.publish(shapeMesh);
}

//-------------------------------------------------------------------------------

void Triangulator::goal_cb_(const sr_vision_msgs::TriangulateGoalConstPtr &goal)
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
  if (goal->mirror_mesh)
    mirror_mesh_(cloud.get());
  this->triangulate(cloud, &pclMesh, &shapeMesh);

  if (pclMesh.polygons.size() > 2)
  {
    result_tri_->pcl_mesh = pclMesh;
    result_tri_->shape_mesh = shapeMesh;
    as_tri_.setSucceeded(*result_tri_);
    ROS_INFO_STREAM("Action " << action_name_ << ": Succeeded");
  }
  else
  {
    as_tri_.setAborted();
    ROS_ERROR_STREAM("Action " << action_name_ << ": Failed");
  }
}

//-------------------------------------------------------------------------------

void Triangulator::triangulate(const Cloud::ConstPtr &cloud,
                               pcl_msgs::PolygonMesh *pclMesh,
                               shape_msgs::Mesh *shapeMesh)
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
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    // normals should contain the point normals + surface curvatures

    cloud_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
  }
  else
  {
    // Use MSL surface reconstruction method to smooth and resample noisy data.
    // URL: http://pointclouds.org/documentation/tutorials/resampling.php

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointType, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(mls_search_radius_);

    // Reconstruct.
    pcl::PointCloud<pcl::PointNormal> mls_points;
    mls.process(mls_points);

    // Set shared_ptr cloud_with_normals
    cloud_with_normals = boost::make_shared< pcl::PointCloud<pcl::PointNormal> > (mls_points);
  }
  // cloud_with_normals = cloud + normals

  // Create search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(gp3_search_radius_);

  // Set typical values for the parameters
  gp3.setMu(mu_);
  gp3.setMaximumNearestNeighbors(maximum_nearest_neighbors_);
  gp3.setMaximumSurfaceAngle(maximum_surface_angle_);
  gp3.setMinimumAngle(minimum_angle_);
  gp3.setMaximumAngle(maximum_angle_);
  // The triangles generated by GreedyProjectionTriangulation are often misdirected.
  // Solution: Use gp3.setNormalConsistency(true).
  // URL: http://www.pcl-users.org/Help-GreedyProjectionTriangulation-td3637361.html
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(triangles);

  // Convert to ROS type
  pcl_conversions::fromPCL(triangles, *pclMesh);

  // Convert to shape_msgs::Mesh type
  // The given cloud should NOT be used after resampling.
  this->from_PCLPolygonMesh_(triangles, cloud_with_normals, shapeMesh);

  // Debug
  // pcl::io::saveVTKFile("mesh.vtk", triangles);
}

//-------------------------------------------------------------------------------

void Triangulator::from_PCLPolygonMesh_(const pcl::PolygonMesh &pclMesh,
                                        const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_with_normals,
                                        shape_msgs::Mesh *shapeMesh)
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
    shapeMesh->vertices.push_back(vertex);
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
    shapeMesh->triangles.push_back(triangle);
  }
}

void Triangulator::mirror_mesh_(Cloud *cloud)
{
  using Eigen::Vector4f;
  using Eigen::Vector3f;
  using Eigen::Affine3f;
  using Eigen::Translation3f;
  using pcl::TransformationFromCorrespondences;

  Vector4f centroid4;
  compute3DCentroid(*cloud, centroid4);
  const Vector3f centroid = centroid4.head(3);

  // calculate oriented bounded box
  PointType min_pt, max_pt;
  getMinMax3D(*cloud, min_pt, max_pt);

  //  calculate transformation based on point correspondences

  //  0---1
  //  |   |
  //  |   |
  //  2---3

  //  correspondences will be 0 - 1  and  2 - 3
  //  one of these points is max_pt and the other 3 will be constructed by adding x and y from min_pt
  Vector3f points[] =
  {
    Vector3f(min_pt.x, max_pt.y, max_pt.z),
    Vector3f(max_pt.x, max_pt.y, max_pt.z),
    Vector3f(min_pt.x, min_pt.y, max_pt.z),
    Vector3f(max_pt.x, min_pt.y, max_pt.z)
  };

  // get transformation according to back side of bounding box
  // If the point cloud includes outlier points the bounding box will be much bigger than it should
  // No filtering is done here for that case and another getting a new point cloud is recommended
  TransformationFromCorrespondences transformer;
  transformer.add(points[0], points[1]);
  transformer.add(points[1], points[0]);
  transformer.add(points[2], points[3]);
  transformer.add(points[3], points[2]);
  Affine3f transformationMatrix = transformer.getTransformation();

  // When the object is offset from the center of the field of view
  // The point cloud will be rotated and the bounding box will be deeper
  // So the mirror mesh appears farther than it should
  // This translation is meant to fix that
  float x2z = fabs(centroid.x()/centroid.z());
  transformationMatrix *= Translation3f(0, 0, 0.25*x2z*(max_pt.x - min_pt.x));

  // Construct the mirrored point cloud and append its points to the original
  Cloud mirrored_cloud;
  transformPointCloud(*cloud, mirrored_cloud, transformationMatrix);
  *cloud += mirrored_cloud;
}


//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_triangulator");
  sr_point_cloud::Triangulator tri_node;
  tri_node.run();
  return 0;
}

//-------------------------------------------------------------------------------
