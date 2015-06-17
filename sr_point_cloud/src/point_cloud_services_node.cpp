/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   point_cloud_transformer_node.cpp
 * @author Toni Oliver <toni@shadowrobot.com>
 * @brief  Does...
 **/

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h> // Allow use of PCL cloud types for pubs and subs
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include "sr_vision_msgs/PclTransform.h"
#include "sr_vision_msgs/PclFilter.h"

namespace sr_point_cloud {

class PointCloudServicesNode {

public:
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    /**
     * Construct a new PointCloudServicesNode, setting up it's publishers,
     * subscribers, etc.
     */
    PointCloudServicesNode()
      : nh_("~")
    {
      cloud_tf_server_ = nh_.advertiseService("transform_cloud", &PointCloudServicesNode::transform_cloud_, this);
      //This is a service that will allow to filter a single point cloud message (apply VoxelGrid for the time being)
      cloud_filter_server_ = nh_.advertiseService("filter_cloud", &PointCloudServicesNode::filter_cloud_, this);
    }

    /**
     * Set the node spinning. ros::init should have already been called.
     */
    void spin()
    {
        ros::spin();
    }

protected:
    ros::NodeHandle nh_;
    ros::ServiceServer cloud_tf_server_;
    ros::ServiceServer cloud_filter_server_;
    tf::TransformListener tf_listener_;


    bool
    transform_cloud_(sr_vision_msgs::PclTransform::Request  &req, sr_vision_msgs::PclTransform::Response &res)
    {
      // If an output frame has been specified, we convert the cloud to that frame
      if (!req.output_frame_id.empty())
      {
        pcl_ros::transformPointCloud(req.output_frame_id, req.point_cloud, res.point_cloud, tf_listener_);
        return true;
      }

      res.point_cloud = req.point_cloud;
      return true;
    }

    void
    gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
    {
      pcl::VoxelGrid<PointType> grid;
      grid.setLeafSize (leaf_size, leaf_size, leaf_size);
      grid.setInputCloud (cloud);
      grid.filter (result);
    }

    bool
    filter_cloud_(sr_vision_msgs::PclFilter::Request  &req, sr_vision_msgs::PclFilter::Response &res)
    {
      Cloud downsampled_cloud;
      CloudPtr input_cloud (new Cloud);

      pcl::fromROSMsg(req.point_cloud, *input_cloud);

      gridSample(input_cloud, downsampled_cloud, req.downsampling_grid_size);

      pcl::toROSMsg(downsampled_cloud, res.point_cloud);

      return true;
    }
};

} //sr_point_cloud

int main (int argc, char** argv)
{
  ros::init (argc, argv, "point_cloud_transformer");
  sr_point_cloud::PointCloudServicesNode node;
  node.spin();
  return 0;
}
