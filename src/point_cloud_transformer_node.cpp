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

#include <sensor_msgs/PointCloud2.h>
#include "sr_grasp_msgs/PclTransform.h"

namespace sr_point_cloud {

class PointCloudTransformerNode {

public:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> Cloud;

    /**
     * Construct a new PointCloudTransformerNode, setting up it's publishers,
     * subscribers, etc.
     */
    PointCloudTransformerNode()
      : nh_("~")
    {
        cloud_tf_server_ = nh_.advertiseService("transform_cloud", &PointCloudTransformerNode::transform_cloud_, this);
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
    tf::TransformListener tf_listener_;


    bool transform_cloud_(sr_grasp_msgs::PclTransform::Request  &req, sr_grasp_msgs::PclTransform::Response &res)
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
};

} //sr_point_cloud

int main (int argc, char** argv)
{
  ros::init (argc, argv, "point_cloud_transformer");
  sr_point_cloud::PointCloudTransformerNode node;
  node.spin();
  return 0;
}
