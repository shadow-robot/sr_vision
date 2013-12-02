/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   cluster_segmentor.cpp
 * @author Mark Addison <mark@shadowrobot.com>
 * @brief  Does...
 **/

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include "sr_point_cloud/ClusterSegmentorConfig.h"

#include "sr_point_cloud/cluster_segmentor.h"

namespace sr_point_cloud {

class ClusterSegmentorNode {

public:
    typedef pcl::PointXYZ PointType;

    /**
     * Construct a new ClusterSegmentorNode, setting up it's publishers,
     * subscribers, etc.
     */
    ClusterSegmentorNode() : nh_("~")
    {
        // Setup ROS topics and services
    }

    /**
     * Handles incoming dynamic reconfigure requests, updating the classes
     * members from the config.
     */
    void config_cb_(sr_point_cloud::ClusterSegmentorConfig &config, uint32_t level)
    {
      //cluster_segmentor_.setUseConvexHull(config.use_convex_hull);
      cluster_segmentor_.setClusterTolerance(config.cluster_tolerance);
      cluster_segmentor_.setMinClusterSize(config.min_cluster_size);
      cluster_segmentor_.setMaxClusterSize(config.max_cluster_size);
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
    ClusterSegmentor<PointType> cluster_segmentor_;

    dynamic_reconfigure::Server<sr_point_cloud::ClusterSegmentorConfig> config_server_;
};

} //sr_point_cloud

int main (int argc, char** argv)
{
  ros::init (argc, argv, "cluster_segmentor");
  sr_point_cloud::ClusterSegmentorNode node;
  node.spin();
  return 0;
}
