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
#include "sr_point_cloud/cluster_segmentor.h"

#include <dynamic_reconfigure/server.h>
#include "sr_point_cloud/ClusterSegmentorConfig.h"
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>  // Allow use of PCL cloud types for pubs and subs
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>

#include <sensor_msgs/PointCloud2.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>
#include <actionlib/server/simple_action_server.h>

#include <string>
#include <vector>

namespace sr_point_cloud
{

using std::vector;
using namespace object_recognition_msgs;

class ClusterSegmentorNode
{
public:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> Cloud;
    typedef actionlib::SimpleActionServer<ObjectRecognitionAction> RecognitionServer;

    /**
     * Construct a new ClusterSegmentorNode, setting up it's publishers,
     * subscribers, etc.
     */
    ClusterSegmentorNode()
      : nh_("~")
      , remove_nan_(true)
      , recognize_objects_as_(nh_, "recognize_objects",
boost::bind(&ClusterSegmentorNode::recognize_objects_execute_cb_, this, _1), false)
    {
        config_server_.setCallback(boost::bind(&ClusterSegmentorNode::config_cb_, this, _1, _2));
        input_cloud_sub_ = nh_.subscribe("input/points", 1, &ClusterSegmentorNode::cloud_cb_, this);
        nh_.param<std::string>("output_frame", output_frame_, "");

        // @todo - would be nice to add a streaming option to the node that
        // constantly streams out the found clusters when on. Will need some
        // messing around with locks to get this to play nice with the actionlib
        // interface at the same time.
        // See cloud_cb_ for publish code.
        // output_objects_pub_ = nh_.advertise<RecognizedObjectArray>("output/clusters", 1);

        recognize_objects_as_.start();
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
    Cloud::ConstPtr input_cloud_;
    bool remove_nan_;
    ClusterSegmentor<PointType> cluster_segmentor_;
    dynamic_reconfigure::Server<sr_point_cloud::ClusterSegmentorConfig> config_server_;
    ros::Subscriber input_cloud_sub_;
    ros::Publisher output_objects_pub_;
    RecognitionServer recognize_objects_as_;
    std::string output_frame_;
    tf::TransformListener tf_listener_;

    void config_cb_(sr_point_cloud::ClusterSegmentorConfig &config, uint32_t level)
    {
      cluster_segmentor_.setUseConvexHull(config.use_convex_hull);
      cluster_segmentor_.setClusterTolerance(config.cluster_tolerance);
      cluster_segmentor_.setMinClusterSize(config.min_cluster_size);
      cluster_segmentor_.setMaxClusterSize(config.max_cluster_size);
    }

    void cloud_cb_(const Cloud::ConstPtr& cloud)
    {
      if (remove_nan_)
      {
        // Convex hull does not like a cloud with NaN in (which kinect give us).
        vector<int> indices;
        Cloud::Ptr clean_cloud(new Cloud);
        pcl::removeNaNFromPointCloud(*cloud, *clean_cloud, indices);
        input_cloud_ = clean_cloud;
        // ROS_INFO_STREAM("Segmenting cloud: " << *cloud << "\n" << *input_cloud_);
      }
      else
        input_cloud_ = cloud;

      // @todo - streaming interface, see constructor
      // RecognizedObjectArray objs;
      // extract_(objs);
      // output_objects_pub_.publish(objs);
    }

    void recognize_objects_execute_cb_(const RecognitionServer::GoalConstPtr& goal)
    {
      ObjectRecognitionResult res;
      RecognizedObjectArray objs;
      extract_(objs, goal->use_roi, goal->filter_limits);
      res.recognized_objects = objs;
      recognize_objects_as_.setSucceeded(res);
    }

    void extract_(RecognizedObjectArray &out, bool use_roi, const std::vector<float> & filter_limits)
    {
      if (input_cloud_->points.empty())
        return;

      vector<Cloud::Ptr> clusters;

      if (use_roi)
      {
        ROS_ASSERT(filter_limits.size() == 6);

        Cloud::Ptr cloud_pass_x;
        Cloud::Ptr cloud_pass_y;
        Cloud::Ptr cloud_pass_z;

        cloud_pass_x.reset (new Cloud);
        cloud_pass_y.reset (new Cloud);
        cloud_pass_z.reset (new Cloud);

        pcl::PassThrough<PointType> pass;

        pass.setFilterFieldName("x");
        pass.setFilterLimits(filter_limits[0], filter_limits[1]);
        pass.setKeepOrganized(false);
        pass.setInputCloud(input_cloud_);
        pass.filter(*cloud_pass_x);

        pass.setFilterFieldName("y");
        pass.setFilterLimits(filter_limits[2], filter_limits[3]);
        pass.setKeepOrganized(false);
        pass.setInputCloud(cloud_pass_x);
        pass.filter(*cloud_pass_y);

        pass.setFilterFieldName("z");
        pass.setFilterLimits(filter_limits[4], filter_limits[5]);
        pass.setKeepOrganized(false);
        pass.setInputCloud(cloud_pass_y);
        pass.filter(*cloud_pass_z);

        cluster_segmentor_.setInputCloud(cloud_pass_z);
      }
      else
        cluster_segmentor_.setInputCloud(input_cloud_);

      ROS_INFO("Segmenting cloud...");
      cluster_segmentor_.extract(clusters);
      ROS_INFO("... found %i clusters", static_cast<int>(clusters.size()));

      std_msgs::Header head = pcl_conversions::fromPCL(input_cloud_->header);
      if (!output_frame_.empty())
      {
        head.frame_id = output_frame_;
      }

      for (vector<Cloud::Ptr>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
        Cloud::Ptr cluster_cloud = *it;

        // If an output frame has been specified, we convert each one of the clusters to that frame
        if (!output_frame_.empty())
        {
          cluster_cloud->header = input_cloud_->header;
          Cloud::Ptr transformed_cluster_cloud (new Cloud);

          pcl_ros::transformPointCloud(output_frame_, *cluster_cloud, *transformed_cluster_cloud, tf_listener_);
          cluster_cloud = transformed_cluster_cloud;
        }

        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg(*cluster_cloud, pc2);
        pc2.header = head;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid<PointType>(*cluster_cloud, centroid);

        RecognizedObject obj;
        obj.header = head;
        obj.point_clouds.push_back(pc2);
        obj.pose.header = head;
        obj.pose.pose.pose.position.x = centroid.x();
        obj.pose.pose.pose.position.y = centroid.y();
        obj.pose.pose.pose.position.z = centroid.z();
        out.objects.push_back(obj);
      }
    }
};

}  // namespace sr_point_cloud

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cluster_segmentor");
  sr_point_cloud::ClusterSegmentorNode node;
  node.spin();
  return 0;
}
