#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sr_recognizer/RecognizerAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <iostream>
#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include <v4r/io/filesystem.h>
#include <v4r/apps/ObjectRecognizer.h>

typedef pcl::PointXYZRGB PointT;

class RecognizerROS
{
private:
    ros::NodeHandle nh_;  // NodeHandle instance must be created before next line. Otherwise strange error occurs.
    actionlib::SimpleActionServer<sr_recognizer::RecognizerAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    sr_recognizer::RecognizerFeedback feedback_;
    sr_recognizer::RecognizerResult result_;

    boost::shared_ptr<v4r::apps::ObjectRecognizer<PointT>> rec;
    std::vector<std::string> arguments;
    pcl::PointCloud<PointT>::Ptr kinectCloudPtr;
    std::string topic_;
    bool KINECT_OK_;

public:
    RecognizerROS(std::string name):
        as_(nh_, name, boost::bind(&RecognizerROS::recognize_cb, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~RecognizerROS(void)
    {
    }

    void checkCloudArrive(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool checkKinect();
    bool initialize();
    void recognize_cb(const sr_recognizer::RecognizerGoalConstPtr &goal);
};


