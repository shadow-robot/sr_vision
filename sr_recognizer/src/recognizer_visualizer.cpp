#include "ros/ros.h"
#include <sr_recognizer/RecognizerAction.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <v4r/core/macros.h>
#include <v4r/common/pcl_visualization_utils.h>
#include <v4r/recognition/local_feature_matching.h>
#include <v4r/recognition/object_hypothesis.h>
#include <v4r/recognition/source.h>

#include <v4r/apps/visualization.h>
#include <v4r/recognition/model.h>
#include <pcl/common/transforms.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

typedef pcl::PointXYZRGB PointT;

class visualizer
{
private:
     boost::shared_ptr<ros::NodeHandle> n_;
     boost::shared_ptr<ros::Subscriber> sub_rec_;
     boost::shared_ptr<ros::Subscriber> sub_kin_;
     boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
     int vp1a_, vp1b_, vp2_;

public:

bool visualizer_init(int argc, char ** argv)
{
    ros::init(argc, argv, "visualizer");
    n_.reset(new ros::NodeHandle( "~" ));
    sub_rec_.reset(new ros::Subscriber);
    *sub_rec_ = n_->subscribe("/recognizer/result", 1, &visualizer::visualizer_cb_, this);

    sub_kin_.reset(new ros::Subscriber);
    *sub_kin_ = n_->subscribe("/camera/depth_registered/points", 1, &visualizer::kinect_cb_, this);

    if(!vis_)
    {
        vis_.reset(new pcl::visualization::PCLVisualizer("single-view recognition results"));
        vis_->createViewPort(0  , 0  , 0.5 , 0.5, vp1a_);
        vis_->createViewPort(0.5, 0  , 1   , 0.5, vp1b_);
        vis_->createViewPort(0  , 0.5, 1   , 1  , vp2_);
    }

    vis_->addText("recognized objects (no data)", 10, 10, 20, 1, 1, 1, "input_text_vp1b", vp1b_);
    vis_->addText("input cloud for recognizer (no data)", 10, 10, 20, 1, 1, 1, "input_text_vp1a", vp1a_);
    vis_->addText("live camera point cloud", 10, 10, 20, 1, 1, 1, "input_test_vp2", vp2_);

    vis_->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
    vis_->resetCamera();

    return true;
}

void kinect_cb_(const sensor_msgs::PointCloud2::ConstPtr& msg )
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (*msg, *cloud);

    vis_->removeAllPointClouds(vp2_);

    vis_->addPointCloud(cloud, "input_vp2", vp2_);
}

void visualizer_cb_(const sr_recognizer::RecognizerActionResult &result_)
{
    pcl::PointCloud<PointT>::Ptr cloud_(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (result_.result.input_scene, *cloud_);

    vis_->removeAllShapes();
    std::stringstream verified_hyp_ss; verified_hyp_ss << "recognized objects: " << result_.result.ids.size();
    vis_->addText("input cloud for recognizer", 10, 10, 20, 1, 1, 1, "input_text_vp1a", vp1a_);
    vis_->addText("live camera picture", 10, 10, 20, 1, 1, 1, "input_test_vp2", vp2_);

    vis_->addText(verified_hyp_ss.str(), 10, 10, 20, 0, 0, 0, "verified hypotheses_text", vp1b_);

   // vis_->removeAllPointClouds();
    vis_->removeAllPointClouds(vp1a_);
    vis_->removeAllPointClouds(vp1b_);

    typename pcl::PointCloud<PointT>::Ptr vis_cloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud_, *vis_cloud);
    vis_cloud->sensor_origin_ = Eigen::Vector4f::Zero();
    vis_cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();


    for(size_t i=0; i<result_.result.ids.size(); i++)
    {
        const std::string model_id = result_.result.ids[i].data;
        std::stringstream model_label;
        model_label << model_id << "_verified_" << i;
        typename pcl::PointCloud<PointT>::Ptr model_cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg (result_.result.model_cloud[i], *model_cloud);

        vis_->addPointCloud(model_cloud, model_label.str(), vp1b_);
    }

    vis_->addPointCloud(vis_cloud, "input_vp1a", vp1a_);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> gray3 (vis_cloud, 128, 128, 128);
    vis_->addPointCloud(vis_cloud, gray3, "input_vp3", vp1b_);
    vis_->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "input_vp3");
    vis_->setBackgroundColor(1.f, 1.f, 1.f, vp1b_);
}

void run()
{
    ros::Rate r(10);
    while(!vis_->wasStopped() && ros::ok())
    {
        ros::spinOnce();
        vis_->spinOnce();
        r.sleep();
    }
}

};

int main(int argc, char **argv)
{
  visualizer v;
  v.visualizer_init(argc, argv);
  v.run();

  return 0;
}
