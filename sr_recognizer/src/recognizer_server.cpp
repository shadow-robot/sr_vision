/* Copyright 2017 ShadowRobot */

#include "recognizer/recognizer_ros.h"
#include <vector>
#include <string>

/*
 * Sets kineticCloudPtr from ROS message input 
 */
void RecognizerROS::checkCloudArrive(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    kinectCloudPtr.reset(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*msg, *kinectCloudPtr);
    KINECT_OK_ = true;
}

/*
 * Creates ROS subscriber  nh_ to recieve message from topic through checkCloudArrive function
 * and polls for KINECT_OK_ == true
 */
bool RecognizerROS::checkKinect()
{
    ros::Subscriber sub_pc = nh_.subscribe(topic_, 1, &RecognizerROS::checkCloudArrive, this);
    ros::Rate loop_rate(1);
    size_t kinect_trials_ = 0;

    while (!KINECT_OK_ && ros::ok () && kinect_trials_ < 30)
    {
        std::cout << "Checking kinect status..." << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
        kinect_trials_++;
    }
    return KINECT_OK_;
}

bool RecognizerROS::initialize()
{
    std::string models_dir;
    if (nh_.getParam("recognizer_server/models_dir", models_dir)  && !models_dir.empty())
    {
        arguments.push_back("-m");
        arguments.push_back(models_dir);
    }
    else
    {
        ROS_ERROR("Models directory is not set. Must be set with param \"m\"!");
        return false;
    }

    std::string hv_config_xml;
    if (nh_.getParam("recognizer_server/hv_config_xml", hv_config_xml))
    {
        arguments.push_back("--hv_config_xml");
        arguments.push_back(hv_config_xml);
    }

    std::string sift_config_xml;
    if (nh_.getParam("recognizer_server/sift_config_xml", sift_config_xml))
    {
        arguments.push_back("--sift_config_xml");
        arguments.push_back(sift_config_xml);
    }

    std::string shot_config_xml;
    if (nh_.getParam("recognizer_server/shot_config_xml", shot_config_xml))
    {
        arguments.push_back("--shot_config_xml");
        arguments.push_back(shot_config_xml);
    }

    std::string esf_config_xml;
    if (nh_.getParam("recognizer_server/esf_config_xml", esf_config_xml))
    {
        arguments.push_back("--esf_config_xml");
        arguments.push_back(esf_config_xml);
    }

    std::string alexnet_config_xml;
    if (nh_.getParam("recognizer_server/alexnet_config_xml", alexnet_config_xml))
    {
        arguments.push_back("--alexnet_config_xml");
        arguments.push_back(alexnet_config_xml);
    }

    std::string camera_xml;
    if (nh_.getParam("recognizer_server/camera_xml", camera_xml))
    {
        arguments.push_back("--camera_xml");
        arguments.push_back(camera_xml);
    }

    std::string additional_arguments;
    if (nh_.getParam("recognizer_server/arg", additional_arguments))
    {
        std::vector<std::string> strs;
        boost::split(strs, additional_arguments, boost::is_any_of("\t "));
        arguments.insert(arguments.end(), strs.begin(), strs.end());
    }

    std::string recognizer_config;
    nh_.getParam("recognizer_server/multipipeline_config_xml", recognizer_config);

    std::cout << "Initialized recognizer with: " << std::endl;
    std::cout << "--multipipeline_config_xml" << std::endl;
    std::cout << recognizer_config << std::endl;

    for (auto arg : arguments)
    {
       std::cout << arg << " ";
       std::cout << std::endl;
    }

    v4r::apps::ObjectRecognizerParameter param(recognizer_config);
    rec.reset(new v4r::apps::ObjectRecognizer<PointT>(param));

    return true;
}

void RecognizerROS::recognize_cb(const sr_recognizer::RecognizerGoalConstPtr &goal)
{
    static bool init = true;
    ROS_INFO("Executing");
    pcl::PointCloud<PointT>::Ptr inputCloudPtr(new pcl::PointCloud<PointT>());

    //  if path in the launch file for test_file is set, Recognizer uses the .pcd file instead the Kinect
    std::string test_file;
    if (nh_.getParam ("recognizer_server/test_file", test_file )  && !test_file.empty())
        pcl::io::loadPCDFile(test_file, *inputCloudPtr);
    else
    {
        if (!nh_.getParam("topic", topic_ ))
        {
            topic_ = "/camera/depth_registered/points";
        }
        std::cout << "Trying to connect to camera on topic " <<
                     topic_ << ". You can change the topic with param topic or " <<
                     " test pcd files from a directory by specifying param directory. " << std::endl;

        KINECT_OK_ = false;
        if (checkKinect())  // Updates kineticCloudPtr from topic
        {
            std::cout << "Camera (topic: " << topic_ << ") is up and running." << std::endl;
        }
        else
        {
            std::cerr << "Camera (topic: " << topic_ << ") is not working." << std::endl;
            return;
        }
        inputCloudPtr = kinectCloudPtr;
    }

    if (init)
    {
      rec->initialize(arguments);
      init = false;
    }

    // Start recognition on input image (inputCloudPtr)
    std::cout << "Start Recognition" << std::endl;

    std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr > ohs = rec->recognize(inputCloudPtr);

    std::cout << "Finished Recognition" << std::endl;

    result_.ids.clear();
    result_.confidences.clear();
    result_.transforms.clear();

    for (size_t m_id = 0; m_id < ohs.size(); m_id++)
    {
        std::cout << "************   " << ohs[m_id]->model_id_ << "   ************" << std::endl
                  << ohs[m_id]->transform_ << std::endl << std::endl;

        std_msgs::String ss_tmp;
        ss_tmp.data = ohs[m_id]->model_id_;
        result_.ids.push_back(ss_tmp);

        float confidence = ohs[m_id]->confidence_;
        result_.confidences.push_back(confidence);

        Eigen::Matrix4f trans = ohs[m_id]->transform_;
        geometry_msgs::Transform tt;
        tt.translation.x = trans(0, 3);
        tt.translation.y = trans(1, 3);
        tt.translation.z = trans(2, 3);

        Eigen::Matrix3f rotation = trans.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation);
        tt.rotation.x = q.x();
        tt.rotation.y = q.y();
        tt.rotation.z = q.z();
        tt.rotation.w = q.w();
        result_.transforms.push_back(tt);
    }

    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "recognizer_server");
    RecognizerROS recognizer("recognizer");

    recognizer.initialize();
    ros::spin();

    return 0;
}
