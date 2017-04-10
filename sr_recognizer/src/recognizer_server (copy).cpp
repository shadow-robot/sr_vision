#include "recognizer/recognizer_ros.h"




class openNIGrabber
 {
   public:
     pcl::visualization::CloudViewer viewer;
     pcl::PointCloud<PointT> cloudPublic;
     openNIGrabber ();
     void cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr &cloud);
     void save2cloud (pcl::PointCloud<PointT>::Ptr &cloud, bool vis);
 };

     openNIGrabber::openNIGrabber() : viewer ("PCL OpenNI Viewer")
     {

     }

void openNIGrabber::cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr &cloud)
     {


       if (!viewer.wasStopped())
         viewer.showCloud (cloud);

        cloudPublic = *cloud;

     }// end cloud_cb_

void openNIGrabber::save2cloud (pcl::PointCloud<PointT>::Ptr &cloud, bool vis)
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f =
         boost::bind (&openNIGrabber::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped() && vis)
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }
       boost::this_thread::sleep (boost::posix_time::seconds (1));

       interface->stop ();

       *cloud = cloudPublic;
     } //end save2cloud()


class RecognizerROS
{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<sr_recognizer::RecognizerAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
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
  RecognizerROS(std::string name) :
    as_(nh_, name, boost::bind(&RecognizerROS::recognize_cb, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~RecognizerROS(void)
  {
  }

  void checkCloudArrive (const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
      kinectCloudPtr.reset(new pcl::PointCloud<PointT>());
      pcl::fromROSMsg (*msg, *kinectCloudPtr);
      KINECT_OK_ = true;
  }

  bool checkKinect ()
  {
      ros::Subscriber sub_pc = nh_.subscribe (topic_, 1, &RecognizerROS::checkCloudArrive, this);
      ros::Rate loop_rate (1);
      size_t kinect_trials_ = 0;


      while (!KINECT_OK_ && ros::ok () && kinect_trials_ < 30)
      {
          std::cout << "Checking kinect status..." << std::endl;
          ros::spinOnce ();
          loop_rate.sleep ();
          kinect_trials_++;
      }
      return KINECT_OK_;
  }


  bool initialize() {

      std::string models_dir;
      if( nh_.getParam ( "recognizer_server/models_dir", models_dir )  && !models_dir.empty() )
      {
          arguments.push_back("-m");
          arguments.push_back(models_dir);
      }
      else
      {
          ROS_ERROR("Models directory is not set. Must be set with param \"m\"!");
          return false;
      }

      std::string additional_arguments;
      if( nh_.getParam ( "recognizer_server/arg", additional_arguments ) )
      {
          std::vector<std::string> strs;
          boost::split( strs, additional_arguments, boost::is_any_of("\t ") );
          arguments.insert( arguments.end(), strs.begin(), strs.end() );
      }


      std::cout << "Initializing recognizer with: " << std::endl;
      for( auto arg : arguments )
          std::cout << arg << " ";
      std::cout << std::endl;

      std::string recognizer_config;
      nh_.getParam ( "recognizer_server/config", recognizer_config);

      v4r::apps::ObjectRecognizerParameter param(recognizer_config);
      rec.reset(new v4r::apps::ObjectRecognizer<PointT>(param));
      rec->initialize(arguments);

      return true;
  }

  void recognize_cb(const sr_recognizer::RecognizerGoalConstPtr &goal)
  {

      ROS_INFO("Executing");

      pcl::PointCloud<PointT>::Ptr inputCloudPtr(new pcl::PointCloud<PointT>());

      //if path in the launch file for test_file is set, Recognizer uses the .pcd file instead the Kinect
      std::string test_file;
      if(nh_.getParam ( "recognizer_server/test_file", test_file )  && !test_file.empty() )
          pcl::io::loadPCDFile (test_file, *inputCloudPtr);
      else {

          if(!nh_.getParam ( "topic", topic_ ))
          {
              topic_ = "/camera/depth_registered/points";
          }
          std::cout << "Trying to connect to camera on topic " <<
                       topic_ << ". You can change the topic with param topic or " <<
                       " test pcd files from a directory by specifying param directory. " << std::endl;

          KINECT_OK_ = false;
          if ( checkKinect() )
          {
              std::cout << "Camera (topic: " << topic_ << ") is up and running." << std::endl;
          }
          else
          {
              std::cerr << "Camera (topic: " << topic_ << ") is not working." << std::endl;
              return ;
          }

         // openNIGrabber g;
         // g.save2cloud(inputCloudPtr, true);

          inputCloudPtr = kinectCloudPtr;
      }

      //pcl::io::savePCDFile ("/home/thomas/DA/test_pcd.pcd", *inputCloudPtr, 1);
      std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr > ohs = rec->recognize(inputCloudPtr);

      result_.ids.clear();
      result_.transforms.clear();

      for(size_t m_id=0; m_id<ohs.size(); m_id++)
      {
          std::cout << "************   " << ohs[m_id]->model_id_ << "   ************" << std::endl
                    << ohs[m_id]->transform_ << std::endl << std::endl;

          std_msgs::String ss_tmp;
          ss_tmp.data = ohs[m_id]->model_id_;
          result_.ids.push_back(ss_tmp);

          Eigen::Matrix4f trans = ohs[m_id]->transform_;
          geometry_msgs::Transform tt;
          tt.translation.x = trans(0,3);
          tt.translation.y = trans(1,3);
          tt.translation.z = trans(2,3);

          Eigen::Matrix3f rotation = trans.block<3,3>(0,0);
          Eigen::Quaternionf q(rotation);
          tt.rotation.x = q.x();
          tt.rotation.y = q.y();
          tt.rotation.z = q.z();
          tt.rotation.w = q.w();
          result_.transforms.push_back(tt);
      }

      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded(result_);

  } //end executeCB


};

int main(int argc, char** argv)
{
    ros::init (argc, argv, "recognizer_server");
    RecognizerROS recognizer("recognizer");

    recognizer.initialize();
    ros::spin();

    return 0;
}
