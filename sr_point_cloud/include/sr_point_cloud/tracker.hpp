#include <sstream>
#include <typeinfo>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include "sr_point_cloud/TrackAction.h"

#include "sr_point_cloud/TrackerConfig.h"

#include <kdl/frames.hpp>

#include "sr_point_cloud/cluster_segmentor.h"

// ROS pcl includes
#include "pcl_conversions/pcl_conversions.h"

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/io/pcd_io.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>


namespace sr_point_cloud 
{

using namespace pcl::tracking;

// pcl::PointXYZ
template <class PointType>
class Tracker
{
public:
  typedef ParticleXYZRPY ParticleT;
  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename pcl::PointCloud<PointType>::Ptr CloudPtr;
  typedef typename pcl::PointCloud<PointType>::ConstPtr CloudConstPtr;
  typedef ParticleFilterTracker<PointType, ParticleT> ParticleFilter;
  typedef actionlib::SimpleActionServer<sr_point_cloud::TrackAction> TrackServer;

  enum SegmentSort
    {
      SEGMENT_SORT_BY_DISTANCE,
      SEGMENT_SORT_BY_CENTERED,
    };

  Tracker()
    : nh_home_("~")
    , track_server_(nh_home_, "track", false)
    , frame_id_("tracker")
    , downsampling_grid_size_(0.01)
    , filter_z_min_(0.0)
    , filter_z_max_(10.0)
    , downsample_(true)
  {
    // Setup ROS topics and services
    config_server_.setCallback(boost::bind(&Tracker::config_cb, this, _1, _2) );

    input_sub_ = nh_home_.subscribe("input/points", 1, &Tracker::cloud_cb, this);

    output_downsampled_pub_ = nh_home_.advertise<Cloud>("cloud_downsampled/points", 1);
    particle_cloud_pub_ = nh_home_.advertise<Cloud>("particle/points", 1);
    result_cloud_pub_ = nh_home_.advertise<Cloud>("result/points", 1);
    result_pose_pub_ = nh_home_.advertise<geometry_msgs::PoseStamped>("result/pose", 1);

    if (nh_home_.hasParam("downsample"))
      nh_home_.getParam("downsample", downsample_);

    track_server_.registerGoalCallback(boost::bind(&Tracker::track_goal_cb, this));
    track_server_.registerPreemptCallback(boost::bind(&Tracker::track_preempt_cb, this));
    track_server_.start();

    // Start tracking an empty cloud
    CloudPtr ref_cloud(new Cloud);
    trackCloud(ref_cloud);
  }

  void run() { ros::spin(); }

protected:
  /** Reset the tracker object to initial state.
   * Note: called by constructor.
   */
  void initTracker()
  {
    bool use_fixed = false;
    int thread_nr = 8;

    std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
    if (use_fixed)
    {
      boost::shared_ptr<ParticleFilterOMPTracker<PointType,
 ParticleT> > tracker(new ParticleFilterOMPTracker<PointType, ParticleT>(thread_nr));
      tracker_ = tracker;
    }
    else 
    {
      boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<PointType,
 ParticleT> > tracker(new KLDAdaptiveParticleFilterOMPTracker<PointType, ParticleT>(thread_nr));
      tracker->setMaximumParticleNum(500);
      tracker->setDelta(0.99);
      tracker->setEpsilon(0.2);
      ParticleT bin_size;
      bin_size.x = 0.1;
      bin_size.y = 0.1;
      bin_size.z = 0.1;
      bin_size.roll = 0.1;
      bin_size.pitch = 0.1;
      bin_size.yaw = 0.1;
      tracker->setBinSize(bin_size);
      tracker_ = tracker;
    }

    tracker_->setTrans(Eigen::Affine3f::Identity());
    tracker_->setStepNoiseCovariance(default_step_covariance);
    tracker_->setInitialNoiseCovariance(initial_noise_covariance);
    tracker_->setInitialNoiseMean(default_initial_mean);
    tracker_->setIterationNum(1);

    tracker_->setParticleNum(400);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal(false);

    setup_coherences();
  }

  virtual void
  setup_coherences(void)
  {
    // setup coherences
    typename ApproxNearestPairPointCloudCoherence<PointType>::Ptr coherence(new ApproxNearestPairPointCloudCoherence<PointType>());

    boost::shared_ptr<DistanceCoherence<PointType> > distance_coherence(new DistanceCoherence<PointType>());
    coherence->addPointCoherence(distance_coherence);

    boost::shared_ptr<pcl::search::Octree<PointType> > search(new pcl::search::Octree<PointType>(0.01));
    coherence->setSearchMethod(search);
    coherence->setMaximumDistance(0.01);
    tracker_->setCloudCoherence(coherence);
  }

  void
  config_cb(TrackerConfig &config, uint32_t level)
  {
    // ROS_INFO("Reconfigure Request: %f %f %f", config.downsampling_grid_size,
    // config.filter_z_min, config.filter_z_max );
    frame_id_ = config.frame_id;
    downsampling_grid_size_ = config.downsampling_grid_size;
    filter_z_min_ = config.filter_z_min;
    filter_z_max_ = config.filter_z_max;
  }

  void
  cloud_cb (const typename Cloud::ConstPtr& cloud)
  {
    input_ = cloud;

    cloud_pass_.reset(new Cloud);
    cloud_pass_downsampled_.reset(new Cloud);

    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(filter_z_min_, filter_z_max_);
    pass.setKeepOrganized(false);
    pass.setInputCloud(cloud);
    pass.filter(*cloud_pass_);

    if (downsample_)
    {
      // TODO(shadow): param toggle use of approx downsampling
      // gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
      gridSample(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
    }
    else
    {
      // We do this, as tracking() is using the cloud_pass_downsampled_
      cloud_pass_downsampled_ = cloud_pass_;
    }

    if (reference_->points.size() > 0)
      tracking();

    output_downsampled_pub_.publish(cloud_pass_downsampled_);
  }

  void
  track_goal_cb ()
  {
    TrackServer::GoalConstPtr goal = track_server_.acceptNewGoal();

    // Did we get a target
    std::string target = goal->target;
    if ( target != "" ) {
      if ( target == "nearest" )
        segmentReferece(SEGMENT_SORT_BY_DISTANCE);
      else if ( target == "centered" )
        segmentReferece(SEGMENT_SORT_BY_CENTERED);
      else
      {
        std::stringstream ss;
        ss << "Unknown target: " << target;
        ROS_ERROR_STREAM(ss.str());
        track_server_.setAborted(TrackResult(), ss.str());
        CloudPtr ref_cloud(new Cloud);
        trackCloud(ref_cloud);
      }
      return;
    }

    // No target, try the cloud
    CloudPtr ref_cloud(new Cloud);
    sensor_msgs::PointCloud2 input = goal->cloud;
    pcl::fromROSMsg(input, *ref_cloud);
    trackCloud(ref_cloud);
  }

  void
  track_preempt_cb ()
  {
    CloudPtr ref_cloud(new Cloud);
    trackCloud(ref_cloud);
    track_server_.setPreempted();
  }

  void
  tracking ()
  {
    tracker_->setInputCloud(cloud_pass_downsampled_);
    tracker_->compute();

    // Publish the particle cloud
    typename ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
    if (particles) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      for (size_t i = 0; i < particles->points.size(); i++) 
      {
        pcl::PointXYZ point;
        point.x = particles->points[i].x;
        point.y = particles->points[i].y;
        point.z = particles->points[i].z;
        particle_cloud->points.push_back(point);
      }
      // Copy the header so we get the right frame
      // XXX - Should we update the time stamp?
      particle_cloud->header = input_->header;
      particle_cloud_pub_.publish(*particle_cloud);
    }

    // Publish the result cloud
    // (tracker_->getReferenceCloud() for non-downsampled cloud)
    ParticleXYZRPY result = tracker_->getResult();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);
    CloudPtr result_cloud(new Cloud());
    pcl::transformPointCloud<PointType>(*reference_, *result_cloud, transformation);
    result_cloud->header = input_->header;
    result_cloud_pub_.publish (*result_cloud);

    std_msgs::Header ros_header = pcl_conversions::fromPCL(input_->header);

    // Publish the transformation (pose)
    geometry_msgs::PoseStamped pose;
    pose.header = ros_header;
    pose.pose.position.x = result.x;
    pose.pose.position.y = result.y;
    pose.pose.position.z = result.z;
    KDL::Rotation rot = KDL::Rotation::RPY(result.roll, result.pitch, result.yaw);
    rot.GetQuaternion(
                      pose.pose.orientation.x, pose.pose.orientation.y,
                      pose.pose.orientation.z, pose.pose.orientation.w);
    result_pose_pub_.publish(pose);

    // broadcast the frame of the tracked object
    tf::Vector3 vect(result.x,
                     result.y,
                     result.z);

    tf::Quaternion quat(pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z,
                        pose.pose.orientation.w);

    target_transform_.setOrigin(vect);
    target_transform_.setRotation(quat);
    target_broadcaster_.sendTransform(tf::StampedTransform(target_transform_,
                                                           ros_header.stamp,
                                                           ros_header.frame_id,
                                                           frame_id_));
  }

  void
  gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(result);
  }

  void
  gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    pcl::VoxelGrid<PointType> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(result);
  }

  bool
  segmentReferece(SegmentSort sort_type)
  {
    // If this fails we get an empty ref_cloud
    CloudPtr ref_cloud(new Cloud);
    std::vector<CloudPtr> clusters;
    ClusterSegmentor<PointType> cluster_segmentor;
    cluster_segmentor.setInputCloud(cloud_pass_downsampled_);

    ROS_INFO("Segmenting cloud...");
    if (sort_type == SEGMENT_SORT_BY_CENTERED)
      cluster_segmentor.extractByCentered(clusters);
    else
      cluster_segmentor.extractByDistance(clusters);
    ROS_INFO("... found %i clusters", static_cast<int>clusters.size());
    if (clusters.size() > 0)
      ref_cloud = clusters[0];

    trackCloud(ref_cloud);

    return true;
  }

  void
  trackCloud (const CloudConstPtr &ref_cloud)
  {
    Eigen::Vector4f c;
    CloudPtr transed_ref (new Cloud);
    pcl::compute3DCentroid<PointType> (*ref_cloud, c);
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    trans.translation() = Eigen::Vector3f (c[0], c[1], c[2]);
    pcl::transformPointCloud<PointType> (*ref_cloud, *transed_ref, trans.inverse ());

    initTracker();
    tracker_->setReferenceCloud(transed_ref);
    tracker_->setTrans(trans);
    tracker_->setMinIndices(ref_cloud->points.size () / 2);

    reference_ = transed_ref;
    ROS_INFO_STREAM("ref_cloud: "
                    << " points: " << ref_cloud->points.size()
                    << " wh:" << ref_cloud->width << "x" << ref_cloud->height
                    << " is_dense: " << (ref_cloud->is_dense ? "Yes" : "No"));
  }

  ros::NodeHandle nh_, nh_home_;
  std::string frame_id_;
  dynamic_reconfigure::Server<TrackerConfig> config_server_;
  ros::Subscriber input_sub_;
  ros::Publisher output_downsampled_pub_, particle_cloud_pub_, result_cloud_pub_, result_pose_pub_;
  TrackServer track_server_;
  CloudConstPtr input_;

  boost::shared_ptr<ParticleFilter> tracker_;
  CloudPtr cloud_pass_;
  CloudPtr cloud_pass_downsampled_;
  CloudPtr reference_;
  double downsampling_grid_size_;
  double filter_z_min_, filter_z_max_;

  bool downsample_;

  tf::TransformBroadcaster target_broadcaster_;
  tf::Transform target_transform_;
};  // Tracker

}  // namespace sr_point_cloud
