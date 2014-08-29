#include "sr_point_cloud/tracker.hpp"

namespace sr_point_cloud {

// pcl::PointXYZRGB
class TrackerRGB : public Tracker<pcl::PointXYZRGB>
{
public:
  TrackerRGB()
    : Tracker()
  {
  }

protected:
  virtual void
  setup_coherences(void)
  {
    // setup coherences
    typename ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGB>::Ptr coherence(new ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGB>());

    boost::shared_ptr<DistanceCoherence<pcl::PointXYZRGB> > distance_coherence(new DistanceCoherence<pcl::PointXYZRGB>());
    coherence->addPointCoherence(distance_coherence);

    /*
     * Color coherance so we must have and XYZRGB cloud coming in.
     * ie a depth_registered cloud from a prime sense.
     */
    boost::shared_ptr<HSVColorCoherence<pcl::PointXYZRGB> > color_coherence(new HSVColorCoherence<pcl::PointXYZRGB>());
    color_coherence->setWeight(0.1);
    coherence->addPointCoherence(color_coherence);

    boost::shared_ptr<pcl::search::Octree<pcl::PointXYZRGB> > search(new pcl::search::Octree<pcl::PointXYZRGB>(0.01));
    coherence->setSearchMethod(search);
    coherence->setMaximumDistance(0.01);
    tracker_->setCloudCoherence(coherence);
  }
};  // TrackerRGB

} // sr_point_cloud
