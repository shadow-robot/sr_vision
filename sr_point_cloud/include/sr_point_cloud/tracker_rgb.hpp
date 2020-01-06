/*
* Copyright 2015 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "sr_point_cloud/tracker.hpp"

namespace sr_point_cloud
{

using pcl::tracking::HSVColorCoherence;

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
    typename ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGB>::Ptr
    coherence(new ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGB>());

    boost::shared_ptr<DistanceCoherence<pcl::PointXYZRGB> >
    distance_coherence(new DistanceCoherence<pcl::PointXYZRGB>());
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

}  // namespace sr_point_cloud
