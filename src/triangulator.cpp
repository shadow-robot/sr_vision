#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <dynamic_reconfigure/server.h>
#include "sr_point_cloud/TriangulatorConfig.h"

// ROS pcl includes
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h" // Allow use of PCL cloud types for pubs and subs
#include "pcl_msgs/PolygonMesh.h"

// PCL specific includes
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

namespace sr_point_cloud {

class Triangulator {

protected:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> Cloud;

    ros::NodeHandle nh_;

    dynamic_reconfigure::Server<TriangulatorConfig> config_server_;
    double mu_;
    int maximum_nearest_neighbors_;
    double maximum_surface_angle_;
    double minimum_angle_;
    double maximum_angle_;

    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;
    Cloud::ConstPtr input_;

public:
    Triangulator()
      : nh_("~")
      , mu_(2.5)
      , maximum_nearest_neighbors_(100)
      , maximum_surface_angle_(M_PI/4) // 45 degs
      , minimum_angle_(M_PI/18) // 10 degs
      , maximum_angle_(2*M_PI/3) // 120 degs
    {
        // Setup ROS topics and services
        config_server_.setCallback( boost::bind(&Triangulator::config_cb, this, _1, _2) );
        input_sub_ = nh_.subscribe("input/points", 1, &Triangulator::cloud_cb, this);
        output_pub_ = nh_.advertise<pcl_msgs::PolygonMesh>("output/mesh", 1);
    }

    void run () { ros::spin(); }

protected:

    void
    config_cb(TriangulatorConfig &config, uint32_t level)
    {
        mu_ = config.mu;
        maximum_nearest_neighbors_ = config.maximum_nearest_neighbors;
        maximum_surface_angle_ = config.maximum_surface_angle;
        minimum_angle_ = config.minimum_angle;
        maximum_angle_ = config.maximum_angle;
    }

    /*
     * Runs the triangulation. Based on this:
     * http://www.pointclouds.org/documentation/tutorials/greedy_projection.php#greedy-triangulation
     */
    void
    cloud_cb (const Cloud::ConstPtr& cloud)
    {
        input_ = cloud;

        // Normal estimation*
        pcl::NormalEstimation<PointType, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (cloud);
        n.setInputCloud (cloud);
        n.setSearchMethod (tree);
        n.setKSearch (20);
        n.compute (*normals);
        //* normals should contain the point normals + surface curvatures

        // Concatenate the XYZ and normal fields*
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
        //* cloud_with_normals = cloud + normals

        // Create search tree*
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud (cloud_with_normals);

        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius (0.025);

        // Set typical values for the parameters
        gp3.setMu (mu_);
        gp3.setMaximumNearestNeighbors (maximum_nearest_neighbors_);
        gp3.setMaximumSurfaceAngle(maximum_surface_angle_);
        gp3.setMinimumAngle(minimum_angle_);
        gp3.setMaximumAngle(maximum_angle_);
        gp3.setNormalConsistency(false);

        // Get result
        gp3.setInputCloud (cloud_with_normals);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (triangles);

        // Additional vertex information
        std::vector<int> parts = gp3.getPartIDs();
        std::vector<int> states = gp3.getPointStates();

        // Convert to ROS type and publish
        pcl_msgs::PolygonMesh mesh;
        pcl_conversions::fromPCL(triangles, mesh);
        output_pub_.publish(mesh);

        // Debug
        //pcl::io::saveVTKFile("mesh.vtk", triangles);
    }

}; // Triangulator

} // sr_point_cloud::

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "point_cloud_triangulator");
  sr_point_cloud::Triangulator node;
  node.run();
  return 0;
}
