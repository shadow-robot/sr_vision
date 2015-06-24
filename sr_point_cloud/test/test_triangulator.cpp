/* Copyright 2015 ShadowRobot */

#include <string>
#include <sr_grasp_msgs/TriangulateAction.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <gtest/gtest.h>

//-------------------------------------------------------------------------------

// Called once when the goal completes
void done_cb(const actionlib::SimpleClientGoalState& state,
             const sr_grasp_msgs::TriangulateResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());

  const pcl_msgs::PolygonMesh &pcl_mesh = result->pcl_mesh;
  const shape_msgs::Mesh &shape_mesh =result-> shape_mesh;

  EXPECT_GT(pcl_mesh.polygons.size(), 0);
  EXPECT_GT(shape_mesh.triangles.size(), 0);
  EXPECT_GT(shape_mesh.vertices.size(), 0);

  ros::shutdown();
}

//-------------------------------------------------------------------------------

// Called once when the goal becomes active
void active_cb()
{
  ROS_INFO("Goal just went active");
  ASSERT_TRUE(true);
}

//-------------------------------------------------------------------------------

// Called every time feedback is received for the goal
void feedback_cb(const sr_grasp_msgs::TriangulateFeedbackConstPtr& feedback)
{
  // No feedback.
}

//-------------------------------------------------------------------------------

TEST(TestTriangulator, testTriangulator)
{
  std::string package_path = ros::package::getPath("sr_point_cloud");
  std::string pcd_filename = package_path + "/test/bun0.pcd";

  // Load input file into a pcl::PCLPointCloud2.
  pcl::PCLPointCloud2 pcl_pc2;
  int res = pcl::io::loadPCDFile(pcd_filename, pcl_pc2);
  ASSERT_EQ(res, 0); // PCD file loaded?

  // Convert to sensor_msgs::PointCloud2.
  sensor_msgs::PointCloud2 sensor_pc2;
  pcl_conversions::fromPCL(pcl_pc2, sensor_pc2);

  // Set the action goal.
  sr_grasp_msgs::TriangulateGoal goal;
  goal.point_cloud = sensor_pc2;

  // Create the action client, and true causes the client to spin its own thread.
  // true -> don't need ros::spin()
  std::string action_name("triangulator/triangulate");
  actionlib::SimpleActionClient<sr_grasp_msgs::TriangulateAction> ac(action_name, true);

  ROS_INFO_STREAM("Waiting for action server to start.");
  ac.waitForServer();

  // http://goo.gl/BcuAFa
  ROS_INFO_STREAM("Action server started, sending goal.");
  ac.sendGoal(goal, &done_cb, &active_cb, &feedback_cb);
}

//-------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_triangulator");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

//-------------------------------------------------------------------------------

