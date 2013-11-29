#include <sr_grasp_msgs/TriangulateAction.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <gtest/gtest.h>

//-------------------------------------------------------------------------------

// Called once when the goal completes
void done_cb(const actionlib::SimpleClientGoalState& state,
             const sr_grasp_msgs::TriangulateResultConstPtr& result)
{
  /*
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  const std::vector<moveit_msgs::Grasp> &grasps = result->grasps;
  EXPECT_GT(grasps.size(), 0);
  for (size_t i = 0; i < grasps.size(); i++)
  {
    ROS_INFO_STREAM("id: " << grasps[i].id);
    ROS_INFO_STREAM("grasp_quality: " << grasps[i].grasp_quality);
    EXPECT_GT(grasps[i].grasp_quality, 0);
  }
  ros::shutdown();
  */
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

  /*
  ReadPLY reader;
  if (reader.load(ply_filename.c_str()) != 0)
    ROS_ERROR_STREAM("Failed to load " << ply_filename << " using method ReadPLY::Load.");
  ROS_INFO_STREAM("Number of vertices read  = " << reader.total_vertices_  << ".");
  ROS_INFO_STREAM("Number of triangles read = " << reader.total_triangles_ << ".");

  sr_grasp_msgs::TriangulateGoal goal;

  // Set the list of triangles.
  for (int i = 0; i < reader.total_triangles_; i++)
  {
    shape_msgs::MeshTriangle triangle;
    const ReadPLY::PlyTriangle &curr_tri = reader.triangles_[i];
    triangle.vertex_indices[0] = curr_tri.n1;
    triangle.vertex_indices[1] = curr_tri.n2;
    triangle.vertex_indices[2] = curr_tri.n3;
    goal.object.bounding_mesh.triangles.push_back(triangle);
  }

  // Set the actual vertices that make up the mesh.
  for (int i = 0; i < reader.total_vertices_; i++)
  {
    geometry_msgs::Point vertex;
    const ReadPLY::PlyVertex &curr_ver = reader.vertices_[i];
    vertex.x = curr_ver.x;
    vertex.y = curr_ver.y;
    vertex.z = curr_ver.z;
    goal.object.bounding_mesh.vertices.push_back(vertex);
  }

  // Create the action client, and true causes the client to spin its own thread.
  actionlib::SimpleActionClient<sr_grasp_msgs::TriangulateAction> ac("sr_grasp_mesh_planner", true);

  ROS_INFO_STREAM("Waiting for action server to start.");
  ac.waitForServer();

  // http://goo.gl/BcuAFa
  ROS_INFO_STREAM("Action server started, sending goal.");
  ac.sendGoal(goal, &done_cb, &active_cb, &feedback_cb);

  ros::spin();
  */
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

