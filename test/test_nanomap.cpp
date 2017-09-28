// Bring in my package's API, which is what I'm testing
//#include "foo/foo.h"
// Bring in gtest
#include <gtest/gtest.h>
#include "nanomap.h"

NanoMap NanoMapDefaults() {
  NanoMap nanomap;
  Matrix3 K;
  K << 205.27, 0.0, 160.0, 0.0, 205.27, 120.0, 0.0, 0.0, 1.0;
  nanomap.SetCameraInfo(4.0, 320.0, 240.0, K); // this K matrix was for binned, but the actual data is not binned
  nanomap.SetSensorRange(20.0);
  nanomap.SetNumDepthImageHistory(150);
  Matrix3 body_to_rdf;
  body_to_rdf << 0, -1, 0, 0, 0, -1, 1, 0, 0;
  nanomap.SetBodyToRdf(body_to_rdf);
  return nanomap;
}

void AddZeroPoses(NanoMap& nanomap, int num_poses) {
  NanoMapTime nm_time(0, 0);
  NanoMapPose nm_pose(Vector3(0.0, 0.0, 0.0), Quat(0,0,0,1), nm_time);
  for (int i = 0; i < num_poses; i++) {
    nm_time.nsec = i;
    nm_pose.position = Vector3(0.0, 0.0, 0.0);
    nm_pose.time = nm_time;
    nanomap.AddPose(nm_pose);
  }
}

void AddMovingPoses(NanoMap& nanomap, int num_poses) {
  NanoMapTime nm_time(0, 0);
  NanoMapPose nm_pose(Vector3(0.0, 0.0, 0.0), Quat(0,0,0,1), nm_time);
  for (int i = 0; i < num_poses; i++) {
    nm_pose.time.nsec = i;
    nm_pose.position = Vector3(0.1*i, 0.0, 0.0);
    nanomap.AddPose(nm_pose);
  }
}

void AddEmptyPointClouds(NanoMap& nanomap, int num_point_clouds) {
  NanoMapTime nm_time(0, 0);
  for (int i = 0; i < num_point_clouds; i++) {
    nm_time.nsec = i;
    pcl::PointCloud<pcl::PointXYZ> empty_cloud;
    PointCloudPtr empty_cloud_ptr = empty_cloud.makeShared();
    nanomap.AddPointCloud(empty_cloud_ptr, nm_time, i);
  }
}

void AddUpdatePoses(NanoMap& nanomap, int num_poses) {
  std::vector<NanoMapPose> smoothed_poses;
  NanoMapTime nm_time(0, 0);
  NanoMapPose nm_pose(Vector3(0.0, 0.0, 0.0), Quat(0,0,0,1), nm_time);
  for (size_t i = 0; i < num_poses; i++) {
    nm_pose.time.nsec = i;
    Scalar x_pos = 10.0 + 0.01*i;
    nm_pose.position = Vector3(x_pos, 0.0, 0.0);
    smoothed_poses.push_back(nm_pose);
  }
  nanomap.AddPoseUpdates(smoothed_poses);
}

TEST(AddNewData, VerifyAddingEdges)
{
  int num_point_clouds = 100;
  NanoMap nanomap = NanoMapDefaults();
  AddZeroPoses(nanomap, num_point_clouds);
  AddEmptyPointClouds(nanomap, num_point_clouds);
  std::vector<Matrix4> current_edges = nanomap.GetCurrentEdges();
  ASSERT_EQ(current_edges.size(), num_point_clouds);
}

void NoJumps(std::vector<Matrix4> edges) {
  double max_expected = 1.0;
  int edges_size = edges.size();
  for (int i = 0; i < edges_size; i++) {
    Vector3 translation = edges.at(i).block<3,1>(0,3);
    ASSERT_TRUE(translation.norm() < max_expected);
  }
}


TEST(UpdatePoses, VerifyNoJump)
{
  int num_point_clouds = 10;
  NanoMap nanomap = NanoMapDefaults();
  AddMovingPoses(nanomap, num_point_clouds);
  AddEmptyPointClouds(nanomap, num_point_clouds);
  std::vector<Matrix4> current_edges = nanomap.GetCurrentEdges();
  NoJumps(current_edges);
  AddUpdatePoses(nanomap, 5);
  std::vector<Matrix4> updated_edges = nanomap.GetCurrentEdges();
  NoJumps(updated_edges);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  std::cout << "Running NanoMap tests" << std::endl;	
  return RUN_ALL_TESTS();
}
