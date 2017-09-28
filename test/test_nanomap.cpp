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

TEST(UpdatePoses, VerifyAddingEdges)
{
  int num_point_clouds = 100;
  NanoMap nanomap = NanoMapDefaults();
  NanoMapTime nm_time(0, 0);
  NanoMapPose nm_pose(Vector3(0.0, 0.0, 0.0), Quat(0,0,0,1), nm_time);

  // add poses
  for (int i = 0; i < num_point_clouds; i++) {
	  nm_time.nsec = i;
  	nm_pose.position = Vector3(0.1, 0.0, 0.0);
  	nm_pose.time = nm_time;
  	nanomap.AddPose(nm_pose);
  }

  // add pointclouds
  for (int i = 0; i < num_point_clouds; i++) {
  	nm_time.nsec = i;
  	pcl::PointCloud<pcl::PointXYZ> empty_cloud;
    PointCloudPtr empty_cloud_ptr = empty_cloud.makeShared();
  	nanomap.AddPointCloud(empty_cloud_ptr, nm_time, i);
  }
  std::vector<Matrix4> current_edges = nanomap.GetCurrentEdges();
  ASSERT_EQ(current_edges.size(), num_point_clouds);
}

// Declare another test
// TEST(TestSuite, testCase2)
// {
// //<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  //testing::internal::CaptureStdout();
  std::cout << "My test" << std::endl;	
  return RUN_ALL_TESTS();
}
