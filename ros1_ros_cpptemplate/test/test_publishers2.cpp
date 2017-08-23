#include <gtest/gtest.h>

#include <std_msgs/Int32.h>
#include <ros/duration.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>

void callback(const std_msgs::Int32Ptr& number)
{

}

TEST(Publishers2, internal)
{
//   char** argv;
//   ros::init(0, argv, "node_name");
//   ros::NodeHandle node_handle;
//   int queue_size = 10;
//   ros::Subscriber subscriber = node_handle.subscribe("topic", queue_size, &callback);

//   ros::Duration(10.0).sleep();

  EXPECT_EQ(true, false);
}

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "node_name");
//   ros::spin();
// }

// Run with "catkin_make run_tests_ros1_cpptemplate"
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int time_seed = static_cast<int>(time(0));
  srand(time_seed);
  return RUN_ALL_TESTS();
}
