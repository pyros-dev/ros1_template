#include <gtest/gtest.h>

#include <std_msgs/Int32.h>
#include <ros/duration.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>

#include <atomic>
#include <iostream>

TEST(Subscriber, internal)
{
  ros::NodeHandle node_handle;
  int queue_size = 1;
  ros::Publisher publisher_to_interal_topic = node_handle.advertise<std_msgs::Int32>("fibonacci_publisher_internal", queue_size);

  while (publisher_to_interal_topic.getNumSubscribers() == 0 && ros::ok())
  {
    ros::Duration(0.001).sleep();
  }

  EXPECT_EQ(publisher_to_interal_topic.getNumSubscribers(), 1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int time_seed = static_cast<int>(time(0));
  srand(time_seed);

  ros::init(argc, argv, "subscribers");

  return RUN_ALL_TESTS();
}
