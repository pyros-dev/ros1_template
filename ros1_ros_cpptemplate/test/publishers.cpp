#include <gtest/gtest.h>

#include <ros/master.h>
#include <std_msgs/Int32.h>
#include <ros/duration.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>

#include <atomic>
#include <iostream>

static double CALLBACK_TEST_TIME_LIMIT_ = 1.0;

class Int32CallbackTest
{
public:
  Int32CallbackTest(const std::string& topic) : got_msg_(false)
  {
    ros::NodeHandle node_handle;
    int queue_size = 1;
    ros::Subscriber subscriber = node_handle.subscribe(topic, queue_size, &Int32CallbackTest::callback, this);

    ros::Time time_out = ros::Time::now() + ros::Duration(CALLBACK_TEST_TIME_LIMIT_);

    while (subscriber.getNumPublishers() == 0 && ros::Time::now() < time_out && ros::ok())
    {
      ros::Duration(0.001).sleep();
    }

    while (!got_msg_.load() && ros::Time::now() < time_out && ros::ok())
    {
      ros::spinOnce();
      ros::Duration(0.001).sleep();
    }
  }

  ~Int32CallbackTest()
  {
    EXPECT_TRUE(got_msg_.load());
  }

  void callback(const std_msgs::Int32Ptr& number)
  {
    got_msg_ = true;
  }

private:
  std::atomic<bool> got_msg_;
};

TEST(Publishers, internal)
{
  Int32CallbackTest("fibonacci_publisher_internal");
}

TEST(Publishers, other)
{
  Int32CallbackTest("fibonacci_publisher_other");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int time_seed = static_cast<int>(time(0));
  srand(time_seed);

  ros::init(argc, argv, "publishers");

  ros::NodeHandle node_handle("~");
  node_handle.getParam("single_test_time_limit", CALLBACK_TEST_TIME_LIMIT_);

  return RUN_ALL_TESTS();
}
