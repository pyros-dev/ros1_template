#include <gtest/gtest.h>

#include <ros1_template_msgs/Answer.h>
#include <std_msgs/Int32.h>
#include <ros/service.h>
#include <ros/service_client.h>
#include <ros/node_handle.h>

#include <atomic>
#include <iostream>
#include <string>

TEST(Service, call)
{
  ros::NodeHandle node_handle;

  std::string service_name = "fibonacci_service";
  std::string full_service_name = node_handle.resolveName(service_name);
  bool service_is_up = ros::service::waitForService(full_service_name);
  EXPECT_TRUE(service_is_up);

  bool persistent_service = true;
  ros::ServiceClient service_client = node_handle.serviceClient<ros1_template_msgs::Answer>(
                                          service_name, persistent_service);

  EXPECT_TRUE(service_client.isValid());

  ros1_template_msgs::Answer service;
  bool call_successful = service_client.call(service);
  EXPECT_TRUE(call_successful);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int time_seed = static_cast<int>(time(0));
  srand(time_seed);

  ros::init(argc, argv, "service");

  return RUN_ALL_TESTS();
}
