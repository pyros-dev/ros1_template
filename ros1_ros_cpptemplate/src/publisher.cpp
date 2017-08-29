// own class header first: https://stackoverflow.com/a/2762596/8224054
#include "ros1_ros_cpptemplate/publisher.hpp"

#include <std_msgs/Int32.h>
#include <sstream>
#include <string>

namespace ros1_ros_cpptemplate
{
using ros1_cpptemplate::AtomicFibonacci;
using ros1_cpptemplate::AtomicFibonacciPtr;

Publisher::Publisher(ros1_cpptemplate::AtomicFibonacciPtr atomic_fibonacci,
                     ros::NodeHandle& node_handle,
                     const std::string& output_topic_name)
{
  int queue_size = 1;
  bool latched = false;
  publisher_ = node_handle.advertise<std_msgs::Int32>(output_topic_name, queue_size, latched);
  atomic_fibonacci_ = atomic_fibonacci;
}

Publisher::~Publisher()
{
}

void Publisher::publish()
{
  if (publisher_.getNumSubscribers() == 0)
  {
    ROS_INFO_STREAM("Nobody subscribed to " << publisher_.getTopic() << ", not publishing");
    return;
  }

  std::stringstream log_prefix_string_stream;
  log_prefix_string_stream << "Publishing on " << publisher_.getTopic() << ": ";
  std::string log_prefix = log_prefix_string_stream.str();

  int next_number = atomic_fibonacci_->nextAndLog(log_prefix);

  // Ptr to enable intraporcess publishing:
  // http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing
  std_msgs::Int32Ptr number_msg(new std_msgs::Int32());
  number_msg->data = next_number;
  publisher_.publish(number_msg);
}

}  // namespace
