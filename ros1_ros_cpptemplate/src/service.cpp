// own class header first: https://stackoverflow.com/a/2762596/8224054
#include "ros1_ros_cpptemplate/service.hpp"

#include <std_msgs/Int32.h>
#include <sstream>
#include <string>

namespace ros1_ros_cpptemplate
{
using ros1_cpptemplate::AtomicFibonacci;
using ros1_cpptemplate::AtomicFibonacciPtr;

Service::Service(ros1_cpptemplate::AtomicFibonacciPtr atomic_fibonacci,
                 ros::NodeHandle& node_handle,
                 const std::string& service_topic_name)
{
  int queue_size = 1;
  bool latched = false;
  service_ = node_handle.advertiseService("fibonacci_service", &Service::serve,
                                          this);
  atomic_fibonacci_ = atomic_fibonacci;
}

Service::~Service()
{
}

bool Service::serve(ros1_template_msgs::Answer::Request &request,
                    ros1_template_msgs::Answer::Response &response)
{
  ROS_INFO_STREAM("Service called with question: " << request.question);

  std::stringstream log_prefix_string_stream;
  log_prefix_string_stream << "Serving on " << service_.getService() << ": ";
  std::string log_prefix = log_prefix_string_stream.str();

  int next_number = atomic_fibonacci_->nextAndLog(log_prefix);
  response.answer = next_number;

  return true;
}

}  // namespace
