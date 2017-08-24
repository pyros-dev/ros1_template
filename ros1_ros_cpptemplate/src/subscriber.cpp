// own class header first: https://stackoverflow.com/a/2762596/8224054
#include "ros1_ros_cpptemplate/subscriber.hpp"

#include <string>

namespace ros1_ros_cpptemplate
{

Subscriber::Subscriber(ros::NodeHandle& node_handle, const std::string& topic_name)
{
  // Subscribing should be last after everything else of the class is ready
  int queue_size = 10;
  subscriber_ = node_handle.subscribe(topic_name, queue_size, &Subscriber::callback, this);
}

Subscriber::~Subscriber()
{
}

void Subscriber::callback(const std_msgs::Int32Ptr& number)
{
  ROS_INFO_STREAM("Got number " << number->data << " on " << subscriber_.getTopic());
}

}  // namespace
