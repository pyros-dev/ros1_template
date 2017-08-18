#ifndef ros1_ros_cpp_template_SUBSCRIBER_HPP_
#define ros1_ros_cpp_template_SUBSCRIBER_HPP_

#include <std_msgs/Int32.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>

#include <memory>

namespace ros1_ros_cpptemplate
{

/**
 * Class which publishes fibonacci numbers
 */
class Subscriber
{
public:
  /**
   * Constructor
   *
   * @param node_handle node node_handle
   * @param topic_name subscriber topic name
   */
  Subscriber(ros::NodeHandle& node_handle, const std::string& topic_name);

  /**
   * Deconstructor
   */
  virtual ~Subscriber();

protected:
  /**
   * Publish the next Fibonacci number
   */
  virtual void callback(const std_msgs::Int32Ptr& number);

  /**
   * Protected default construtor for easy mocking
   */
  Subscriber()
  {
  }

  ros::Subscriber subscriber_;
};
typedef std::shared_ptr<Subscriber> SubscriberPtr;

}  // namespace

#endif
