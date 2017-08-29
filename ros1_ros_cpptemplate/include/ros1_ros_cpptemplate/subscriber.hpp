#ifndef ros1_ros_cpp_template_SUBSCRIBER_HPP_
#define ros1_ros_cpp_template_SUBSCRIBER_HPP_

#include <std_msgs/Int32.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>

#include <string>
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
   * @param node_handle  NodeHandle the subscriber will be registered under
   * @param topic_name Subscriber topic name
   */
  Subscriber(ros::NodeHandle& node_handle, const std::string& topic_name);

  /**
   * Deconstructor
   */
  virtual ~Subscriber();

protected:
  /**
   * Callback for next Fibonacci number
   * 
   * @param number Msg with next number
   */
  virtual void callback(const std_msgs::Int32Ptr& number);

  /**
   * Protected default construtor for easy mocking
   */
  Subscriber()
  {
  }

  /// Actual subscriber
  ros::Subscriber subscriber_;
};
typedef std::shared_ptr<Subscriber> SubscriberPtr;

}  // namespace

#endif
