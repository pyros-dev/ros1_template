#ifndef ros1_ros_cpp_template_PUBLISHER_HPP_
#define ros1_ros_cpp_template_PUBLISHER_HPP_

#include <ros1_cpptemplate/atomic_fibonacci.hpp>

#include <ros/publisher.h>
#include <ros/node_handle.h>

#include <memory>
#include <string>

namespace ros1_ros_cpptemplate
{

/**
 * Class which publishes fibonacci numbers
 */
class Publisher
{
public:
  /**
   * Constructor
   *
   * @param atomic_fibonacci Shared pointer to AtomicFibonacci object
   * @param node_handle NodeHandle the publisher will be registered under
   * @param output_topic_name Publish topic name
   */
  explicit Publisher(ros1_cpptemplate::AtomicFibonacciPtr atomic_fibonacci,
                     ros::NodeHandle& node_handle,
                     const std::string& output_topic_name);

  /**
   * Deconstructor
   */
  virtual ~Publisher();

  /**
   * Publish the next Fibonacci number
   */
  virtual void publish();

protected:
  /**
   * Protected default construtor for easy mocking
   */
  Publisher()
  {
  }

  /// AtomicFibonacci supplier (shared)
  ros1_cpptemplate::AtomicFibonacciPtr atomic_fibonacci_;

  /// Actual publisher
  ros::Publisher publisher_;
};
typedef std::shared_ptr<Publisher> PublisherPtr;

}  // namespace

#endif
