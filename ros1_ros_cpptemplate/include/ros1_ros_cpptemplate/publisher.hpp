#ifndef ros1_ros_cpp_template_PUBLISHER_HPP_
#define ros1_ros_cpp_template_PUBLISHER_HPP_

#include <ros1_cpptemplate/atomic_fibonacci.hpp>

#include <ros/publisher.h>
#include <ros/node_handle.h>

#include <memory>

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
   * @param atomic_fibonacci shared pointer to AtomicFibonacci object
   */
  Publisher(ros1_cpptemplate::AtomicFibonacciPtr atomic_fibonacci, ros::NodeHandle& node_handle, const std::string& output_topic_name);

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

  ros1_cpptemplate::AtomicFibonacciPtr atomic_fibonacci_;
  ros::Publisher publisher_;
};
typedef std::shared_ptr<Publisher> PublisherPtr;

}  // namespace

#endif
