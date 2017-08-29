#ifndef ros1_ros_cpp_template_SERVICE_HPP_
#define ros1_ros_cpp_template_SERVICE_HPP_

#include <ros1_cpptemplate/atomic_fibonacci.hpp>
#include <ros1_template_msgs/Answer.h>

#include <ros/service.h>
#include <ros/node_handle.h>

#include <memory>
#include <string>

namespace ros1_ros_cpptemplate
{

/**
 * Class which serves fibonacci numbers
 */
class Service
{
public:
  /**
   * Constructor
   *
   * @param atomic_fibonacci Shared pointer to AtomicFibonacci object
   * @param node_handle NodeHandle the service will be registered under
   * @param service_topic_name Service topic name
   */
  explicit Service(ros1_cpptemplate::AtomicFibonacciPtr atomic_fibonacci,
                     ros::NodeHandle& node_handle,
                     const std::string& service_topic_name);

  /**
   * Deconstructor
   */
  virtual ~Service();

protected:
  /**
   * Protected default construtor for easy mocking
   */
  Service()
  {
  }

  /**
   * Serve the next Fibonacci number
   */
  virtual bool serve(ros1_template_msgs::Answer::Request &request,
                     ros1_template_msgs::Answer::Response &response);

  /// AtomicFibonacci supplier (shared)
  ros1_cpptemplate::AtomicFibonacciPtr atomic_fibonacci_;
  
  /// Actual service
  ros::ServiceServer service_;
};
typedef std::shared_ptr<Service> ServicePtr;

}  // namespace

#endif
