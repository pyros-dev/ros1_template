
#include "ros1_ros_cpptemplate/publisher.hpp"
#include "ros1_ros_cpptemplate/subscriber.hpp"
#include "ros1_ros_cpptemplate/service.hpp"
#include <ros1_cpptemplate/atomic_fibonacci.hpp>

#include <dynamic_reconfigure/server.h>
#include <ros1_template_msgs/FibonacciConfig.h>

#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>

#include <atomic>
#include <thread>
#include <string>
#include <vector>

namespace ros1_ros_cpptemplate
{

using ros1_cpptemplate::AtomicFibonacci;
using ros1_cpptemplate::AtomicFibonacciPtr;

class Nodelet : public nodelet::Nodelet
{
public:
  Nodelet()
  {
    // initialize simple member variables here instead of the
    // initializer list for cleaner structure
    shutdown_requested_ = false;
  }

  ~Nodelet()
  {
    NODELET_INFO_STREAM("Waiting for publishing threads to finish");
    shutdown_requested_ = true;

    for (int i = 0; i < publish_threads_.size(); ++i)
    {
      publish_threads_[i].join();
    }
  }

  virtual void onInit()
  {
    private_node_handle_ = this->getPrivateNodeHandle();

    // get fixed parameters
    double publish_rate = 10.0;
    private_node_handle_.getParam("publish_rate", publish_rate);
    int fibonacci_last_number = 0;
    private_node_handle_.getParam("start_last_number", fibonacci_last_number);
    int fibonacci_current_number = 1;
    private_node_handle_.getParam("start_current_number", fibonacci_current_number);

    // The max number will be set by the reconfigure callback. When the reconfigure
    // server is constructed it loads the values from the param server (from the files)
    // and triggers a callback.
    int temp_fibonacci_max_number = 256;

    // NODELET logging will put the name of the nodelet instead of
    // the nodelet manager into the log
    NODELET_INFO_STREAM("Initializing with publish rate " << publish_rate << " Hz");

    std::string fibonacci_log_name = "AtomicFibonacci";
    atomic_fibonacci_ = std::make_shared<AtomicFibonacci>(fibonacci_last_number, fibonacci_current_number,
                                                          temp_fibonacci_max_number, fibonacci_log_name);

    reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<ros1_template_msgs::FibonacciConfig>>
        (private_node_handle_);
    dynamic_reconfigure::Server<ros1_template_msgs::FibonacciConfig>::CallbackType reconfigure_cb =
        boost::bind(&Nodelet::reconfigureCB, this, _1, _2);
    reconfigure_server_->setCallback(reconfigure_cb);

    std::string internal_publish_topic_name = "fibonacci_publisher_internal";
    publisher_used_interally_ = std::make_shared<Publisher>(atomic_fibonacci_, private_node_handle_,
                                                            internal_publish_topic_name);
    std::string other_publish_topic_name = "fibonacci_publisher_other";
    publisher_other_ = std::make_shared<Publisher>(atomic_fibonacci_, private_node_handle_, other_publish_topic_name);

    publish_threads_.emplace_back(&Nodelet::run, this, publish_rate, publisher_used_interally_);
    publish_threads_.emplace_back(&Nodelet::run, this, publish_rate, publisher_other_);

    // subsciber and service stuff last, or at least after everything else is ready
    subscriber_ = std::make_shared<Subscriber>(private_node_handle_, internal_publish_topic_name);
    service_ = std::make_shared<Service>(atomic_fibonacci_, private_node_handle_, "fibonacci_service");

    NODELET_INFO_STREAM("Initialized");
  }

private:
  void run(double publish_rate, PublisherPtr publisher)
  {
    ros::Rate rate(publish_rate);
    while (!shutdown_requested_ && ros::ok())
    {
      publisher->publish();
      rate.sleep();
    }
  }

  void reconfigureCB(ros1_template_msgs::FibonacciConfig &config, uint32_t level)
  {
    atomic_fibonacci_->setMax(config.fibonacci_max_number);
    NODELET_INFO_STREAM("Max. Fibonacci number set to " << config.fibonacci_max_number);
  }

  std::atomic<bool> shutdown_requested_;
  ros::NodeHandle private_node_handle_;

  std::shared_ptr<dynamic_reconfigure::Server<ros1_template_msgs::FibonacciConfig>> reconfigure_server_;

  AtomicFibonacciPtr atomic_fibonacci_;
  PublisherPtr publisher_used_interally_;
  PublisherPtr publisher_other_;
  std::vector<std::thread> publish_threads_;

  SubscriberPtr subscriber_;
  ServicePtr service_;
};

}  // namespace

PLUGINLIB_EXPORT_CLASS(ros1_ros_cpptemplate::Nodelet, nodelet::Nodelet);
