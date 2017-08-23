
#include "ros1_ros_cpptemplate/publisher.hpp"
#include "ros1_ros_cpptemplate/subscriber.hpp"
#include <ros1_cpptemplate/atomic_fibonacci.hpp>

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
    int fibonacci_max_number = 256;
    private_node_handle_.getParam("fibonacci_max_number", fibonacci_max_number);

    // NODELET logging will put the name of the nodelet instead of
    // the nodelet manager into the log
    NODELET_INFO_STREAM("Initializing with publish rate " << publish_rate << " Hz");

    std::string fibonacci_log_name = "AtomicFibonacci";
    AtomicFibonacciPtr atomic_fibonacci = std::make_shared<AtomicFibonacci>(fibonacci_last_number,
                                                                            fibonacci_current_number,
                                                                            fibonacci_max_number, fibonacci_log_name);

    std::string internal_publish_topic_name = "fibonacci_publisher_internal";
    publisher_used_interally_ = std::make_shared<Publisher>(atomic_fibonacci, private_node_handle_,
                                                            internal_publish_topic_name);
    std::string other_publish_topic_name = "fibonacci_publisher_other";
    publisher_other_ = std::make_shared<Publisher>(atomic_fibonacci, private_node_handle_, other_publish_topic_name);

    publish_threads_.emplace_back(&Nodelet::run, this, publish_rate, publisher_used_interally_);
    publish_threads_.emplace_back(&Nodelet::run, this, publish_rate, publisher_other_);

    // subsciber stuff last, or at least after everything else is ready
    subscriber_ = std::make_shared<Subscriber>(private_node_handle_, internal_publish_topic_name);

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

  std::atomic<bool> shutdown_requested_;
  ros::NodeHandle private_node_handle_;

  AtomicFibonacciPtr atomic_fibonacci_;
  PublisherPtr publisher_used_interally_;
  PublisherPtr publisher_other_;
  std::vector<std::thread> publish_threads_;

  SubscriberPtr subscriber_;
};

}  // namespace

PLUGINLIB_EXPORT_CLASS(ros1_ros_cpptemplate::Nodelet, nodelet::Nodelet);
