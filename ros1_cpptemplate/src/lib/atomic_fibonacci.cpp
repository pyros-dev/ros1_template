#include "ros1_cpptemplate/atomic_fibonacci.hpp"

#include <ros/console.h>
#include <iostream>
#include <sstream>
#include <string>

namespace ros1_cpptemplate
{

AtomicFibonacci::AtomicFibonacci(const int& last_number, const int& current_number, const int& max_number,
                                 const std::string& name)
{
  last_number_ = last_number;
  current_number_ = current_number;
  max_number_ = max_number;

  if (name == "")
  {
    // don't show empty [] brackets
    log_prefix_ = "";
  }
  else
  {
    std::stringstream log_prefix_string_stream;
    log_prefix_string_stream << "[" << name << "] ";
    log_prefix_ = log_prefix_string_stream.str();
  }
}

AtomicFibonacci::~AtomicFibonacci()
{
}

int AtomicFibonacci::nextAndLog(const std::string& log_prefix)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int next_number = next_();
  ROS_INFO_STREAM(log_prefix_ << log_prefix << "Next fibonacci_number: " << next_number);
  return next_number;
}

int AtomicFibonacci::next()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return next_();
}

int AtomicFibonacci::nextNext()
{
  std::lock_guard<std::mutex> lock(mutex_);
  next_();
  return next_();
}

int AtomicFibonacci::next_()
{
  int new_number = current_number_ + last_number_;
  last_number_ = current_number_;

  if (new_number > max_number_)
  {
    // wrap around to avoid overflow
    new_number = 1;
    last_number_ = 0;
  }

  current_number_ = new_number;

  return new_number;
}

void AtomicFibonacci::setMax(const int& value)
{
  max_number_ = value;
}

}  // namespace
