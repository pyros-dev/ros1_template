#include "ros1_cpptemplate/atomic_fibonacci.hpp"

#include <iostream>

namespace ros1_cpptemplate
{

AtomicFibonacci::AtomicFibonacci(const int& last_number, const int& current_number, const int& max_number)
{
  last_number_ = last_number;
  current_number_ = current_number;
  max_number_ = max_number;
}

AtomicFibonacci::~AtomicFibonacci()
{
}

int AtomicFibonacci::nextAndPrint()
{
  std::lock_guard<std::mutex> lock(mutex_);
  int next_number = next_();
  std::cout << "[AtomicFibonacci]: Next fibonacci_number: " << next_number << std::endl;
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

}  // namespace
