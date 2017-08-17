#include "ros1_cpptemplate/atomic_fibonacci.hpp"

#include <sstream>
#include <iostream>

#include <atomic>
#include <vector>
#include <thread>
#include <chrono>

#include <ros/console.h>

using ros1_cpptemplate::AtomicFibonacci;
using ros1_cpptemplate::AtomicFibonacciPtr;

typedef std::shared_ptr<std::atomic<bool>> FlagPtr;

int argToInt(char* arg)
{
  std::stringstream string_stream;
  string_stream << arg;

  int value;
  string_stream >> value;

//   ROS_INFO_STREAM("Converted " << arg << " to " << value);
  return value;
}

void callFibonacci(AtomicFibonacciPtr atomic_fibonacci, std::atomic<bool>& start_flag)
{
  while (!start_flag.load())
  {
    std::chrono::duration<double> sleep_duration(0.001);
    std::this_thread::sleep_for(sleep_duration);
  }

  atomic_fibonacci->nextAndPrint();
}

int main(int argc, char **argv)
{
  int last_number = 0;
  int current_number = 1;
  int max_number = 256;

  switch (argc)
  {
    case 1:
      break;
    case 2:
      max_number = argToInt(argv[1]);
      break;
    case 3:
      last_number = argToInt(argv[1]);
      current_number = argToInt(argv[2]);
      break;
    case 4:
      last_number = argToInt(argv[1]);
      current_number = argToInt(argv[2]);
      max_number = argToInt(argv[3]);
      break;
    default:
      ROS_INFO_STREAM("Wrong number of arguments! Max. 3, got " << argc);
      return 1;
  }

  ROS_INFO("Initializing AtomicFibonacci with: ");
  ROS_INFO_STREAM("  last_number    = " << last_number);
  ROS_INFO_STREAM("  current_number = " << current_number);
  ROS_INFO_STREAM("  max_number     = " << max_number);

  AtomicFibonacciPtr atomic_fibonacci = std::make_shared<AtomicFibonacci>(last_number,
                                                                          current_number,
                                                                          max_number);

  ROS_INFO_STREAM("Initialized AtomicFibonacci object");

  std::atomic<bool> start_flag;
  start_flag = false;

  int number_of_threads = 20;
  std::vector<std::thread> threads;

  ROS_INFO_STREAM("Start spawning " << number_of_threads << " threads");

  for (int i = 0; i < number_of_threads; ++i)
  {
    threads.push_back(std::thread(&callFibonacci, atomic_fibonacci, std::ref(start_flag)));
  }

  ROS_INFO_STREAM("Spawning finished, let's go!");

  start_flag = true;

  for (int i = 0; i < threads.size(); ++i)
  {
    threads[i].join();
  }

  ROS_INFO_STREAM("Finished. Goodbye!");
}
