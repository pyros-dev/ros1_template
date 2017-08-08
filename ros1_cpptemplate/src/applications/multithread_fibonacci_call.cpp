#include "ros1_cpptemplate/atomic_fibonacci.hpp"

#include <sstream>
#include <iostream>

#include <atomic>
#include <vector>
#include <thread>
#include <chrono>

using ros1_cpptemplate::AtomicFibonacci;
using ros1_cpptemplate::AtomicFibonacciPtr;

typedef std::shared_ptr<std::atomic<bool>> FlagPtr;

int argToInt(char* arg)
{
  std::stringstream string_stream;
  string_stream << arg;

  int value;
  string_stream >> value;

//   std::cout << "Converted " << arg << " to " << value << std::endl;
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
      std::cout << "Wrong number of arguments! Max. 3, got " << argc << std::endl;
      return 1;
  }

  std::cout << "Initializing AtomicFibonacci with: " << std::endl;
  std::cout << "  last_number    = " << last_number << std::endl;
  std::cout << "  current_number = " << current_number << std::endl;
  std::cout << "  max_number     = " << max_number << std::endl;

  AtomicFibonacciPtr atomic_fibonacci = std::make_shared<AtomicFibonacci>(last_number,
                                                                          current_number,
                                                                          max_number);

  std::cout << "Initialized AtomicFibonacci object" << std::endl;

  std::atomic<bool> start_flag;
  start_flag = false;

  int number_of_threads = 20;
  std::vector<std::thread> threads;

  std::cout << "Start spawning " << number_of_threads << " threads" << std::endl;

  for (int i = 0; i < number_of_threads; ++i)
  {
    threads.push_back(std::thread(&callFibonacci, atomic_fibonacci, std::ref(start_flag)));
  }

  std::cout << "Spawning finished, let's go!" << std::endl;

  start_flag = true;

  for (int i = 0; i < threads.size(); ++i)
  {
    threads[i].join();
  }

  std::cout << "Finished. Goodbye!" << std::endl;
}
