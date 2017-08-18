#ifndef ros1_cpp_template_ATOMIC_FIBONACCI_HPP_
#define ros1_cpp_template_ATOMIC_FIBONACCI_HPP_

#include <string>
#include <mutex>
#include <memory>

namespace ros1_cpptemplate
{

/**
 * Class which returns the next (or next next) Fibonacci number thread safe
 */
class AtomicFibonacci
{
public:
  /**
   * Constructor
   * If \p max_number is reached the sqeuence is reset to (<b>0, 1</b>)
   * making \c last_number_ \b 0 and \c current_number_ \b 1
   *
   * @param last_number initial last Fibonacci number
   * @param current_number initial current Fibonacci number
   * @param max_number max_number after the sequence gets reset to (0, 1)
   */
  AtomicFibonacci(const int& last_number, const int& current_number,
                  const int& max_number = 256, const std::string& name = "");

  /**
   * Deconstructor
   */
  virtual ~AtomicFibonacci();

    /**
   * Get the next Fibonacci number and log it
   * @return next Fibonacci number
   */
  virtual int nextAndLog(const std::string& log_prefix = "");

  /**
   * Get the next Fibonacci number
   * @return next Fibonacci number
   */
  virtual int next();

  /**
   * Get the Fibonacci number after the next Fibonacci number
   * This advances the sequence of the internal state by two steps
   * (bascially it calls next() twice)
   * @return next next Fibonacci number
   */
  virtual int nextNext();

protected:
  /**
   * Protected default construtor for easy mocking
   */
  AtomicFibonacci()
  {
  }

  /// Mutex for locking
  mutable std::mutex mutex_;

private:
  /**
   * Get the next Fibonacci number without locking
   * @return next Fibonacci number
   */
  int next_();

  /// last Fibonacci number
  int last_number_;

  /// current Fibonacci number
  int current_number_;

  /// max Fibonacci number before reseting the sequence
  int max_number_;

  /// log prefix
  std::string log_prefix_;
};
typedef std::shared_ptr<AtomicFibonacci> AtomicFibonacciPtr;

}  // namespace

#endif
