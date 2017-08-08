#include <gtest/gtest.h>

#include <ros1_cpptemplate/atomic_fibonacci.hpp>

namespace ros1_cpptemplate
{

TEST(AtomicFibonacci, next)
{
  int last = 0;
  int current = 1;
  int max = 17;
  AtomicFibonacci atomic_fibonacci(last, current, max);

  EXPECT_EQ(1, atomic_fibonacci.next());
  EXPECT_EQ(2, atomic_fibonacci.next());
  EXPECT_EQ(3, atomic_fibonacci.next());
  EXPECT_EQ(5, atomic_fibonacci.next());
  EXPECT_EQ(8, atomic_fibonacci.next());
  EXPECT_EQ(13, atomic_fibonacci.next());

  // max is 17, so overflow
  EXPECT_EQ(1, atomic_fibonacci.next());
  EXPECT_EQ(1, atomic_fibonacci.next());
  EXPECT_EQ(2, atomic_fibonacci.next());
  EXPECT_EQ(3, atomic_fibonacci.next());
  EXPECT_EQ(5, atomic_fibonacci.next());
}

TEST(AtomicFibonacci, nextNext)
{
  int last = 0;
  int current = 1;
  int max = 10;
  AtomicFibonacci atomic_fibonacci_next(last, current, max);
  AtomicFibonacci atomic_fibonacci_nextNext(last, current, max);

  int next = 0;
  int nextNext = 0;

  next = atomic_fibonacci_next.next();
  next = atomic_fibonacci_next.next();
  nextNext = atomic_fibonacci_nextNext.nextNext();

  EXPECT_EQ(2, nextNext);
  EXPECT_EQ(next, nextNext);

  next = atomic_fibonacci_next.next();
  next = atomic_fibonacci_next.next();
  nextNext = atomic_fibonacci_nextNext.nextNext();

  EXPECT_EQ(5, nextNext);
  EXPECT_EQ(next, nextNext);

  // max is 10, so overflow
  next = atomic_fibonacci_next.next();
  next = atomic_fibonacci_next.next();
  nextNext = atomic_fibonacci_nextNext.nextNext();

  EXPECT_EQ(1, nextNext);
  EXPECT_EQ(next, nextNext);

  next = atomic_fibonacci_next.next();
  next = atomic_fibonacci_next.next();
  nextNext = atomic_fibonacci_nextNext.nextNext();

  EXPECT_EQ(2, nextNext);
  EXPECT_EQ(next, nextNext);

  next = atomic_fibonacci_next.next();
  next = atomic_fibonacci_next.next();
  nextNext = atomic_fibonacci_nextNext.nextNext();

  EXPECT_EQ(5, nextNext);
  EXPECT_EQ(next, nextNext);
}

TEST(AtomicFibonacci, construtorParams)
{
  int last = 12;
  int current = 17;
  int max = 33;
  AtomicFibonacci default_constructed(last, current, max);

  // 12, 17, 29, overflow 1, 1, 2, 3

  EXPECT_EQ(29, default_constructed.next());
  EXPECT_EQ(1, default_constructed.nextNext());
  EXPECT_EQ(2, default_constructed.next());

  AtomicFibonacciPtr shared_pointer = std::make_shared<AtomicFibonacci>(2, 3, max);
  EXPECT_EQ(5, shared_pointer->next());
}

}  // namespace
