
#include <gtest/gtest.h>

// Run with "catkin_make run_tests_ros1_cpptemplate"
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int time_seed = static_cast<int>(time(0));
  srand(time_seed);
  return RUN_ALL_TESTS();
}
