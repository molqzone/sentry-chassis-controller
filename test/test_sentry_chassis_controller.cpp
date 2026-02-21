#include <cstdlib>
#include <ctime>

#include <gtest/gtest.h>

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  return RUN_ALL_TESTS();
}
