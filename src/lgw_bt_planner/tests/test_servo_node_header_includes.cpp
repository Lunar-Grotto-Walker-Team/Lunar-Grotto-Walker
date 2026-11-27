// tests/test_servo_node_header_includes.cpp
//
// Self-containment check for servo_node.hpp (2026-11-12 header split).
//
// The test is the translation unit compiling: nothing but gtest and the header
// itself is included. Now that the implementation lives in .cpp files, an
// include that only the implementation needed would sit in the header and be
// papered over by whichever .cpp happened to include it first -- this TU is
// what fails when that happens.

#include <gtest/gtest.h>

#include "lgw_bt_planner/servo/servo_node.hpp"

TEST(ServoNodeHeader, IsSelfContained)
{
  // A class with an out-of-line destructor cannot be instantiated without
  // its definition; merely naming the type proves the declaration is complete.
  EXPECT_FALSE(std::is_abstract<lgw_bt_planner::ServoNode>::value);
}
