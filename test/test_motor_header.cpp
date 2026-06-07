// Compile/contract tests for the Motor hardware-driver header.
//
// The Motor constructor opens an I2C device, so a Motor instance cannot be
// created in a unit-test environment without hardware. These tests therefore
// exercise the header's public contract at compile time via type traits,
// proving that motor.hpp parses, that Motor's declared interface is stable,
// and that it cannot be misused (no default / copy construction). They link
// against the real motor.cpp translation unit through jetank_motor_lib.

#include <type_traits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "motor.hpp"

// Motor owns an I2C file descriptor; it must not be default-constructible
// (the only constructor requires a logger and motor number).
TEST(MotorHeader, NotDefaultConstructible)
{
  EXPECT_FALSE(std::is_default_constructible<Motor>::value);
}

// Motor manages a raw resource (i2c_fd_) and is held via std::unique_ptr in
// RobotController, so accidental copying must be impossible.
TEST(MotorHeader, NotCopyConstructible)
{
  EXPECT_FALSE(std::is_copy_constructible<Motor>::value);
  EXPECT_FALSE(std::is_copy_assignable<Motor>::value);
}

// The documented constructor signature
//   Motor(rclcpp::Logger, int, double = 1.0, double = 0.0)
// must remain callable with both its required and its full argument list.
TEST(MotorHeader, ConstructorSignatureStable)
{
  EXPECT_TRUE((std::is_constructible<Motor, rclcpp::Logger, int>::value));
  EXPECT_TRUE((std::is_constructible<Motor, rclcpp::Logger, int, double>::value));
  EXPECT_TRUE(
    (std::is_constructible<Motor, rclcpp::Logger, int, double, double>::value));
}

// setValue / stop are the public control surface; their member-pointer types
// must keep their signatures (void(double) and void()).
TEST(MotorHeader, PublicControlSurfaceStable)
{
  void (Motor::* set_value)(double) = &Motor::setValue;
  void (Motor::* stop)() = &Motor::stop;
  EXPECT_NE(set_value, nullptr);
  EXPECT_NE(stop, nullptr);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
