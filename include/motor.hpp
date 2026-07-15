#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <cstddef>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"

class Motor
{
public:
  Motor(rclcpp::Logger logger, int motor_num, double alpha = 1.0, double beta = 0.0);
  ~Motor();

  // Owns an I2C file descriptor; copying would double-close it. Held via
  // std::unique_ptr in RobotController, so it is never copied — make that explicit.
  Motor(const Motor &) = delete;
  Motor & operator=(const Motor &) = delete;

  void setValue(double value);
  void stop();

private:
  rclcpp::Logger logger_;
  int motor_num_, pwm_pin, fwd_pin, rev_pin, fwd_ch, rev_ch;
  double alpha_, beta_;
  int i2c_fd_;
  // True once the PCA9685 MODE1 AI (auto-increment, 0x20) bit is verified set;
  // enables 5-byte burst transfers in setPWM. False falls back to per-register writes.
  bool auto_increment_ = false;

  bool openI2C();
  bool writeRegister(uint8_t reg, uint8_t value);
  bool writeRegisters(uint8_t reg, const uint8_t * data, size_t len);
  bool readRegister(uint8_t reg, uint8_t & value);

  void resetPCA9685();
  void setPWMFreq(int freq);
  void setPWM(int channel, int on, int off);
  void setPin(int pin, int value);
  void setState(int value, int speed);
};

#endif
