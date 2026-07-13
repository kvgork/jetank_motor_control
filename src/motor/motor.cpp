#include "motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#define PCA9685_ADDRESS 0x60
#define MODE1 0x00
#define MODE2  0x01
#define PRESCALE 0xFE
#define LED0_ON_L 0x06

// Bits
#define SLEEP 0x10
#define ALLCALL 0x01
#define AI 0x20      // MODE1 register auto-increment
#define OUTDRV 0x04

Motor::Motor(rclcpp::Logger logger, int motor_num, double alpha, double beta)
: logger_(logger), motor_num_(motor_num), alpha_(alpha), beta_(beta), i2c_fd_(-1)
{
  RCLCPP_INFO(
    logger_, "Initializing motor %d with alpha=%.3f, beta=%.3f",
    motor_num_, alpha_, beta_);

  if (motor_num_ == 0) {
    pwm_pin = 8;
    fwd_pin = 9;
    rev_pin = 10;
    fwd_ch = 0;
    rev_ch = 1;
  } else {
    pwm_pin = 13;
    fwd_pin = 12;
    rev_pin = 11;
    fwd_ch = 3;
    rev_ch = 2;
  }


  try {
    if (!openI2C()) {
      throw std::runtime_error("Failed to open I2C connection");
    }
    resetPCA9685();
    setPWMFreq(1600);
    RCLCPP_INFO(logger_, "Motor %d initialized successfully", motor_num_);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(logger_, "Motor %d initializing error: %s", motor_num_, e.what());
  }
}

Motor::~Motor()
{
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
  }
}

void Motor::setValue(double value)
{
  try {
    int mapped_value = static_cast<int>(255.0 * (alpha_ * value + beta_));
    int speed = std::min(std::max(std::abs(mapped_value), 0), 255);

    setState(mapped_value, speed);
    RCLCPP_DEBUG(logger_, "Motor %d value setting, value = %f", motor_num_, value);
  } catch (...) {
    RCLCPP_WARN(logger_, "Motor %d value setting failed", motor_num_);
  }

}
void Motor::setState(int value, int speed)
{

  setPWM(pwm_pin, 0, speed * 16);
  if (value > 0) {
    setPin(fwd_pin, 1);
    setPin(rev_pin, 0);
    setPWM(fwd_ch, 0, speed * 16);
    setPWM(rev_ch, 0, 0);
  } else if (value < 0) {
    setPin(fwd_pin, 0);
    setPin(rev_pin, 1);
    setPWM(rev_ch, 0, speed * 16);
    setPWM(fwd_ch, 0, 0);
  } else {
    setPin(fwd_pin, 0);
    setPin(rev_pin, 0);
  }

}

void Motor::stop()
{
  setPin(fwd_pin, 0);
  setPin(rev_pin, 0);
}

bool Motor::openI2C()
{
  const char * device = "/dev/i2c-7";
  i2c_fd_ = open(device, O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to open I2C device, while running motor %d", motor_num_);
    return false;
  }

  if (ioctl(i2c_fd_, I2C_SLAVE, PCA9685_ADDRESS) < 0) {
    RCLCPP_ERROR(logger_, "Failed to set I2C address, while running motor %d", motor_num_);
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  RCLCPP_INFO(logger_, "I2C successful, while running motor %d", motor_num_);
  return true;
}

bool Motor::writeRegister(uint8_t reg, uint8_t value)
{
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(logger_, "I2C not initialized");
    return false;
  }

  uint8_t buffer[2] = {reg, value};
  int result = write(i2c_fd_, buffer, 2);
  if (result != 2) {
    RCLCPP_ERROR(logger_, "I2C write failed: reg=0x%02X, value=0x%02X", reg, value);
    return false;
  }
  return true;
}

bool Motor::writeRegisters(uint8_t reg, const uint8_t * data, size_t len)
{
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(logger_, "I2C not initialized");
    return false;
  }

  uint8_t buffer[8];
  if (len + 1 > sizeof(buffer)) {
    RCLCPP_ERROR(logger_, "I2C burst too long: %zu bytes", len);
    return false;
  }
  buffer[0] = reg;
  std::memcpy(buffer + 1, data, len);
  const ssize_t result = write(i2c_fd_, buffer, len + 1);
  if (result != static_cast<ssize_t>(len + 1)) {
    RCLCPP_ERROR(logger_, "I2C burst write failed: reg=0x%02X, len=%zu", reg, len);
    return false;
  }
  return true;
}

bool Motor::readRegister(uint8_t reg, uint8_t & value)
{
  if (write(i2c_fd_, &reg, 1) != 1) {
    return false;
  }

  if (read(i2c_fd_, &value, 1) != 1) {
    return false;
  }

  return true;
}

void Motor::resetPCA9685()
{
  // Reset sequence
  writeRegister(MODE1, 0x00);    // Clear all bits first
  usleep(1000);

  // Wake up and enable register auto-increment (AI, 0x20) so setPWM can burst
  // all four LEDn registers in a single I2C transaction.
  writeRegister(MODE1, ALLCALL | AI);
  usleep(1000);

  // Read current mode
  uint8_t mode1 = 0;
  readRegister(MODE1, mode1);

  // Clear sleep bit (wake up)
  mode1 = mode1 & ~SLEEP;    // Clear sleep bit
  writeRegister(MODE1, mode1);
  usleep(1000);

  // Set output driver to totem pole
  writeRegister(MODE2, OUTDRV);
  usleep(1000);

  // Burst writes are only safe if AI is verifiably enabled; otherwise setPWM
  // keeps the single-register fallback.
  uint8_t verify = 0;
  auto_increment_ = readRegister(MODE1, verify) && (verify & AI) != 0;
  if (!auto_increment_) {
    RCLCPP_WARN(
      logger_, "PCA9685 auto-increment not confirmed; using single-register writes");
  }

  RCLCPP_INFO(logger_, "PCA9685 reset completed");
}

void Motor::setPWMFreq(int freq)
{
  double prescaleval = 25000000.0 / (4096.0 * freq) - 1.0;
  uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5));

  uint8_t oldmode;
  readRegister(MODE1, oldmode);
  uint8_t newmode = (oldmode & 0x7F) | 0x10;

  writeRegister(MODE1, newmode);
  writeRegister(PRESCALE, prescale);
  writeRegister(MODE1, oldmode);
  usleep(500);
  writeRegister(MODE1, oldmode | 0x80);
}

void Motor::setPWM(int channel, int on, int off)
{
  uint8_t reg = LED0_ON_L + 4 * channel;
  if (auto_increment_) {
    // MODE1 AI is enabled: write LEDn_ON_L..LEDn_OFF_H as one 5-byte transfer.
    const uint8_t data[4] = {
      static_cast<uint8_t>(on & 0xFF),
      static_cast<uint8_t>((on >> 8) & 0xFF),
      static_cast<uint8_t>(off & 0xFF),
      static_cast<uint8_t>((off >> 8) & 0xFF)};
    if (writeRegisters(reg, data, sizeof(data))) {
      return;
    }
    // Burst failed; fall through to single-register writes.
  }
  writeRegister(reg, on & 0xFF);
  writeRegister(reg + 1, on >> 8);
  writeRegister(reg + 2, off & 0xFF);
  writeRegister(reg + 3, off >> 8);
}

void Motor::setPin(int pin, int value)
{
  // Validate pin number
  if (pin < 0 || pin > 15) {
    RCLCPP_ERROR(logger_, "Invalid pin number: %d. Pin must be between 0 and 15", pin);
    return;
  }

  // Validate value parameter
  if (value != 0 && value != 1) {
    RCLCPP_ERROR(logger_, "Invalid value: %d. Value must be 0 or 1", value);
    return;
  }

  try {
    if (value == 0) {
      setPWM(pin, 0, 4096);
      RCLCPP_DEBUG(logger_, "Set pin %d to LOW (0)", pin);
    } else if (value == 1) {
      setPWM(pin, 4096, 0);
      RCLCPP_DEBUG(logger_, "Set pin %d to HIGH (1)", pin);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to set PWM on pin %d: %s", pin, e.what());
  }
}
