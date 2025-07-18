#include "motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cmath>

#define PCA9685_ADDRESS 0x60
#define MODE1 0x00
#define MODE2  0x01
#define SUBADR1 0x02
#define SUBADR2 0x03
#define SUBADR3 0x04
#define PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define ALL_LED_ON_L 0xFA
#define ALL_LED_ON_H 0xFB
#define ALL_LED_OFF_L 0xFC
#define ALL_LED_OFF_H  0xFD

// Bits
#define RESTART 0x80
#define SLEEP 0x10
#define ALLCALL 0x01
#define INVRT 0x10
#define OUTDRV 0x04



Motor::Motor(rclcpp::Logger logger, int motor_num, double alpha, double beta)
    : logger_(logger), motor_num_(motor_num), alpha_(alpha), beta_(beta), value_(0.0), i2c_fd_(-1) {
    RCLCPP_INFO(logger_, "Initializing motor %d with alpha=%.3f, beta=%.3f", 
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
    } catch (const std::runtime_error &e) {
        RCLCPP_ERROR(logger_, "Motor %d initializing error: %s", motor_num_, e.what());
    }
    RCLCPP_INFO(logger_, "Motor %d initialized succesfully", motor_num_);

}

Motor::~Motor() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
    }
}

void Motor::setValue(double value) {
    try {
        int mapped_value = static_cast<int>(255.0 * (alpha_ * value + beta_));
        int speed = std::min(std::max(std::abs(mapped_value), 0), 255);

        if (speed < 0) {
            speed = 0;
        } else if (speed > 255) {
            speed = 255;
        }

        setState(mapped_value, speed);
        RCLCPP_INFO(logger_, "Motor %d value setting, value = %f", motor_num_, value);
    }
    catch (...) {
        RCLCPP_INFO(logger_, "Motor %d value setting failed", motor_num_);
    }

}
void Motor::setState(int value, int speed) {

    setPWM(pwm_pin, 0, speed*16);
    if (value > 0) {
        setPin(fwd_pin, 1);
        setPin(rev_pin, 0);
        setPWM(fwd_ch, 0, speed*16);
        setPWM(rev_ch, 0, 0);
    } else if (value < 0) {
        setPin(fwd_pin, 0);
        setPin(rev_pin, 1);
        setPWM(rev_ch, 0, speed*16);
        setPWM(fwd_ch, 0, 0);
    } else {
        setPin(fwd_pin, 0);
        setPin(rev_pin, 0);
    }
    
}

void Motor::stop() {
    setPin(fwd_pin, 0);
    setPin(rev_pin, 0);
}

bool Motor::openI2C() {
    const char* device = "/dev/i2c-7";
    i2c_fd_ = open(device, O_RDWR);
    if (i2c_fd_ < 0) {
        std::cerr << "Failed to open I2C device" << std::endl;
        RCLCPP_INFO(logger_, "Failed to open I2C device, while running motor %d", motor_num_);
        return false;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, PCA9685_ADDRESS) < 0) {
        std::cerr << "Failed to set I2C address" << std::endl;
        RCLCPP_INFO(logger_, "Failed to set I2C address, while running motor %d", motor_num_);
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    RCLCPP_INFO(logger_, "I2C successfull, while running motor %d", motor_num_);
    return true;
}

bool Motor::writeRegister(uint8_t reg, uint8_t value) {
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

bool Motor::readRegister(uint8_t reg, uint8_t& value) {
    if (write(i2c_fd_, &reg, 1) != 1) {
        return false;
    }

    if (read(i2c_fd_, &value, 1) != 1) {
        return false;
    }

    return true;
}

void Motor::resetPCA9685() {
    // Reset sequence
    writeRegister(MODE1, 0x00);  // Clear all bits first
    usleep(1000);
    
    // Wake up (clear sleep bit) and enable auto-increment
    writeRegister(MODE1, ALLCALL);
    usleep(1000);
    
    // Read current mode
    uint8_t mode1;
    readRegister(MODE1, mode1);
    
    // Clear sleep bit (wake up)
    mode1 = mode1 & ~SLEEP;  // Clear sleep bit
    writeRegister(MODE1, mode1);
    usleep(1000);
    
    // Set output driver to totem pole
    writeRegister(MODE2, OUTDRV);
    usleep(1000);
    
    RCLCPP_INFO(logger_, "PCA9685 reset completed");
}

void Motor::setPWMFreq(int freq) {
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

void Motor::setPWM(int channel, int on, int off) {
    uint8_t reg = LED0_ON_L + 4 * channel;
    writeRegister(reg, on & 0xFF);
    writeRegister(reg + 1, on >> 8);
    writeRegister(reg + 2, off & 0xFF);
    writeRegister(reg + 3, off >> 8);
}

void Motor::setPin(int pin, int value) {
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
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to set PWM on pin %d: %s", pin, e.what());
    }
}
