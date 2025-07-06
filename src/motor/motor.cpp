#include "motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cmath>

#define PCA9685_ADDRESS 0x40
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
    RCLCPP_INFO(logger_, "Initializing motor %d", motor_num_);
    
    // void int pwm_pin;
    // void int fwd_pin;
    // void int rev_pin;

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
        openI2C();
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
        } else if (speed < 255) {
            speed = 255;
        }

        setState(value, speed);
        RCLCPP_INFO(logger_, "Motor %d value setting, value = %f", motor_num_, value);
    }
    catch (...) {
        RCLCPP_INFO(logger_, "Motor %d value setting failed", motor_num_);
    }

}
void Motor::setState(double value, int speed) {
    if (value > 0.0) {
        setPin(fwd_pin, 1);
        setPin(rev_pin, 0);
        setPWM(fwd_ch, 0, speed*16);
        setPWM(rev_ch, 0, 0);
    } else if (value < 0.0) {
        setPin(fwd_pin, 0);
        setPin(rev_pin, 1);
        setPWM(fwd_ch, 0, 0);
        setPWM(rev_ch, 0, speed*16);
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

    return true;
}

bool Motor::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    return write(i2c_fd_, buffer, 2) == 2;
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
    uint8_t mode1;
    writeRegister(MODE1, OUTDRV);
    writeRegister(MODE1, ALLCALL);
    usleep(500);
    readRegister(MODE1, mode1);
    mode1 = mode1 & SLEEP;
    writeRegister(MODE1, mode1);
    usleep(500);
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

// void Motor::setPWMValue(int channel, double value) {
//     value = std::max(0.0, std::min(1.0, value));
//     int pulse = static_cast<int>(value * 4095);
//     setPWM(channel, 0, pulse);
// }

void Motor::setPWM(int channel, int on, int off) {
    uint8_t reg = LED0_ON_L + 4 * channel;
    writeRegister(reg, on & 0xFF);
    writeRegister(reg + 1, on >> 8);
    writeRegister(reg + 2, off & 0xFF);
    writeRegister(reg + 3, off >> 8);
}

void Motor::setPin(int pin, int value) {
    //TODO add some exception handling
    if (value == 0) {
        setPWM(pin, 0, 4096);
    } else if (value == 1) {
        setPWM(pin, 4096, 0);
    }
}
