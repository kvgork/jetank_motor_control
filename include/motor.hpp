#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <cstdint>
#include "rclcpp/rclcpp.hpp"

class Motor {
public:
    Motor(rclcpp::Logger logger, int motor_num, double alpha = 1.0, double beta = 0.0);
    ~Motor();

    void setValue(double value);
    void stop();

private:
    rclcpp::Logger logger_;
    int motor_num_, pwm_pin, fwd_pin, rev_pin, fwd_ch, rev_ch;
    double alpha_, beta_, value_;
    int i2c_fd_;

    bool openI2C();
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);

    void resetPCA9685();
    void setPWMFreq(int freq);
    void setPWMValue(int channel, double value);
    void setPWM(int channel, int on, int off);
    void setPin(int pin, int value);
    void setState(double value, int speed);
};

#endif
