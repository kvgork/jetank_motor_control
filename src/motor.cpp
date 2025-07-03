#include "motor.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

Motor::Motor(int pwm_pin, int in1_pin, int in2_pin, double alpha, double beta)
    : pwm_pin_(pwm_pin), in1_pin_(in1_pin), in2_pin_(in2_pin),
      value_(0.0), alpha_(alpha), beta_(beta), gpio_initialized_(false),
      pwm_fd_(-1), in1_fd_(-1), in2_fd_(-1) {
    setupGPIO();
    stop();
}

Motor::~Motor() {
    release();
    cleanupGPIO();
}

void Motor::setValue(double value) {
    value_ = value;
    writeValue(value);
}

void Motor::writeValue(double value) {
    if (!gpio_initialized_) {
        std::cerr << "GPIO not initialized, cannot write value" << std::endl;
        return;
    }
    
    // Apply alpha and beta scaling
    double mapped_value = alpha_ * value + beta_;
    
    // Convert to PWM duty cycle (0-255 -> 0-100%)
    int duty_cycle = static_cast<int>(std::abs(mapped_value) * 100.0);
    duty_cycle = std::max(0, std::min(100, duty_cycle));
    
    // Set PWM duty cycle
    setPWMDutyCycle(pwm_pin_, duty_cycle);
    
    // Set direction based on sign
    if (mapped_value < 0) {
        // Forward direction
        writeGPIO(in1_pin_, 1);
        writeGPIO(in2_pin_, 0);
    } else if (mapped_value > 0) {
        // Backward direction
        writeGPIO(in1_pin_, 0);
        writeGPIO(in2_pin_, 1);
    } else {
        // Stop
        writeGPIO(in1_pin_, 0);
        writeGPIO(in2_pin_, 0);
    }
}

void Motor::stop() {
    setValue(0.0);
}

void Motor::release() {
    if (gpio_initialized_) {
        writeGPIO(in1_pin_, 0);
        writeGPIO(in2_pin_, 0);
        setPWMDutyCycle(pwm_pin_, 0);
        enablePWM(pwm_pin_, false);
    }
}

void Motor::setupGPIO() {
    // Export GPIO pins
    if (!exportGPIO(pwm_pin_) || !exportGPIO(in1_pin_) || !exportGPIO(in2_pin_)) {
        std::cerr << "Failed to export GPIO pins" << std::endl;
        return;
    }
    
    // Set directions
    if (!setGPIODirection(in1_pin_, "out") || !setGPIODirection(in2_pin_, "out")) {
        std::cerr << "Failed to set GPIO directions" << std::endl;
        return;
    }
    
    // Setup PWM
    std::ostringstream pwm_export_path;
    pwm_export_path << "/sys/class/pwm/pwmchip0/export";
    std::ofstream pwm_export(pwm_export_path.str());
    if (pwm_export.is_open()) {
        pwm_export << pwm_pin_;
        pwm_export.close();
    }
    
    // Set PWM period (20kHz -> 50000ns)
    setPWMPeriod(pwm_pin_, 50000);
    enablePWM(pwm_pin_, true);
    
    gpio_initialized_ = true;
}

void Motor::cleanupGPIO() {
    if (gpio_initialized_) {
        release();
        
        // Unexport GPIO pins
        std::ofstream unexport("/sys/class/gpio/unexport");
        if (unexport.is_open()) {
            unexport << in1_pin_ << std::endl;
            unexport << in2_pin_ << std::endl;
            unexport.close();
        }
        
        // Unexport PWM
        std::ostringstream pwm_unexport_path;
        pwm_unexport_path << "/sys/class/pwm/pwmchip0/unexport";
        std::ofstream pwm_unexport(pwm_unexport_path.str());
        if (pwm_unexport.is_open()) {
            pwm_unexport << pwm_pin_;
            pwm_unexport.close();
        }
        
        gpio_initialized_ = false;
    }
}

bool Motor::exportGPIO(int pin) {
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file.is_open()) {
        std::cerr << "Failed to open GPIO export file" << std::endl;
        return false;
    }
    
    export_file << pin;
    export_file.close();
    
    // Wait a bit for the GPIO to be exported
    usleep(100000); // 100ms
    
    return true;
}

bool Motor::setGPIODirection(int pin, const std::string& direction) {
    std::ostringstream direction_path;
    direction_path << "/sys/class/gpio/gpio" << pin << "/direction";
    
    std::ofstream direction_file(direction_path.str());
    if (!direction_file.is_open()) {
        std::cerr << "Failed to open GPIO direction file for pin " << pin << std::endl;
        return false;
    }
    
    direction_file << direction;
    direction_file.close();
    
    return true;
}

bool Motor::writeGPIO(int pin, int value) {
    std::ostringstream value_path;
    value_path << "/sys/class/gpio/gpio" << pin << "/value";
    
    std::ofstream value_file(value_path.str());
    if (!value_file.is_open()) {
        std::cerr << "Failed to open GPIO value file for pin " << pin << std::endl;
        return false;
    }
    
    value_file << value;
    value_file.close();
    
    return true;
}

bool Motor::setPWMDutyCycle(int pin, int duty_cycle) {
    std::ostringstream duty_cycle_path;
    duty_cycle_path << "/sys/class/pwm/pwmchip0/pwm" << pin << "/duty_cycle";
    
    std::ofstream duty_cycle_file(duty_cycle_path.str());
    if (!duty_cycle_file.is_open()) {
        std::cerr << "Failed to open PWM duty cycle file for pin " << pin << std::endl;
        return false;
    }
    
    // Convert percentage to nanoseconds (period is 50000ns)
    int duty_ns = (duty_cycle * 50000) / 100;
    duty_cycle_file << duty_ns;
    duty_cycle_file.close();
    
    return true;
}

bool Motor::setPWMPeriod(int pin, int period_ns) {
    std::ostringstream period_path;
    period_path << "/sys/class/pwm/pwmchip0/pwm" << pin << "/period";
    
    std::ofstream period_file(period_path.str());
    if (!period_file.is_open()) {
        std::cerr << "Failed to open PWM period file for pin " << pin << std::endl;
        return false;
    }
    
    period_file << period_ns;
    period_file.close();
    
    return true;
}

bool Motor::enablePWM(int pin, bool enable) {
    std::ostringstream enable_path;
    enable_path << "/sys/class/pwm/pwmchip0/pwm" << pin << "/enable";
    
    std::ofstream enable_file(enable_path.str());
    if (!enable_file.is_open()) {
        std::cerr << "Failed to open PWM enable file for pin " << pin << std::endl;
        return false;
    }
    
    enable_file << (enable ? 1 : 0);
    enable_file.close();
    
    return true;
}