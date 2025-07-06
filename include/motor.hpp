#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <memory>
#include <functional>

class Motor {
public:
    Motor(int pwm_pin, int in1_pin, int in2_pin, double alpha = 1.0, double beta = 0.0);
    ~Motor();
    
    void setValue(double value);
    double getValue() const { return value_; }
    
    void setAlpha(double alpha) { alpha_ = alpha; }
    void setBeta(double beta) { beta_ = beta; }
    
    void stop();
    void release();

private:
    void writeValue(double value);
    void setupGPIO();
    void cleanupGPIO();
    
    int pwm_pin_;
    int in1_pin_;
    int in2_pin_;
    
    double value_;
    double alpha_;
    double beta_;
    
    bool gpio_initialized_;
    
    // GPIO file descriptors
    int pwm_fd_;
    int in1_fd_;
    int in2_fd_;
    
    // Helper methods for GPIO control
    bool exportGPIO(int pin);
    bool setGPIODirection(int pin, const std::string& direction);
    bool writeGPIO(int pin, int value);
    bool setPWMDutyCycle(int pin, int duty_cycle);
    bool setPWMPeriod(int pin, int period_ns);
    bool enablePWM(int pin, bool enable);
};

#endif // MOTOR_HPP