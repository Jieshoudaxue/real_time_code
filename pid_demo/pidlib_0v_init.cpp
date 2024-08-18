#include <iostream>
#include <chrono>
#include <thread>

// PID Controller Class
class PIDController {
public:
    explicit PIDController() {
        InitTime();
    }
    // Constructor with initial PID coefficients
    PIDController(double kp_para, double ki_para, double kd_para) : kp_(kp_para), ki_(ki_para), kd_(kd_para) {
        InitTime();
    }

    void InitTime() {
        last_time_ = GetMillis();
    }

    // Calculate and update the output based on setpoint and actual value
    double Compute(double setpoint, double input) {
        uint64_t now = GetMillis();
        double time_change = static_cast<double>(now - last_time_);

        double error = setpoint - input;
        printf("error: %f\n", error);

        // err_sum_ term
        err_sum_ += error * time_change;

        // Derivative term
        double derivative = (error - last_error_) / time_change;

        // Total output
        double output = kp_ * error + ki_ * err_sum_ + kd_ * derivative;

        // Save error to previous error for next iteration
        last_error_ = error;
        last_time_ = now;
        return output;
    }

    void set_tunings(double kp_para, double ki_para, double kd_para) {
        kp_ = kp_para;
        ki_ = ki_para;
        kd_ = kd_para;
    }

private:
    double kp_; // Proportional gain
    double ki_; // err_sum_ gain
    double kd_; // Derivative gain

    double last_error_ = 0;
    double err_sum_ = 0;
    uint64_t last_time_ = 0;

    // Utility function to get the elapsed time in seconds since the last call
    uint64_t GetMillis() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    }
};

int main() {
    PIDController pid; // Create PID controller
    pid.set_tunings(10, 0.01, 0.01); // Set PID coefficients

    // 假设我们控制的是一个锅炉，我们希望将温度控制在100度，初始温度为20度
    double setpoint = 100;
    double temperature = 20;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Simulate control loop
    for (int i = 0; i < 100; ++i) {
        // Calculate control signal
        double control_signal = pid.Compute(setpoint, temperature);

        // 模拟锅炉加热，假设加热器效率为0.1，温度会损失0.01
        temperature += control_signal * 0.1;
        temperature *= 0.99;
        
        std::cout << "Temperature: " << temperature << std::endl;

        // Sleep for a bit (simulate one second delay)
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
