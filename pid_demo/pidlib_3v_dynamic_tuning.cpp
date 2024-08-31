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

    void set_tunings(double kp_para, double ki_para, double kd_para) {
        double sample_time_in_sec = static_cast<double>(sample_time_) / 1000.0;
        kp_ = kp_para;
        ki_ = ki_para * sample_time_in_sec;
        kd_ = kd_para / sample_time_in_sec;
    }

    void set_sample_time(uint64_t new_sample_time) {
        if (new_sample_time > 0) {
            double ratio = static_cast<double>(new_sample_time) / static_cast<double>(sample_time_);
            ki_ = ki_ * ratio;
            kd_ = kd_ / ratio;
            sample_time_ = new_sample_time;
        }
    }

    // Calculate and update the output based on setpoint and actual value
    double Compute(double setpoint, double input) {
        uint64_t now = GetMillis();
        uint64_t time_change = now - last_time_;

        if (time_change < sample_time_) {
            return last_output_;
        }

        double error = setpoint - input;
        printf("error: %f\n", error);

        // err_sum_ term
        err_item_sum_ += ki_ * error;

        // Derivative term
        double derivative = input - last_input_;

        // Total output
        double output = kp_ * error + err_item_sum_ - kd_ * derivative;

        // Save error to previous error for next iteration
        last_input_ = input;
        last_time_ = now;
        last_output_ = output;
        return output;
    }

private:
    double kp_; // Proportional gain
    double ki_; // err_sum_ gain
    double kd_; // Derivative gain

    double last_input_ = 0.0;
    double err_item_sum_ = 0.0;
    uint64_t last_time_ = 0UL;

    double last_output_ = 0.0;
    uint64_t sample_time_ = 1000UL; // 1 second

    // Utility function to get the elapsed time in seconds since the last call
    uint64_t GetMillis() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    }
};

int main() {
    PIDController pid; // Create PID controller
    pid.set_tunings(1, 0.5, 0.05); // Set PID coefficients
    pid.set_sample_time(1000); // Set sample time to 1 second

    // 假设我们控制的是一个锅炉，我们希望将温度控制在100度，初始温度为20度
    double setpoint = 100;
    double temperature = 20;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Simulate control loop
    for (int i = 0; i < 1000; ++i) {
        // Calculate control signal
        double control_signal = pid.Compute(setpoint, temperature);

        // 模拟锅炉加热，假设加热器效率为0.1，温度会损失0.01
        temperature += control_signal * 0.1;
        temperature *= 0.99;

        std::cout << "Temperature: " << temperature << std::endl;

        if (i == 200) {
            pid.set_tunings(1, 0.6, 0.06); // Change PID coefficients halfway through
            std::cout << "PID coefficients changed, 1, 0.5, 0.05 ->1, 0.6, 0.06" << std::endl;
        }    

        // Sleep for a bit (simulate one second delay)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}
