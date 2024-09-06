#include <iostream>
#include <chrono>
#include <thread>

class PIDController {
public:
    explicit PIDController() {
        InitTime();
    }
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

    // 设置 PID 算法输出的最小值和最大值
    // 当最小值大于最大值时，视为参数错误，不设置
    void set_output_limits(double min, double max) {
        if (min > max) {
            return;
        }
        out_min_ = min;
        out_max_ = max;

        // 将缓存值 last_output_ 和 err_item_sum_ 限制在最大值和最小值之间
        SetLimits(last_output_);
        SetLimits(err_item_sum_);
    }

    double Compute(double setpoint, double input) {
        uint64_t now = GetMillis();
        uint64_t time_change = now - last_time_;

        if (time_change < sample_time_) {
            return last_output_;
        }

        double error = setpoint - input;
        printf("error: %f\n", error);

        // 积分部分计算，并限制在最大值和最小值之间
        err_item_sum_ += ki_ * error;
        SetLimits(err_item_sum_);

        double derivative = input - last_input_;

        // PID 输出计算，并限制在最大值和最小值之间
        double output = kp_ * error + err_item_sum_ - kd_ * derivative;
        SetLimits(output);

        last_input_ = input;
        last_time_ = now;
        last_output_ = output;
        return output;
    }

private:
    double kp_;
    double ki_;
    double kd_;

    double last_input_ = 0.0;
    double last_output_ = 0.0;
    double err_item_sum_ = 0.0;

    // PID 算法输出的最小值和最大值
    double out_min_ = 0.0;
    double out_max_ = 0.0;

    uint64_t last_time_ = 0UL;
    uint64_t sample_time_ = 1000UL; // 1 second

    uint64_t GetMillis() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    }

    // 内部函数，用于重置变量在最大值和最小值之间
    void SetLimits(double& val) {
        if (val > out_max_) {
            printf("val: %f > out_max_: %f\n", val, out_max_);
            val = out_max_;
        } else if (val < out_min_) {
            printf("val: %f > out_min_: %f\n", val, out_min_);
            val = out_min_;
        } else {
            ; // Do nothing
        }
    }    
};

int main() {
    PIDController pid;
    pid.set_tunings(1, 0.5, 0.05);
    pid.set_sample_time(1000);
    // 设置 PID 算法输出（加热功率）的最小值为0，最大值为100
    pid.set_output_limits(0, 100);

    // 假设我们控制的是一个锅炉，我们希望将温度控制在 90 度，初始温度为20度
    double setpoint = 90;
    double temperature = 20;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    for (int i = 0; i < 1000; ++i) {
        double control_signal = pid.Compute(setpoint, temperature);

        // 模拟锅炉加热，假设加热器效率为0.1，温度会损失0.01
        temperature += control_signal * 0.1;
        temperature *= 0.99;

        std::cout << "Temperature: " << temperature << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}
