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

    double Compute(double setpoint, double input) {
        uint64_t now = GetMillis();
        uint64_t time_change = now - last_time_;

        if (time_change < sample_time_) {
            return last_output_;
        }

        double error = setpoint - input;
        printf("error: %f\n", error);

        err_sum_ += error;

        // 由于差值 error 的导数也是算法输入的斜率负数，因此这里直接使用算法输入 input 计算微分部分
        double derivative = input - last_input_;
        // 由于差值 error 的导数也是算法输入的斜率负数，因此微分部分的符号是负数
        double output = kp_ * error + ki_ * err_sum_ - kd_ * derivative;

        // 由于差值 error 不再参与微分部分的计算，而是直接使用算法输入 input，因此这里不再保存 error，而是保存 input
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
    double err_sum_ = 0.0;
    uint64_t last_time_ = 0UL;

    double last_output_ = 0.0;
    uint64_t sample_time_ = 1000UL; // 1 second

    uint64_t GetMillis() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    }
};

int main() {
    PIDController pid;
    pid.set_tunings(1, 0.2, 0.02);
    pid.set_sample_time(1000);

    // 假设我们控制的是一个恒温水池，我们希望将温度控制在36度，初始温度为20度
    double setpoint = 36.0;
    double temperature = 20.0;
    
    std::this_thread::sleep_for(std::chrono::seconds(1));

    for (int i = 0; i < 1000; ++i) {
        double control_signal = pid.Compute(setpoint, temperature);

        // 模拟锅炉加热，假设加热器效率为0.1，温度会损失0.01
        temperature += control_signal * 0.1;
        temperature *= 0.99;

        std::cout << "Temperature: " << temperature << std::endl;

        // 系统运行过程中，突然将目标温度从 36 度调整到 50 度
        if (i == 200) {
            setpoint = 50; 
            std::cout << "Setpoint changed to 50" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}
