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

    // 设置 PID 控制器的三个参数，由于固定了采样时间，因此可以提前计算好 ki 和 kd，而不必每次在 Compute 函数中计算
    void set_tunings(double kp_para, double ki_para, double kd_para) {
        // 采样间隔，单位是毫秒，这里转换为秒。默认采样间隔是 1 秒
        double sample_time_in_sec = static_cast<double>(sample_time_) / 1000.0;
        kp_ = kp_para;
        // 由于积分部分是每次差值 e 乘以采样间隔，然后累加，因此这里提前将 ki 乘以固定的采样间隔，并保存起来，数学表达式如下：
        // sum = ki * (error(0) * dt + error(1) * dt + ... + error(n) * dt) = (ki * dt) * (error(0) + error(1) + ... + error(n))
        ki_ = ki_para * sample_time_in_sec;
        // 由于微分部分是每次差值 e 除以采样间隔，因此这里提前将 kd 除以固定的采样间隔，并保存起来，数学表达式如下：
        // derivative = kd * (error(n) - error(n-1)) / dt = (kd / dt) * (error(n) - error(n-1))
        kd_ = kd_para / sample_time_in_sec;
    }

    // 设置采样间隔，单位是毫秒
    // 当设置新的采样间隔时，需要按比例重新更新 ki 和 kd，并保存新的采样间隔
    void set_sample_time(uint64_t new_sample_time) {
        if (new_sample_time > 0) {
            // 计算新采样间隔和默认采样间隔的比例
            double ratio = static_cast<double>(new_sample_time) / static_cast<double>(sample_time_);
            ki_ = ki_ * ratio;
            kd_ = kd_ / ratio;
            sample_time_ = new_sample_time;
        }
    }

    double Compute(double setpoint, double input) {
        uint64_t now = GetMillis();
        uint64_t time_change = now - last_time_;

        // 这里注意：如果采样间隔小于设定的采样间隔，直接返回上一次的输出值，不再进行计算
        if (time_change < sample_time_) {
            return last_output_;
        }

        double error = setpoint - input;
        printf("error: %f\n", error);

        // 积分部分
        // 由于采样间隔是固定的，且提前计算并保存到了 ki_ 中，因此这里只需要累加即可
        err_sum_ += error;

        // 微分部分
        // 由于采样间隔是固定的，且提前计算并保存到了 kd_ 中，因此这里只需要计算差值即可
        double derivative = error - last_error_;

        // 简化后的计算表达式，省去了每次的采样间隔的浮点计算
        double output = kp_ * error + ki_ * err_sum_ + kd_ * derivative;

        last_error_ = error;
        last_time_ = now;
        // 由于采样间隔是固定的，当实际采样间隔小于设定的采样间隔时，直接返回上一次的输出值，因此每次计算后都需要保存当前值
        last_output_ = output;
        return output;
    }

private:
    double kp_;
    double ki_;
    double kd_;

    double last_error_ = 0.0;
    double err_sum_ = 0.0;
    uint64_t last_time_ = 0UL;

    // 保存上一次的输出值，当实际采样间隔小于设定的采样间隔时，直接返回上一次的输出值
    double last_output_ = 0.0;
    // 默认采样间隔是 1 秒
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
    pid.set_sample_time(1000); // Set sample time to 1 second

    // 假设我们控制的是一个锅炉，我们希望将温度控制在36度，初始温度为20度
    double setpoint = 36;
    double temperature = 20;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    for (int i = 0; i < 1000; ++i) {
        double control_signal = pid.Compute(setpoint, temperature);

        // 模拟锅炉加热，假设加热器效率为0.1，温度会损失0.01
        temperature += control_signal * 0.1;
        temperature *= 0.99;

        std::cout << "Temperature: " << temperature << std::endl;

        // 由于内部采样间隔固定为 1 秒，因此这里加快了计算周期
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}
