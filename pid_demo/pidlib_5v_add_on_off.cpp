#include <iostream>
#include <chrono>
#include <thread>

// PID 工具模式枚举量：手动模式和自动模式
enum PID_MODE: uint8_t {
    PID_MODE_MANUAL = 0,
    PID_MODE_AUTOMATIC = 1
};

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

    void set_output_limits(double min, double max) {
        if (min > max) {
            return;
        }
        out_min_ = min;
        out_max_ = max;

        SetLimits(last_output_);
        SetLimits(err_item_sum_);
    }

    // 当从手动模式切换到自动模式时，重新初始化 PID 内部状态
    // 一是更新算法输入值，确保比例和微分部分按照新的状态重新计算
    // 二是更新算法积分部分的历史值，确保积分部分不会对新的算法输出产生扰动
    void InitInnaState(double input, double output) {
        last_input_ = input;
        err_item_sum_ = output;
        SetLimits(err_item_sum_);
    }

    // 设置 PID 控制器的工作模式：手动模式和自动模式
    // 当从手动模式切换到自动模式时，需要给出新的算法输入值和输出值，用于初始化 PID 内部状态
    void set_auto_mode(PID_MODE mode, double input = 0.0, double output = 0.0) {
        // 当识别出模式从手动切换到自动时，初始化 PID 内部状态
        bool new_auto = (mode == PID_MODE_AUTOMATIC);
        if (new_auto == true && in_auto_ == false) {
            InitInnaState(input, output);
        }
        in_auto_ = new_auto;
        std::cout << "PID mode: " << (in_auto_ ? "Automatic" : "Manual") << std::endl;
    }

    double Compute(double setpoint, double input) {
        // 当 PID 控制器处于手动模式时，直接返回上一次的输出值，外部会使用人工操作值覆盖 PID 算法的输出值
        if (in_auto_ == false) {
            return last_output_;
        }

        uint64_t now = GetMillis();
        uint64_t time_change = now - last_time_;

        if (time_change < sample_time_) {
            return last_output_;
        }

        double error = setpoint - input;
        printf("error: %f\n", error);

        err_item_sum_ += ki_ * error;
        SetLimits(err_item_sum_);

        double derivative = input - last_input_;

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

    double out_min_ = 0.0;
    double out_max_ = 0.0;

    uint64_t last_time_ = 0UL;
    uint64_t sample_time_ = 1000UL; // 1 second

    // PID 内部状态控制量：false 表示手动模式，true 表示自动模式
    bool in_auto_ = false;

    uint64_t GetMillis() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    }

    void SetLimits(double& val) {
        if (val > out_max_) {
            printf("val: %f > out_max_: %f\n", val, out_max_);
            val = out_max_;
        } else if (val < out_min_) {
            val = out_min_;
        } else {
            ; // Do nothing
        }
    }
};

int main() {
    PIDController pid;
    pid.set_tunings(1, 0.2, 0.02);
    pid.set_sample_time(1000);
    pid.set_output_limits(0, 100);

    // 假设我们控制的是一个恒温水池，我们希望将温度控制在 36 度，初始温度为20度
    double setpoint = 36.0;
    double temperature = 20.0;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 初始化时，设置 PID 控制器为自动模式
    pid.set_auto_mode(PID_MODE_AUTOMATIC);

    for (int i = 0; i < 1000; ++i) {
        // 当 i 等于 200 时，将 PID 控制器切换到手动模式
        if (i == 200) {
            pid.set_auto_mode(PID_MODE_MANUAL);
            std::cout << "---->>> Switch to manual mode" << std::endl;
        }

        double control_signal = pid.Compute(setpoint, temperature);

        // 切换到手动模式时，这里模拟人工的操作，人工操作值将覆盖 PID 算法的输出值
        if (i >= 200 && i < 250) {
            control_signal = 3;
        }
        if (i >= 250 && i <= 300) {
            control_signal = 4;
        }

        std::cout << "--> Control signal: " << control_signal << std::endl;

        // 模拟锅炉加热，假设加热器效率为0.1，温度会损失0.01
        temperature += control_signal * 0.1;
        temperature *= 0.99;

        std::cout << "<-- Temperature: " << temperature << std::endl;

        // 当 i 等于 300 时，将 PID 控制器重新切换到自动模式
        if (i == 300) {
            pid.set_auto_mode(PID_MODE_AUTOMATIC, temperature, control_signal);
            std::cout << "---->>> Switch back to automatic mode" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}
