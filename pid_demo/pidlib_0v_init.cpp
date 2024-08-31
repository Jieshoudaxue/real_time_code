#include <iostream>
#include <chrono>
#include <thread>

class PIDController {
public:
    explicit PIDController() {
        InitTime();
    }

    // 构造函数，初始化 PID 控制器的三个参数和系统初始时间
    PIDController(double kp_para, double ki_para, double kd_para) : kp_(kp_para), ki_(ki_para), kd_(kd_para) {
        InitTime();
    }

    void InitTime() {
        last_time_ = GetMillis();
    }

    // Compute 是 PID 控制器的核心函数，根据设定值和算法输入值计算输出值
    double Compute(double setpoint, double input) {
        uint64_t now = GetMillis();
        // 两次采样间隔
        double time_change = static_cast<double>(now - last_time_);

        // 差值 error
        double error = setpoint - input;
        printf("error: %f\n", error);

        // 积分部分
        err_sum_ += error * time_change;

        // 微分部分
        double derivative = (error - last_error_) / time_change;

        // 三个部分相加得到算法输出值
        double output = kp_ * error + ki_ * err_sum_ + kd_ * derivative;

        // 保存当前值，供下一次计算使用
        last_error_ = error;
        last_time_ = now;
        return output;
    }

    // 设置 PID 控制器的三个参数
    void set_tunings(double kp_para, double ki_para, double kd_para) {
        kp_ = kp_para;
        ki_ = ki_para;
        kd_ = kd_para;
    }

private:
    double kp_; // 比例系数
    double ki_; // 积分系数
    double kd_; // 微分系数

    double last_error_ = 0; // 上一次的差值 error
    double err_sum_ = 0;    // 积分部分
    uint64_t last_time_ = 0; // 上一次的采样时间

    // 获取当前时间，单位是毫秒
    uint64_t GetMillis() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    }
};

int main() {
    PIDController pid;
    pid.set_tunings(10, 0.01, 0.01);

    // 假设我们控制的是一个锅炉，我们希望将温度控制在36度，初始温度为20度
    double setpoint = 36;
    double temperature = 20;

    // 模拟系统启动延迟
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 模拟控制循环
    for (int i = 0; i < 100; ++i) {
        // 计算控制输出，这里是加热功率
        double control_signal = pid.Compute(setpoint, temperature);

        // 模拟锅炉加热，假设加热器效率为0.1，温度会损失0.01
        temperature += control_signal * 0.1;
        temperature *= 0.99;
        
        std::cout << "Temperature: " << temperature << std::endl;

        // 模拟控制周期
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
