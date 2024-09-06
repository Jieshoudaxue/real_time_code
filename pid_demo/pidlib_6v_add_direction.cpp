#include <iostream>
#include <chrono>
#include <thread>

namespace YCAO_PIDLIB {

enum PID_MODE: uint8_t {
    PID_MODE_MANUAL = 0,
    PID_MODE_AUTOMATIC = 1
};

enum PID_DIRECTION: uint8_t {
    PID_DIRECT = 0,
    PID_REVERSE = 1
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

    void set_tunings(double kp_para, double ki_para, double kd_para, PID_DIRECTION direction = PID_DIRECT) {
        if (kp_para < 0 || ki_para < 0 || kd_para < 0) {
            return;
        }

        double sample_time_in_sec = static_cast<double>(sample_time_) / 1000.0;
        kp_ = kp_para;
        ki_ = ki_para * sample_time_in_sec;
        kd_ = kd_para / sample_time_in_sec;

        if (pid_direct_ == PID_REVERSE) {
            kp_ = 0 - kp_;
            ki_ = 0 - ki_;
            kd_ = 0 - kd_;
        }
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

    void InitInnaState(double input) {
        last_input_ = input;
        err_item_sum_ = last_output_;
        SetLimits(err_item_sum_);
    }

    void set_auto_mode(PID_MODE mode, double input = 0.0) {
        bool new_auto = (mode == PID_MODE_AUTOMATIC);
        if (new_auto == true && in_auto_ == false) {
            InitInnaState(input);
        }
        in_auto_ = new_auto;
        std::cout << "PID mode: " << (in_auto_ ? "Automatic" : "Manual") << std::endl;
    }

    double Compute(double setpoint, double input) {
        if (in_auto_ == false) {
            return last_output_;
        }

        uint64_t now = GetMillis();
        uint64_t time_change = now - last_time_;

        if (time_change < sample_time_) {
            return last_output_;
        }

        double error = setpoint - input;
        // printf("error: %f\n", error);

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

    bool in_auto_ = false;

    PID_DIRECTION pid_direct_ = PID_DIRECT;

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
            printf("val: %f < out_min_: %f\n", val, out_min_);
            val = out_min_;
        } else {
            ; // Do nothing
        }
    }
};

}   // namespace YCAO_PIDLIB

int main() {
    YCAO_PIDLIB::PIDController pid;
    pid.set_tunings(2.0, 0.1, 0.01, YCAO_PIDLIB::PID_REVERSE);
    pid.set_sample_time(1000);
    pid.set_output_limits(-100.0, 100.0);

    // 假设我们控制的是一个冷库，目标是将温度控制在 -26 度，初始温度是 30 度
    double setpoint = -26.0;
    double temperature = 30.0;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    pid.set_auto_mode(YCAO_PIDLIB::PID_MODE_AUTOMATIC, temperature);

    for (int i = 0; i < 1000; ++i) {
        double control_signal = pid.Compute(setpoint, temperature);

        // 模拟压缩机的控制，假设压缩机的热效率是 0.05，环境温度每秒上升 1%
        temperature += control_signal * 0.05;
        temperature *= 1.01;

        std::cout << "Temperature: " << temperature << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}
