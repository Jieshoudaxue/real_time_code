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

// PID 比例部分的两种模式，一是以测例为基础，二是以差值 error 为基础
// Proportional on Measurement(PonM)
// Proportional on Error(PonE)
enum PID_P_MODE: uint8_t {
    PID_P_ON_M = 0,
    PID_P_ON_E = 1
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

    // 添加一个参数，用于设置 PID 比例部分的模式，默认是PID_P_ON_E
    void set_tunings(double kp_para, double ki_para, double kd_para, PID_DIRECTION direction = PID_DIRECT, PID_P_MODE p_mode = PID_P_ON_E) {
        if (kp_para < 0.0 || ki_para < 0.0 || kd_para < 0.0) {
            return;
        }

        // 设置 PID 比例部分的模式
        p_on_e_ = (p_mode == PID_P_ON_E);

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
        SetLimits(p_i_item_sum_);
    }

    void InitInnaState(double input, double output) {
        last_input_ = input;
        // 初始化时，将比例和积分两个部分的累加值设置为上一次的输出值
        p_i_item_sum_ = last_output_;
        SetLimits(p_i_item_sum_);
    }

    void set_auto_mode(PID_MODE mode, double input = 0.0, double output = 0.0) {
        bool new_auto = (mode == PID_MODE_AUTOMATIC);
        if (new_auto == true && in_auto_ == false) {
            InitInnaState(input, output);
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
        printf("error: %f\n", error);

        double derivative = input - last_input_;

        p_i_item_sum_ += ki_ * error;
        // 当比例部分的模式是 PonM 时，将比例部分与积分部分的累加值合并
        if (p_on_e_ == false) {
            p_i_item_sum_ -= kp_ * derivative;
        }
        SetLimits(p_i_item_sum_);

        double output = 0.0;
        if (p_on_e_ == true) {
            output = kp_ * error + p_i_item_sum_ - kd_ * derivative;
        } else {
            // 当比例部分的模式是 PonM 时，P-I-D 将是 PI-D，即比例和积分部分合并 
            output = p_i_item_sum_ - kd_ * derivative;
        }
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

    double out_min_ = 0.0;
    double out_max_ = 0.0;

    uint64_t last_time_ = 0UL;
    uint64_t sample_time_ = 1000UL; // 1 second

    bool in_auto_ = false;

    PID_DIRECTION pid_direct_ = PID_DIRECT;

    // 标识 PID 比例部分的模式，true 表示 Proportional on Error(PonE)，false 表示 Proportional on Measurement(PonM)
    bool p_on_e_ = true;
    // 比例和积分两个部分的累加值
    double p_i_item_sum_ = 0.0;

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

}

int main() {
    YCAO_PIDLIB::PIDController pid;
    // 测试 PonM 模式
    // 这个参数的运行结果就是被控量（水池温度）平滑的上升到设定值，不会出现超调
    pid.set_tunings(0.5, 0.05, 0.0, YCAO_PIDLIB::PID_DIRECT, YCAO_PIDLIB::PID_P_ON_M);
    pid.set_sample_time(1000);
    pid.set_output_limits(0.0, 100.0);

    // 假设我们控制的是一个恒温水池，我们希望将温度控制在 36 度，初始温度为20度
    double setpoint = 36.0;
    double temperature = 20.0;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    pid.set_auto_mode(YCAO_PIDLIB::PID_MODE_AUTOMATIC, temperature);

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
