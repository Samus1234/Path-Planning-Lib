#include <iostream>
#include <memory>
#include <algorithm>
#include <functional>
#include <Eigen/Core>

template<typename T = double>
class NumericalIntegrator {
    static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
public:
    NumericalIntegrator() = default;
    ~NumericalIntegrator() = default;

    NumericalIntegrator(std::function<T(T)> f, T step) : f_(f), step_(step) {
    }

    std::vector<T> integrate(T x_initial, T x_final) {
        T integral = 0;
        T x_k = x_initial;

        std::vector<T> integral_table;
        integral_table.reserve((size_t) (x_final-x_initial)/step_);

        while (x_k < x_final) {
            integral_table.push_back(integral);
            T step = std::min(step_, x_final - x_k);
            T delta = step * (f_(x_k) + f_(x_k + step)) / 2;
            integral += delta;
            x_k += step;
        }
        integral_table.push_back(integral);
        return integral_table;
    }

private:
    std::function<T(T)> f_;
    T step_;
};

class Trajectory {
public:
    Trajectory() = default;
    ~Trajectory() = default;

    Eigen::Vector3f pRef(float t) {
        Eigen::Vector3f p;
        p << cos(t), sin(t), 0.5f*t;
        return p;
    }

    Eigen::Vector3f vRef(float t) {
        Eigen::Vector3f p;
        p << sin(t), -cos(t), 0.5f;
        return p;
    }

    Eigen::Vector3f pRefArc(float s) {
        return pRef(t(s));
    }

    Eigen::Vector3f vRefArc(float s) {
        return vRef(t(s));
    }

    float vRefMag(float t) {
        return vRef(t).norm();
    }

    void tabulateArcLength(float t_final) {
        NumericalIntegrator<float> num_int([this](float t) { return vRefMag(t); }, step_);
        s_table_ = num_int.integrate(0.0f, t_final);
        for (size_t i = 0; i < s_table_.size(); i++) {
            t_table_.push_back(((float)i)*step_);
        }
    }

    void printTable() {
        for (const auto& s: s_table_) {
            std::cout << s << "\n";
        }
        std::cout << "" << std::endl;
    }

    float t(float s) {
        if (s < 0) {
            throw std::out_of_range("Out of bounds!");
        }

        if (s_table_.empty() || t_table_.empty()) {
            throw std::out_of_range("Not tabulated!");
        }
        

        if (s >= s_table_.back()) {
            return t_table_.back();
        }
        
        auto it = std::lower_bound(s_table_.begin(), s_table_.end(), s);
        size_t i = std::distance(s_table_.begin(), it);
        float s_1 = s_table_[i];
        float s_2 = s_table_[i+1];
        float t_1 = t_table_[i];
        float interpolated_t = t_1 + (s - s_1) * step_ / (s_2 - s_1);

        std::cout << "s: " << s << ", s_1: " << s_1 << ", s_2: " << s_2
                << ", t_1: " << t_1 << ", interpolated t: " << interpolated_t << std::endl;

        return interpolated_t;
    }

    float getMaxArcLength() {
        if (s_table_.empty()) {
            throw std::out_of_range("Not tabulated!");
        }
        return s_table_.back();
    }

    float getMaxTime() {
        if (t_table_.empty()) {
            throw std::out_of_range("Not tabulated!");
        }
        return t_table_.back();
    }

private:
    std::vector<float> s_table_;
    std::vector<float> t_table_;
    float step_{1e-2};

};

int main(int argc, char* argv[]) {

    Trajectory traj;

    traj.tabulateArcLength(4*M_PI);

    float max_arc_length = traj.getMaxArcLength();
    float max_time = traj.getMaxTime();

    std::cout << "Max time: " << max_time << " and max arc length: " << max_arc_length << std::endl;

    std::ignore = traj.t(0);
    std::ignore = traj.t(max_arc_length/4);
    std::ignore = traj.t(max_arc_length/2);
    std::ignore = traj.t(3*max_arc_length/4);
    std::ignore = traj.t(max_arc_length);
    return 0;
}