#ifndef _TRAJECTORY_UTILS_H_
#define _TRAJECTORY_UTILS_H_

#include "numerical_integrator.hpp"
#include <memory>
#include <Eigen/Core>

namespace TrajectoryUtils {

template <typename VectorType = Eigen::Vector3f>
class ArcLengthParameterization {
    using RefFunc = std::function<VectorType(float)>;
public:
    ArcLengthParameterization() = default;
    ~ArcLengthParameterization() = default;

    ArcLengthParameterization(RefFunc position_reference, RefFunc velocity_reference, RefFunc acceleration_reference = nullptr, RefFunc jerk_reference = nullptr, float step = 1e-2) : 
        position_reference_(position_reference), velocity_reference_(velocity_reference), acceleration_reference_(acceleration_reference), jerk_reference_(jerk_reference), step_(step) {
    }

    void setStepSize(float step) {
        step_ = step;
    }

    void tabulateArcLength(float t_final) {
        if (!velocity_reference_) {
            std::cerr << "Velocity reference function not set!" << std::endl;
            return;
        }
        
        auto vel_mag_func = [this](float t) { return velocity_reference_(t).norm(); };
        Integrator::TrapezoidalIntegrator<float> num_int(vel_mag_func, step_);
        s_table_ = num_int.integrate(0.0f, t_final);
        for (size_t i = 0; i < s_table_.size(); i++) {
            t_table_.push_back(((float)i)*step_);
        }
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

        return t_1 + (s - s_1) * step_ / (s_2 - s_1);
    }

    VectorType positionReference(float s) {
    if (!position_reference_) {
        throw std::logic_error("Position reference function not set!");
    }        
        return position_reference_(t(s));
    }

    VectorType velocityReference(float s) {
    if (!velocity_reference_) {
        throw std::logic_error("Velocity reference function not set!");
    }        
        return velocity_reference_(t(s));
    }

    VectorType accelerationReference(float s) {
    if (!acceleration_reference_) {
        throw std::logic_error("Acceleration reference function not set!");
    }        
        return acceleration_reference_(t(s));
    }

    VectorType jerkReference(float s) {
    if (!jerk_reference_) {
        throw std::logic_error("Jerk reference function not set!");
    }        
        return jerk_reference_(t(s));
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
    RefFunc position_reference_{nullptr};
    RefFunc velocity_reference_{nullptr};
    RefFunc acceleration_reference_{nullptr};
    RefFunc jerk_reference_{nullptr};

    std::vector<float> s_table_;
    std::vector<float> t_table_;
    float step_{1e-2};
};

} // End TrajectoryUtils

#endif /* _TRAJECTORY_UTILS_H_ */