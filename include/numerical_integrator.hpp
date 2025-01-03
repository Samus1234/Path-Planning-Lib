#ifndef _NUMERICAL_INTEGRATOR_H_
#define _NUMERICAL_INTEGRATOR_H_

#include <iostream>
#include <algorithm>
#include <functional>

namespace Integrator {

template<typename T = double>
class TrapezoidalIntegrator {
    static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
public:
    TrapezoidalIntegrator() = default;
    ~TrapezoidalIntegrator() = default;

    TrapezoidalIntegrator(std::function<T(T)> f, T step) : f_(f), step_(step) {
    }

    std::vector<T> integrate(T x_initial, T x_final) {
        T integral = 0;
        T x_k = x_initial;

        std::vector<T> integral_table;
        integral_table.reserve(static_cast<size_t>((x_final - x_initial) / step_));

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

template<typename T = double>
class SimpsonsIntegrator {
    static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
public:
    SimpsonsIntegrator() = default;
    ~SimpsonsIntegrator() = default;

    SimpsonsIntegrator(std::function<T(T)> f, T step) : f_(f), step_(step) {
    }

    std::vector<T> integrate(T x_initial, T x_final) {
        T integral = 0;
        T x_k = x_initial;

        std::vector<T> integral_table;
        integral_table.reserve(static_cast<size_t>((x_final - x_initial) / step_));

        while (x_k < x_final) {
            T step = std::min(step_, x_final - x_k);
            if (x_k + 2 * step > x_final) {
                integral += step * (f_(x_k) + f_(x_k + step)) / 2;
                break;
            }

            T delta = (step / 3) * (f_(x_k) + 4 * f_(x_k + step / 2) + f_(x_k + step));
            integral += delta;
            x_k += step;
            integral_table.push_back(integral);
        }
        integral_table.push_back(integral);
        return integral_table;
    }

private:
    std::function<T(T)> f_;
    T step_;
};

}



#endif /* _NUMERICAL_INTEGRATOR_H_ */