#include <iostream>
#include <Eigen/Core>
#include <vector>

template<typename T>
class Spline {
public:
    using SplineVector = Eigen::Matrix<T, 4, 1>;
public:
    Spline() = default;
    ~Spline() = default;

    Spline(std::vector<T> c) {
        c_(c.data(), 4);
    }

    Spline(SplineVector c) : c_(c) {
    }

    SplineVector getTimeVector(float t) {
        SplineVector t_vec;
        t_vec << 1, t, t*t, t*t*t;
        return t_vec;
    }

    float computeSpline(float t) {
        return c_.dot(getTimeVector(t));
    }


private:
    SplineVector c_{SplineVector::Zero()};
};


int main(int argc, char** argv) {

    Spline<float>::SplineVector c_1;

    c_1 << 0.0, 1.0, 0.5, -0.3;

    Spline<float>::SplineVector c_2;

    c_2 << 1.2, -0.5, 0.3, -0.1;

    Spline<float> spline_1(c_1);

    Spline<float> spline_2(c_2);

    std::vector<float> t_points = {0.25, 0.5, 0.75, 1.25, 1.5, 1.75};

    for (const auto& t : t_points) {
        std::cout << spline_1.computeSpline(t) << " ";
    }

    std::cout << "\n";

    for (const auto& t : t_points) {
        std::cout << spline_2.computeSpline(t) << " ";
    }

    std::cout << "" << std::endl;
    

    return 0;
}