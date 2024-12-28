#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>

#include <fstream>
#include <iomanip>
#include <string>

static constexpr double g = 9.8;

namespace Drone {

template <typename T>
struct Parameters {
  Parameters() {
    static_assert(std::is_floating_point_v<T>, "Type must be float or double");
    I(0, 0) = Ix;
    I(1, 1) = Iy;
    I(2, 2) = Iz;

    A(0, 0) = Ax;
    A(1, 1) = Ay;
    A(2, 2) = Az;

    Gt << 0, 0, 0, 0,
          0, 0, 0, 0,
          1, 1, 1, 1;

    T k = Km/Kf;

    Gm << 0, -l, 0, l,
          -l, 0, l, 0,
          -k, k, -k, k;
    
    g_v << 0, 0, (T)-g;
  }

  T m{0.468};
  T l{0.225};
  T Ix{4.856e-3};
  T Iy{4.856e-3};
  T Iz{8.801e-3};
  T Kf{2.980e-3};
  T Km{1.490e-3};
  T Ax{10e-2};
  T Ay{10e-2};
  T Az{10e-2};

  Eigen::Matrix<T, 3, 3> I;
  Eigen::Matrix<T, 3, 3> A;
  Eigen::Matrix<T, 3, 4> Gt;
  Eigen::Matrix<T, 3, 4> Gm;
  Eigen::Matrix<T, 3, 1> g_v;
};

template <typename T> 
class Quaternion {
public:
  using Quat = Eigen::Matrix<T, 4, 1>;
  using EulerAngle = Eigen::Matrix<T, 3, 1>;
  using QuatMatrix = Eigen::Matrix<T, 4, 4>;
  using RotMatrix = Eigen::Matrix<T, 3, 3>;
public:
  Quaternion() = delete;
  ~Quaternion() = delete;

  static const QuatMatrix Lq(Quat q) {
    QuatMatrix L_q;
    L_q << q(0), -q(1), -q(2), -q(3),
           q(1), q(2), -q(3), q(0),
           q(2), q(3), q(0), -q(1),
           q(3), -q(0), q(1), q(2);
    return L_q;
  }

  static const QuatMatrix Rq(Quat q) {
    QuatMatrix R_q;
    R_q << q(0), -q(1), -q(2), -q(3),
           q(1), q(2), q(3), -q(0),
           q(2), -q(3), q(0), q(1),
           q(3), q(0), -q(1), q(2);
    return R_q;
  }

  static const RotMatrix R(Quat q) {
    RotMatrix R;
    R << 1 - 2*(q(2)*q(2) + q(3)*q(3)), 2*(q(1)*q(2) - q(0)*q(3)), 2*(q(1)*q(3) + q(0)*q(2)),
         2*(q(1)*q(2) + q(0)*q(3)), 1 - 2*(q(1)*q(1) + q(3)*q(3)), 2*(q(2)*q(3) - q(0)*q(1)),
         2*(q(1)*q(3) + q(0)*q(2)), 2*(q(2)*q(3) + q(0)*q(1)), 1 - 2*(q(1)*q(1) + q(2)*q(2));
    return R;
  }

  static const EulerAngle quaternionToEuler(const Quat& q) {
      EulerAngle euler;
      // Roll (phi)
      euler(0) = std::atan2(2.0 * (q(0) * q(1) + q(2) * q(3)), 1.0 - 2.0 * (q(1) * q(1) + q(2) * q(2)));
      // Pitch (theta)
      euler(1) = std::asin(std::clamp(2.0 * (q(0) * q(2) - q(3) * q(1)), -1.0, 1.0));
      // Yaw (psi)
      euler(2) = std::atan2(2.0 * (q(0) * q(3) + q(1) * q(2)), 1.0 - 2.0 * (q(2) * q(2) + q(3) * q(3)));
      return euler;
  }

};

template <typename T>
class Dynamics {
public:
using StateVector = Eigen::Matrix<T, 13, 1>;
using ControlVector = Eigen::Matrix<T, 4, 1>;
using Vector3 = Eigen::Matrix<T, 3, 1>;
using Matrix3 = Eigen::Matrix<T, 3, 3>;
using ControlMatrix = Eigen::Matrix<T, 13, 4>;
using Quat = typename Quaternion<T>::Quat;
public:
  Dynamics() {
    static_assert(std::is_floating_point_v<T>, "Type must be float or double");
  }

  ~Dynamics() = default;

  StateVector F(const StateVector& x) {
    StateVector x_dot = StateVector::Zero();
    // Segment the state vector
    auto v = x.template segment<3>(3);
    auto q = x.template segment<4>(6);
    auto w = x.template segment<3>(10);
    // Set the angular velocity quaternion
    Quat w_q = Quat::Zero();
    w_q.template segment<3>(1) = w;
    Matrix3 J = params_->I.inverse() * skewSymmetricMatrix(w) * params_->I;
    // Compute state derivative
    x_dot.template segment<3>(0) = v;
    x_dot.template segment<3>(3) = -params_->A * v;
    x_dot.template segment<4>(6) = 0.5 * Quaternion<T>::Rq(w_q) * q;
    x_dot.template segment<3>(10) = -J * w;
    // Normalize the quaternion
    x_dot.template segment<4>(6).normalize();

    return x_dot;
  }

  ControlMatrix G(const StateVector& x) {
    ControlMatrix G = ControlMatrix::Zero();
    Quat q = x.template segment<4>(6);
    G.template block<3, 3>(3, 0) = Quaternion<T>::R(q) * params_->Gt;
    G.template block<3, 3>(10, 0) = params_->Gm;
    return G;
  }

  StateVector F_sys(const StateVector& x, const ControlVector& u) {
    auto x_dot = F(x);
    x_dot.template segment<3>(3) += getForce(x, u)/params_->m;
    x_dot.template segment<3>(10) += params_->I.inverse() * getMoment(x, u);
    return x_dot;
  }

private:

  Matrix3 skewSymmetricMatrix(const Vector3& v) const {
    Matrix3 V;
    V << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
    return V;
  }

  Vector3 getForce(const StateVector& x, const ControlVector& u) const {
    auto q = x.template segment<4>(6);
    return Quaternion<T>::R(q) * params_->Gt * u + params_->m * params_->g_v;
  }

  Vector3 getMoment(const StateVector& x, const ControlVector& u) const {
    return params_->Gm * u;
  }

  std::unique_ptr<Parameters<T>> params_{std::make_unique<Parameters<T>>()};
};

template <typename T>
class Controller {
  using StateVector = typename Drone::Dynamics<T>::StateVector;
  using ControlVector = typename Drone::Dynamics<T>::ControlVector;
public:
  Controller() = default;
  ~Controller() = default;

  ControlVector compute(const StateVector& x, const StateVector& x_ref) {
    ControlVector u_hover;
    u_hover << g/4, g/4, g/4, g/4;
    u_hover *= params_->m;
    return u_hover;
  }

private:
  std::unique_ptr<Parameters<T>> params_{std::make_unique<Parameters<T>>()};

};

template <typename T>
class Solver {
  using StateVector = typename Drone::Dynamics<T>::StateVector;
  using ControlVector = typename Drone::Dynamics<T>::ControlVector;
public:
  Solver(std::shared_ptr<Drone::Dynamics<T>> dynamics, T dt)
      : dynamics_(dynamics), dt_(dt) {}

  ~Solver() = default;

  // Integrates the dynamics for one step
  StateVector step(const StateVector& x, const ControlVector& u) {
    return integrateRK4(x, u);
  }

private:
  // Runge-Kutta 4th Order Integration
  StateVector integrateRK4(const StateVector& x, const ControlVector& u) {
    const auto k1 = dynamics_->F_sys(x, u);
    const auto k2 = dynamics_->F_sys(x + 0.5 * dt_ * k1, u);
    const auto k3 = dynamics_->F_sys(x + 0.5 * dt_ * k2, u);
    const auto k4 = dynamics_->F_sys(x + dt_ * k3, u);

    StateVector x_result = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    x_result.template segment<4>(6).normalize();

    return x_result;
  }

  std::shared_ptr<Drone::Dynamics<T>> dynamics_;
  T dt_;  // Time step
};

}


class CSVLogger {
public:
    CSVLogger(const std::string& filename) : file_(filename) {
        if (!file_.is_open()) {
            throw std::runtime_error("Unable to open file for logging.");
        }
        writeHeader();
    }

    ~CSVLogger() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    template <typename T>
    void log(double time, const Eigen::Matrix<T, 12, 1>& state) {
        file_ << std::fixed << std::setprecision(5) << time << ",";
        for (int i = 0; i < state.size(); ++i) {
            file_ << state(i) << (i == state.size() - 1 ? "\n" : ",");
        }
    }

private:
    std::ofstream file_;

    void writeHeader() {
        file_ << "Time,"
              << "Position_X,Position_Y,Position_Z,"
              << "Velocity_X,Velocity_Y,Velocity_Z,"
              << "Roll,Pitch,Yaw,"
              << "Angular_Velocity_X,Angular_Velocity_Y,Angular_Velocity_Z\n";
    }
};


int main() {
    using T = double;

    // Initialize dynamics and solver
    auto dynamics = std::make_shared<Drone::Dynamics<T>>();
    auto params = std::make_shared<Drone::Parameters<T>>();
    auto control = std::make_shared<Drone::Controller<T>>();
    Drone::Solver<T> solver(dynamics, 0.01);  // Time step of 0.01 seconds

    // Open CSV Logger
    CSVLogger logger("data/drone_simulation.csv");

    // Initial state and control inputs
    typename Drone::Dynamics<T>::StateVector x = Drone::Dynamics<T>::StateVector::Zero();
    x.template segment<3>(3) << 0.1, 0, 0;
    x.template segment<4>(6) << 1, 0, 0, 0;  // Initial quaternion

    // Simulate for 100 timesteps
    double time = 0.0;
    double dt = 0.01;

    for (int i = 0; i < 5000; ++i) {
        Eigen::Matrix<T, 12, 1> x_save = Eigen::Matrix<T, 12, 1>::Zero();
        x_save.segment<6>(0) = x.template segment<6>(0);
        x_save.segment<3>(9) = x.template segment<3>(9);
        x_save.segment<3>(6) = Drone::Quaternion<T>::quaternionToEuler(x.template segment<4>(6));
        logger.log(time, x_save);  // Log current state
        x = solver.step(x, control->compute(x, Drone::Dynamics<T>::StateVector::Zero()));  // Integrate one step
        time += dt;
    }

    return 0;
}