#include "numerical_integrator.hpp"
#include "trajectory_utils.hpp"
#include "linear_mpc.hpp"

class System {
public:
    using StateVector = Eigen::Matrix<float, 6, 1>;
    using ControlVector = Eigen::Matrix<float, 3, 1>;
    using StateMatrix = Eigen::Matrix<float, 6, 6>;
    using ControlMatrix = Eigen::Matrix<float, 6, 3>;
public:
    System() {
        A_.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();
        A_.block<3, 3>(3, 3) = -1*Eigen::Matrix3f::Identity();
        B_.block<3, 3>(3, 0) = Eigen::Matrix3f::Identity();
    }
    ~System() = default;

    System(StateMatrix A, ControlMatrix B, float Ts) : A_(A), B_(B), Ts_(Ts) {
    }

    StateVector f(const StateVector& x, const ControlVector& u) const {
        return A_*x + B_*u;
    }

    StateVector f_d(const StateVector& x, const ControlVector& u) const {
        StateVector K1 = Ts_*f(x, u);
        StateVector K2 = Ts_*f(x + K1/2, u);
        StateVector K3 = Ts_*f(x + K2/2, u);
        StateVector K4 = Ts_*f(x + K3, u);

        return x + (K1 + K2*2 + K3*2 + K4)/6;
    }

    const StateMatrix& getA() const {
        return A_;
    }

    const ControlMatrix& getB() const {
        return B_;
    }

private:
    StateMatrix A_{StateMatrix::Zero()};
    ControlMatrix B_{ControlMatrix::Zero()};
    float Ts_{1e-2};
};

Eigen::MatrixXf computeJacobian(
    const std::function<Eigen::VectorXf(const Eigen::VectorXf&)>& func,
    const Eigen::VectorXf& x,
    float epsilon = 1e-5f) {

    int input_dim = x.size();
    Eigen::VectorXf f_x = func(x);
    int output_dim = f_x.size();

    Eigen::MatrixXf J(output_dim, input_dim);

    for (int i = 0; i < input_dim; ++i) {
        Eigen::VectorXf x_forward = x;  // Use VectorXf
        Eigen::VectorXf x_backward = x; // Use VectorXf

        x_forward(i) += epsilon;
        x_backward(i) -= epsilon;

        Eigen::VectorXf f_forward = func(x_forward);  // Use VectorXf
        Eigen::VectorXf f_backward = func(x_backward); // Use VectorXf

        J.col(i) = (f_forward - f_backward) / (2 * epsilon);
    }

    return J;
}


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

void testTrajectory() {
    auto position_reference = [] (float t) {return pRef(t);};
    auto velocity_reference = [] (float t) {return vRef(t);};

    float s_robot = 0.0f;
    float delta_t = 0.01f;

    TrajectoryUtils::ArcLengthParameterization<Eigen::Vector3f> arc_length_traj(position_reference, velocity_reference);

    arc_length_traj.setStepSize(delta_t);

    arc_length_traj.tabulateArcLength(4*M_PI);

    for (int i = 0; i < 100; ++i) {
        s_robot += 0.05f;  // Simulated arc-length progress
        Eigen::Vector3f robot_pos = arc_length_traj.positionReference(s_robot);
        std::cout << "Robot Position: " << robot_pos.transpose() << " with s: " << s_robot << " and predicted s: " << arc_length_traj.findClosest(robot_pos) << std::endl;
    }
}

void testSystem() {
    System sys;

    auto f1 = [sys] (const Eigen::VectorXf& x) {return sys.f_d(x, System::ControlVector::Zero()); };
    auto f2 = [sys] (const Eigen::VectorXf& u) {return sys.f_d(System::StateVector::Zero(), u); };

    Eigen::MatrixXf Ad = computeJacobian(f1, System::StateVector::Zero());
    Eigen::MatrixXf Bd = computeJacobian(f2, System::ControlVector::Zero());

    MPC::LQR lqr(Ad, Bd);

    auto position_reference = [] (float t) {return pRef(t);};
    auto velocity_reference = [] (float t) {return vRef(t);};

    float delta_t = 0.01f;

    TrajectoryUtils::ArcLengthParameterization<Eigen::Vector3f> arc_length_traj(position_reference, velocity_reference);

    arc_length_traj.setStepSize(delta_t);

    arc_length_traj.tabulateArcLength(4*M_PI);

    System::StateVector x;
    x << 0, 0, 0, 0, 0, 0; // Initial state: zero position, unit velocity
    System::StateVector x_ref;
    float v_desired = 1.0f;

    for (int i = 0; i < 200; ++i) {
        float s_nearest = arc_length_traj.findClosest(x.segment<3>(0));
        s_nearest += v_desired*delta_t;
        x_ref.segment<3>(0) = arc_length_traj.positionReference(s_nearest);
        x_ref.segment<3>(3) = arc_length_traj.velocityReference(s_nearest);
        System::ControlVector u = lqr.computeControl(x - x_ref);
        x = sys.f_d(x, u);
        std::cout << "Step " << i << ": " << x.transpose() << std::endl;
    }
}

int main(int argc, char* argv[]) {

    testSystem();

    return 0;
}