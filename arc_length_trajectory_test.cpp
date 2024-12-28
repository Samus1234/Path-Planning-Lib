#include "numerical_integrator.hpp"
#include "trajectory_utils.hpp"
#include "linear_mpc.hpp"

#include <fstream>
#include <iomanip>

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
        Eigen::VectorXf x_forward = x;
        Eigen::VectorXf x_backward = x;

        x_forward(i) += epsilon;
        x_backward(i) -= epsilon;

        Eigen::VectorXf f_forward = func(x_forward);
        Eigen::VectorXf f_backward = func(x_backward);

        J.col(i) = (f_forward - f_backward) / (2 * epsilon);
    }

    return J;
}


Eigen::Vector3f pRef(float t) {
    Eigen::Vector3f p;
    float w = 2*M_PI/20;
    float v_z = 0.1f;
    p << cos(w*t), sin(w*t), v_z*t;
    return p;
}

Eigen::Vector3f vRef(float t) {
    Eigen::Vector3f p;
    float w = 2*M_PI/20;
    float v_z = 0.1f;
    p << -w*sin(w*t), w*cos(w*t), v_z;
    return p;
}

void testTrajectory() {
    auto position_reference = [] (float t) {return pRef(t);};
    auto velocity_reference = [] (float t) {return vRef(t);};

    float t = 0.0f;
    float delta_t = 0.1f;

    TrajectoryUtils::ArcLengthParameterization<Eigen::Vector3f> arc_length_traj(position_reference, velocity_reference);

    arc_length_traj.setStepSize(delta_t);

    arc_length_traj.tabulateArcLength(80);

    for (int i = 0; i < 100; ++i) {
        t += 1e-5f;
        Eigen::Vector3f robot_pos = pRef(t);
        float s_nearest = arc_length_traj.findClosest(robot_pos);
        std::cout << "Robot Position: " << robot_pos.transpose() << " with t: " << t << " and predicted t: " << arc_length_traj.t(s_nearest) << " and predicted s: " << s_nearest << std::endl;
    }
}

void testSystem() {
    System sys;

    auto f1 = [sys](const Eigen::VectorXf& x) { return sys.f_d(x, System::ControlVector::Zero()); };
    auto f2 = [sys](const Eigen::VectorXf& u) { return sys.f_d(System::StateVector::Zero(), u); };

    Eigen::MatrixXf Ad = computeJacobian(f1, System::StateVector::Zero());
    Eigen::MatrixXf Bd = computeJacobian(f2, System::ControlVector::Zero());

    MPC::LQR lqr(Ad, Bd);

    auto position_reference = [](float t) { return pRef(t); };
    auto velocity_reference = [](float t) { return vRef(t); };

    float delta_t = 0.01f;

    TrajectoryUtils::ArcLengthParameterization<Eigen::Vector3f> arc_length_traj(position_reference, velocity_reference);

    arc_length_traj.setStepSize(1e-3);
    arc_length_traj.tabulateArcLength(80);

    std::cout << "Max Trajectory Length: " << arc_length_traj.getMaxArcLength() << " and Max Trajectory Time: " << arc_length_traj.getMaxTime() << std::endl;

    System::StateVector x = System::StateVector::Zero();
    System::StateVector x_ref = System::StateVector::Zero();

    float v_desired = 4.0f;

    std::ofstream output_file("data/trajectory_data.csv");
    output_file << "time,s,x,y,z,vx,vy,vz,x_ref,y_ref,z_ref,vx_ref,vy_ref,vz_ref,u1,u2,u3,e\n";

    for (int i = 0; i < 10000; ++i) {
        float s_nearest = arc_length_traj.findClosest(x.segment<3>(0));
        s_nearest += v_desired * delta_t;
        if (true) {
            x_ref.segment<3>(0) = arc_length_traj.positionReference(s_nearest);
            x_ref.segment<3>(3) = arc_length_traj.velocityReference(s_nearest);
        } else {
            x_ref.segment<3>(0) = pRef(i*delta_t);
            x_ref.segment<3>(3) = vRef(i*delta_t);
        }
        
        System::StateVector delta_x = x_ref - x;
        System::ControlVector u = lqr.computeControl(-delta_x);
        x = sys.f_d(x, u);
        float e = arc_length_traj.getMinDistance();
        // Save data to file
        output_file << std::fixed << std::setprecision(6)
                    << i * delta_t << ","
                    << s_nearest - v_desired*delta_t << ","
                    << x(0) << "," << x(1) << "," << x(2) << ","
                    << x(3) << "," << x(4) << "," << x(5) << ","
                    << x_ref(0) << "," << x_ref(1) << "," << x_ref(2) << ","
                    << x_ref(3) << "," << x_ref(4) << "," << x_ref(5) << ","
                    << u(0) << "," << u(1) << "," << u(2) << "," << e << "\n";
    }

    output_file.close();
    std::cout << "Trajectory data saved to trajectory_data.csv" << std::endl;
}

int main(int argc, char* argv[]) {

    testSystem();

    return 0;
}