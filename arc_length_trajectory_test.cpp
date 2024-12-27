#include "numerical_integrator.hpp"
#include "trajectory_utils.hpp"


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

void test() {
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
        std::cout << "Robot Position: " << robot_pos.transpose() << std::endl;
    }
}

int main(int argc, char* argv[]) {

    test();

    return 0;
}