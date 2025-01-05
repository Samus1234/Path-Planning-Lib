#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <iomanip>


template<typename T, size_t degree, size_t order>
class Spline {
private:
    static constexpr size_t N = degree + 1;
    static constexpr size_t K = order;
    static size_t permutations(size_t n, size_t k) {
        if (n < k) {
            return 0;
        }
        size_t result = 1;
        for (size_t i = 0; i < k; i++) {
            result *= (n - i);
        }
        return result;
    }
public:
    using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
    using Array = Eigen::Array<T, Eigen::Dynamic, 1>;
    using WaypointMatrix = Eigen::Matrix<T, 3, Eigen::Dynamic>;
public:
    Spline() = default;
    ~Spline() = default;

    Spline(Array timestamps, WaypointMatrix positions,
        WaypointMatrix velocities) {
        timestamps_ = timestamps;
        waypoints_.push_back(positions);
        waypoints_.push_back(velocities);
        num_constraints_ = 2;
        num_waypoints_ = timestamps_.size();
        time_intervals_ = Array::Zero(num_waypoints_-1);
        for (size_t i = 0; i < num_waypoints_-1; i++) {
            time_intervals_(i) = timestamps(i+1) - timestamps(i);
        }
        coefficients_.resize(3);
        for (auto& coeff : coefficients_) {
            coeff = Vector::Zero((num_waypoints_-1)*N);
        }
        buildPermutationMatrix();
        buildConstraintMatrix();
        buildConstraintVectors();
        buildObjectiveMatrix();
        optimize();
    }

    Spline(Array timestamps, WaypointMatrix positions, 
        WaypointMatrix velocities, WaypointMatrix accelerations) {
        timestamps_ = timestamps;
        waypoints_.push_back(positions);
        waypoints_.push_back(velocities);
        waypoints_.push_back(accelerations);
        num_constraints_ = 3;
        num_waypoints_ = timestamps_.size();
        time_intervals_ = Array::Zero(num_waypoints_-1);
        for (size_t i = 0; i < num_waypoints_-1; i++) {
            time_intervals_(i) = timestamps(i+1) - timestamps(i);
        }
        coefficients_.resize(3);
        for (auto& coeff : coefficients_) {
            coeff = Vector::Zero((num_waypoints_-1)*N);
        }
        buildPermutationMatrix();
        buildConstraintMatrix();
        buildConstraintVectors();
        buildObjectiveMatrix();
        optimize();
    }

    const Matrix& getPermutationMatrix() const {
        return permutation_matrix_;
    }

    const std::vector<Vector>& getCoefficients() const {
        return coefficients_;
    }

    const Matrix& getConstraintMatrices() const {
        return constraint_matrix_;
    }

    const std::vector<Vector>& getConstraintVectors() const {
        return constraint_vectors_;
    }

    Vector trajectory(T t, size_t derivative_degree) const {
        size_t segment_index;
        if (t <= timestamps_(0)) {
            segment_index = 0;
        } else if (t >= timestamps_(num_waypoints_-1)) {
            segment_index = num_waypoints_-2;
        } else {
            auto it = std::lower_bound(timestamps_.begin(), timestamps_.end(), t);
            segment_index = std::max<size_t>(0, std::distance(timestamps_.begin(), it) - 1);
        }
        Vector traj = Vector::Zero(3);
        size_t start_index = segment_index * N;
        size_t dim = 0;
        for (const auto& coefficient_vector : coefficients_) {
            traj(dim) = coefficient_vector.segment(start_index, N).dot(Phi(t - timestamps_[segment_index], derivative_degree));
            dim++;
        }
        return traj;
    }

private:

    Vector Phi(T t, size_t k) const {
        Vector phi = Vector::Zero(N);
        for (size_t i = k; i < N; i++) {
            size_t power = i-k;
            phi(i) = permutations(i, k)*std::pow<T>(t, power);
        }
        return phi;
    }

    Matrix M() const {
        size_t num_segments = num_waypoints_ - 1;
        Matrix m = Matrix::Zero(num_segments * N, num_segments * N);

        for (size_t k = 0; k < num_segments; k++) {
            Matrix m_block = Matrix::Zero(N, N);
            for (size_t i = K; i < N; i++) {
                for (size_t j = K; j < N; j++) {
                    size_t power = i + j - 2 * K + 1;
                    T interval_power = std::pow<T>(time_intervals_(k), power);
                    m_block(i, j) = interval_power / (T)power;
                }
            }
            size_t offset = k * N;
            m.block(offset, offset, N, N) = m_block;
        }
        return m;
    }

    void buildObjectiveMatrix() {
        auto m = M();
        objective_matrix_ = permutation_matrix_ * m * permutation_matrix_;
    }

    void buildPermutationMatrix() {
        size_t num_segments = (num_waypoints_ - 1);
        Matrix permutation_block = Matrix::Zero(N, N);
        for (size_t i = K; i < N; i++) {
            permutation_block(i, i) = permutations(N, i);
        }
        auto I = Matrix::Identity(num_segments, num_segments);
        permutation_matrix_ = Eigen::kroneckerProduct(I, permutation_block);
    }

    void buildConstraintMatrix() {
        size_t num_constraints = 2 * num_constraints_ * (num_waypoints_ - 1);
        size_t num_coeff = N * (num_waypoints_ - 1);
        constraint_matrix_ = Matrix::Zero(num_constraints, num_coeff);

        for (size_t i = 0; i < num_waypoints_ - 1; i++) {
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> block_matrix =
                Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(2 * num_constraints_, N);
            for (size_t j = 0; j < num_constraints_; j++) {
                block_matrix.row(2 * j) = Phi(0, j).transpose();
                block_matrix.row(2 * j + 1) = Phi(time_intervals_(i), j).transpose();
            }
            size_t row_offset = 2 * num_constraints_ * i;
            size_t col_offset = N * i;
            constraint_matrix_.block(row_offset, col_offset, 2 * num_constraints_, N) = block_matrix;
        }
    }

    void buildConstraintVectors() {
        size_t num_constraints = 2 * num_constraints_ * (num_waypoints_ - 1);
        constraint_vectors_.resize(3);
        for (auto& constraint_vector : constraint_vectors_) {
            constraint_vector = Vector::Zero(num_constraints);
        }
        for (size_t dim = 0; dim < 3; dim++) {
            auto& constraint_vector = constraint_vectors_[dim];

            for (size_t i = 0; i < num_waypoints_ - 1; i++) {
                for (size_t j = 0; j < num_constraints_; j++) {
                    size_t row_offset = 2 * num_constraints_ * i + 2 * j;

                    T w_i = waypoints_[j](dim, i);
                    T w_f = waypoints_[j](dim, i + 1);

                    constraint_vector(row_offset) = w_i;
                    constraint_vector(row_offset + 1) = w_f;
                }
            }
        }
    }

    void optimize() {
        size_t num_segments = (num_waypoints_ - 1);
        size_t num_coeff = num_segments*N;
        size_t num_constraints = 2*num_constraints_*num_segments;
        
        Matrix KKT_matrix = Matrix::Zero(num_coeff + num_constraints, num_coeff + num_constraints);

        KKT_matrix.block(0, 0, num_coeff, num_coeff) = objective_matrix_;
        KKT_matrix.block(0, num_coeff, num_coeff, num_constraints) = constraint_matrix_.transpose();
        KKT_matrix.block(num_coeff, 0, num_constraints, num_coeff) = constraint_matrix_;

        Eigen::LDLT<Matrix> ldlt(KKT_matrix);

        for (size_t dim = 0; dim < 3; dim++) {
            Vector rhs = Vector::Zero(num_coeff + num_constraints);
            rhs.tail(num_constraints) = constraint_vectors_[dim];

            Vector solution = ldlt.solve(rhs);

            coefficients_[dim] = solution.head(num_coeff);
        }
    }

    size_t num_constraints_;
    size_t num_waypoints_;

    Array timestamps_;
    Array time_intervals_;

    std::vector<WaypointMatrix> waypoints_;

    std::vector<Vector> coefficients_;

    Matrix objective_matrix_;
    Matrix constraint_matrix_;

    std::vector<Vector> constraint_vectors_;

    Matrix permutation_matrix_;
};


int main(int argc, char** argv) {

    using MySpline = Spline<float, 5, 4>;

    MySpline::Array timestamps(5);
    timestamps << 0, 1, 2, 3, 4;

    MySpline::WaypointMatrix positions(3, 5);
    positions << 
        0, 1, 2, 3, 4,
        0, 1, 0, -1, 0,
        0, 0, 1, 1, 0;

    MySpline::WaypointMatrix velocities(3, 5);
    velocities << 
        1, 1, 1, 1, 0,
        1, 0, -1, 0, 0,
        0, 1, 1, -1, 0;

    MySpline::WaypointMatrix accelerations(3, 5);
    accelerations << 
        0, 0, 0, -1, 0,
        0, -1, 0, 1, 0,
        1, 1, -1, -1, 0;

    MySpline spline(timestamps, positions, velocities, accelerations);

    auto coeffs = spline.getCoefficients();

    for (const auto& coeff : coeffs) {
        std::cout << coeff.transpose() << std::endl;
    }

    // Test cases
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Testing Spline Trajectory:\n";

    for (float t = 0.01; t < 4.0; t += 0.1) {
        auto traj = spline.trajectory(t, 0); // 0th derivative (position)
        auto vel = spline.trajectory(t, 1); // 1st derivative (velocity)
        auto acc = spline.trajectory(t, 2); // 2nd derivative (acceleration)
        std::cout << "Time: " << t << "\n";
        std::cout << "  Position: [" << traj.transpose() << "]\n";
        std::cout << "  Velocity: [" << vel.transpose() << "]\n";
        std::cout << "  Acceleration: [" << acc.transpose() << "]\n";
    }

    // // // Verify constraints
    // // std::cout << "\nVerifying Constraints:\n";
    // // for (size_t i = 0; i < timestamps.size(); ++i) {
    // //     auto pos = spline.trajectory(timestamps(i), 0);
    // //     auto vel = spline.trajectory(timestamps(i), 1);
    // //     auto acc = spline.trajectory(timestamps(i), 2);

    // //     std::cout << "Waypoint " << i << ":\n";
    // //     std::cout << "  Expected Position: [" << positions.col(i).transpose() << "]\n";
    // //     std::cout << "  Actual Position:   [" << pos.transpose() << "]\n";
    // //     std::cout << "  Expected Velocity: [" << velocities.col(i).transpose() << "]\n";
    // //     std::cout << "  Actual Velocity:   [" << vel.transpose() << "]\n";
    // //     std::cout << "  Expected Accel:    [" << accelerations.col(i).transpose() << "]\n";
    // //     std::cout << "  Actual Accel:      [" << acc.transpose() << "]\n";
    // //     std::cout << "\n";
    // // }

    return 0;
}