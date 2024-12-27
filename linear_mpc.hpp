#ifndef _LINEAR_MPC_H_
#define _LINEAR_MPC_H_

#include <Eigen/Dense>
#include <iostream>

namespace MPC {

class LQR {
public:
    LQR() = default;
    ~LQR() = default;

    LQR(Eigen::MatrixXf A, Eigen::MatrixXf B) {
        num_states_ = A.cols();
        num_controls_ = B.cols();
        A_ = A;
        buildMatrices(A, B);
    }

    void buildMatrices(Eigen::MatrixXf A, Eigen::MatrixXf B) {
        // I am simply assuming Identity matrices for Q and R
        // I am also hardcoding the horizon for now
        Eigen::MatrixXf I = Eigen::MatrixXf::Identity(num_states_, num_states_);
        horizon_ = 100;
        size_t n = horizon_*(num_states_ + num_controls_);
        size_t m = horizon_*num_states_;
        H_ = Eigen::MatrixXf::Identity(n, n);
        C_ = Eigen::MatrixXf::Zero(m, n);
        KKT_ = Eigen::MatrixXf::Zero(m+n, m+n);
        for (size_t i = 0; i < horizon_; ++i) {
            size_t row_offset = i * num_states_;
            size_t col_offset = i * (num_states_ + num_controls_);
            C_.block(row_offset, col_offset, num_states_, num_controls_) = B;
            if (i > 0) {
                C_.block(row_offset, col_offset - num_states_, num_states_, num_states_) = -A;
            }
            C_.block(row_offset, col_offset + num_controls_, num_states_, num_states_) = -I;
            H_.block(col_offset, col_offset, num_states_, num_states_) = 100*I;
        }
        KKT_.block(0, 0, n, n) = H_;
        KKT_.block(0, n, n, m) = C_.transpose();
        KKT_.block(n, 0, m, n) = C_;
        KKT_Inv_ = KKT_.inverse();
    }

    Eigen::VectorXf computeControl(Eigen::VectorXf x) {
        size_t n = horizon_*(num_states_ + num_controls_);
        size_t m = horizon_*num_states_;
        Eigen::VectorXf d = Eigen::VectorXf::Zero(n+m);
        d.block(n, 0, num_states_, 1) = -A_ * x;

        z_ = KKT_Inv_ * d;

        return z_.block(0, 0, num_controls_, 1);
    }

private:
    Eigen::MatrixXf H_;
    Eigen::MatrixXf C_;
    Eigen::MatrixXf A_;
    Eigen::MatrixXf KKT_;
    Eigen::MatrixXf KKT_Inv_;
    Eigen::MatrixXf z_;
    size_t num_states_;
    size_t num_controls_;
    size_t horizon_;
};


}

#endif /* _TRAJECTORY_UTILS_H_ */