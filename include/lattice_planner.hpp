#ifndef _LATTICE_PLANNER_H_
#define _LATTICE_PLANNER_H_

#include <vector>
#include <queue>
#include <tuple>
#include <unordered_set>
#include <unordered_map>
#include <Eigen/Dense>
#include <algorithm>

template <typename T>
struct MatrixHash {
    std::size_t operator()(const T& matrix) const {
        std::size_t seed = 0;
        for (int i = 0; i < matrix.size(); ++i) {
            seed ^= std::hash<typename T::Scalar>()(matrix(i)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

namespace LatticePlanner {

using State = Eigen::Vector3d;
using Control = Eigen::Vector2d;

class Planner {
public:
    Planner(double resolution, double delta_max, double L) :
        resolution_(resolution), delta_max_(delta_max), L_(L) {
    }

    ~Planner() = default;

    const std::vector<std::pair<Control, double>>& getMotionPrimitives () const {
        return motion_primitives_;
    }

    const std::vector<State>& getLatticeNodes () const {
        return lattice_nodes_;
    }

    const std::vector<std::tuple<int, int, double>>& getLatticeEdges () const {
        return lattice_edges_;
    }

    void generateLattice(const State& x_init, int depth) {
        std::queue<std::tuple<State, int, int>> search_queue;
        std::unordered_set<State, MatrixHash<State>> visited_states;

        auto num_primitives = motion_primitives_.size();
        auto reserved_size = num_primitives * depth;
        
        lattice_nodes_.reserve(reserved_size);
        lattice_edges_.reserve(reserved_size);

        lattice_nodes_.push_back(x_init);
        search_queue.push({x_init, 0, 0});
        visited_states.emplace(x_init);

        while (!search_queue.empty()) {
            auto [x_current, current_idx, current_depth] = search_queue.front();
            search_queue.pop();
            if (current_depth >= depth) {
                continue;
            }
            for (const auto& [u, cost] : motion_primitives_) {
                auto x_next = dynamics(x_current, u, 1e-1);
                auto [emplaced_it, was_emplaced] = visited_states.emplace(x_next);
                if (was_emplaced) {
                    lattice_nodes_.push_back(x_next);
                    int next_idx = lattice_nodes_.size() - 1;
                    lattice_edges_.push_back({current_idx, next_idx, cost});
                    search_queue.push({x_next, next_idx, current_depth+1});
                }
            }
        }
    }

    void generateMotionPrimitives() {
        motion_primitives_.clear();
        
        std::vector<double> velocities = {-1, -0.5, 0.5, 1};
        std::vector<double> steering_angles = {-M_PI/6, -M_PI/12, 0, M_PI/12, M_PI/6};
        int num_primitives = velocities.size()*steering_angles.size();
        motion_primitives_.reserve(num_primitives);

        for (const auto& velocity : velocities) {
            for (const auto& steering_angle : steering_angles) {
                if (std::abs(steering_angle) <= delta_max_) {
                    Control u(velocity, steering_angle);
                    double cost = std::abs(velocity) + std::abs(steering_angle);
                    motion_primitives_.push_back({u, cost});
                }
            }
        }
    }

    State dynamics(const State& x, const Control& u, double delta_t) {
        State x_next = x;
        double epsilon = 1e-2;
        double v = u(0);
        double delta = std::max(std::min(u(1), delta_max_), -delta_max_);
        if (std::abs(delta) > epsilon) {
            double R = L_ / tan(delta);
            x_next(2) = x(2) + delta_t * v / R;
            x_next(0) = x(0) + R * (sin(x_next(2)) - sin(x(2)));
            x_next(1) = x(1) - R * (cos(x_next(2)) - cos(x(2)));
        } else {
            x_next(0) = x(0) + delta_t * v * cos(x(2));
            x_next(1) = x(1) + delta_t * v * sin(x(2));
        }
        x_next(2) = std::fmod(x_next(2) + 2 * M_PI, 2 * M_PI);
        if (x_next(2) > M_PI) {
             x_next(2) -= 2 * M_PI;
        }
        return x_next;
    }

private:
    double resolution_;
    double delta_max_;
    double L_;

    std::vector<State> lattice_nodes_;
    std::vector<std::pair<Control, double>> motion_primitives_;
    std::vector<std::tuple<int, int, double>> lattice_edges_;
};

}

#endif /* _LATTICE_PLANNER_H_ */