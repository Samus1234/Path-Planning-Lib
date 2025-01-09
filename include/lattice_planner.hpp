#ifndef _LATTICE_PLANNER_H_
#define _LATTICE_PLANNER_H_

#include <vector>
#include <queue>
#include <tuple>
#include <unordered_set>
#include <unordered_map>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <fstream>

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
    static State quantizeState(const State& x, const State& res) {
        State rounded_state = Eigen::round(x.array() / res.array());
        State x_q = res.array() * rounded_state.array();
        return x_q;
    }
public:
    Planner(double delta_max, double L, const State& resolution) :
        delta_max_(delta_max), L_(L), resolution_(resolution) {
    }

    ~Planner() = default;

    const std::vector<std::pair<Control, double>>& getMotionPrimitives () const {
        return motion_primitives_;
    }

    const std::vector<State>& getLatticeNodes () const {
        return lattice_nodes_;
    }

    const std::vector<std::tuple<int, int, int, double>>& getLatticeEdges () const {
        return lattice_edges_;
    }

    int getNodeIndex(const State& x_q) const {
        auto it = std::find(lattice_nodes_.begin(), lattice_nodes_.end(), x_q);
        if (it != lattice_nodes_.end()) {
            return std::distance(lattice_nodes_.begin(), it);
        }
        return -1;
    }


    void generateLattice(const State& x_init, int depth) {
        std::queue<std::tuple<State, int, int>> search_queue;
        std::unordered_set<State, MatrixHash<State>> visited_states;

        auto num_primitives = motion_primitives_.size();
        auto reserved_size = std::pow<int>(num_primitives, depth);
        
        lattice_nodes_.reserve(reserved_size);
        lattice_edges_.reserve(reserved_size);

        auto x_init_quantized = quantizeState(x_init, resolution_);

        lattice_nodes_.push_back(x_init_quantized);
        search_queue.push({x_init_quantized, 0, 0});
        visited_states.emplace(x_init_quantized);

        while (!search_queue.empty()) {
            auto [x_current, current_idx, current_depth] = search_queue.front();
            search_queue.pop();
            if (current_depth >= depth) {
                continue;
            }
            size_t control_idx = 0;
            for (const auto& [u, cost] : motion_primitives_) {
                auto x_next = dynamics(x_current, u, 1e-1);
                auto x_next_quantized = quantizeState(x_next, resolution_);
                auto [_, was_emplaced] = visited_states.emplace(x_next_quantized);
                if (was_emplaced) {
                    lattice_nodes_.push_back(x_next_quantized);
                    int next_idx = lattice_nodes_.size() - 1;
                    lattice_edges_.push_back({current_idx, next_idx, control_idx, cost});
                    search_queue.push({x_next_quantized, next_idx, current_depth+1});
                }
                ++control_idx;
            }
        }
    }

    void generateMotionPrimitives() {
        motion_primitives_.clear();
        
        std::vector<double> velocities = {1, 2, 3, 4};
        std::vector<double> steering_angles = {-M_PI/4, -M_PI/8, 0, M_PI/8, M_PI/4};
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

    void saveLatticeToCSV(
        const std::string& node_file,
        const std::string& edge_file,
        const std::string& primitive_file) {
        std::ofstream node_out(node_file);
        node_out << "x,y,theta\n";
        for (const auto& node : lattice_nodes_) {
            node_out << node(0) << "," << node(1) << "," << node(2) << "\n";
        }
        node_out.close();

        std::ofstream edge_out(edge_file);
        edge_out << "parent,child,control,cost\n";
        for (const auto& edge : lattice_edges_) {
            int parent, child, control;
            double cost;
            std::tie(parent, child, control, cost) = edge;
            edge_out << parent << "," << child << "," << control << "," << cost << "\n";
        }
        edge_out.close();

        std::ofstream primitive_out(primitive_file);
        primitive_out << "index,velocity,steering_angle,cost\n";
        int index = 0;
        for (const auto& [control, cost] : motion_primitives_) {
            primitive_out << index++ << "," << control(0) << "," << control(1) << "," << cost << "\n";
        }
        primitive_out.close();
    }

private:
    double delta_max_;
    double L_;

    State resolution_;
    std::vector<State> lattice_nodes_;
    std::vector<std::pair<Control, double>> motion_primitives_;
    std::vector<std::tuple<int, int, int, double>> lattice_edges_;
};

} // namespace LatticePlanner

#endif /* _LATTICE_PLANNER_H_ */