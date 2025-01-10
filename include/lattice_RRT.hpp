#ifndef _LATTICE_RRT_H_
#define _LATTICE_RRT_H_

#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <algorithm>
#include "dag.hpp"
#include "lattice_planner.hpp"

struct CircularObstacle {
    Eigen::Vector2d center;
    double radius;
};

template <typename StateType, typename ControlType, typename CostType>
class RRT {
    using GraphType = DAG<StateType, CostType>;
    using MotionPrimitiveType = std::vector<std::pair<ControlType, CostType>>;
    using DynamicsFunctionType = std::function<StateType(StateType, ControlType, double)>;
public:
    RRT() =  default;
    ~RRT() = default;

    RRT(double max_distance, double bounds, double search_radius)
    : max_distance_(max_distance), bounds_(bounds), search_radius_(search_radius) {

    }

    const GraphType& getGraph() const {
        return graph_;
    }
    
    const std::vector<CostType>& getEdgeCosts() const {
        return edge_costs_;
    }

    void setGraph(const GraphType& graph) {
        graph_ = graph;
    }

    void setMotionPrimitives(const MotionPrimitiveType& motion_primitives) {
        motion_primitives_ = motion_primitives;
    }

    void setInitialNode(const StateType& initial) {
        if (graph_.empty()) {
            graph_.addNode(initial);
            edge_costs_.emplace_back(0);
        } else {
            std::cout << "Graph not empty!" << std::endl;
        }
    }

    void setGoalNode(const StateType& goal) {
        goal_ = goal;
    }

    void setWeights(const StateType& squared_weights) {
        weights_ = squared_weights.cwiseSqrt();
    }

    void setDynamics(const DynamicsFunctionType& dynamics) {
        dynamics_ = dynamics;
    }

    void setObstacles(const std::vector<CircularObstacle>& obstacles) {
        obstacles_ = obstacles;
    }

    void iterate(size_t num_iters) {
        for (size_t i = 0; i < num_iters; i++) {
            StateType sampled_point = StateType::Random();
            sampled_point(0) = (10*sampled_point(0) + 0)/2;
            sampled_point(1) = (10*sampled_point(1) + 0)/2;
            sampled_point(2) = M_PI*sampled_point(2);
            addNewNode(sampled_point);
            if (complete_) {
                return;
            }
        }
    }

private:
    bool checkCollision(const StateType& a, const StateType& b) {
        for (double alpha = 0; alpha <= 1; alpha += 0.1) {
            auto c = alpha * b + (1 - alpha) * a;
            for (const auto& obstacles_ : obstacles_) {
                double distance = std::sqrt(
                    (c(0) - obstacles_.center(0))*(c(0) - obstacles_.center(0)) + 
                    (c(1) - obstacles_.center(1)) * (c(1) - obstacles_.center(1))
                );
                if (distance <= obstacles_.radius) {
                    return true;
                }
            }
        }
        return false;
    }

    void addNewNode(const StateType& sampled_point) {
        const auto& nodes = graph_.getNodes();
        int closest_idx = 0;
        double closest_distance = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(nodes.size()); i++) {
            auto delta = (sampled_point - nodes[i]);
            auto d = (delta.array() * weights_.array()).matrix().norm();
            if (d < closest_distance) {
                closest_distance = d;
                closest_idx = i;
            }
        }

        auto current_edge_cost = edge_costs_.back();
        auto closest_node = nodes[closest_idx];
        double closest_new_distance = std::numeric_limits<double>::max();
        double best_primitive_idx = 0;
        for (size_t i = 0; i < motion_primitives_.size(); i++) {
            const auto& [u, cost] = motion_primitives_[i];
            auto new_node = dynamics_(closest_node, u, dt_);
            auto d = (sampled_point - new_node).norm();
            if (d < closest_new_distance) {
                closest_new_distance = d;
                best_primitive_idx = i;
            }
        }

        const auto& [u_best, best_cost] = motion_primitives_[best_primitive_idx];
        auto new_node = dynamics_(closest_node, u_best, dt_);

        if (checkCollision(closest_node, new_node)) {
            return;
        }

        auto shortest_rewire_distance = best_cost + current_edge_cost;
        int rewire_node_index = closest_idx;

        if(rewire_) {
            for (int i = 0; i < static_cast<int>(nodes.size()); i++) {
                auto delta = (new_node - nodes[i]);
                auto d = (delta.array() * weights_.array()).matrix().norm();
                if (d < search_radius_) {
                    auto rewired_distance = best_cost + d;
                    if (rewired_distance < shortest_rewire_distance) {
                        shortest_rewire_distance = rewired_distance;
                        rewire_node_index = i;
                    }
                }
            }
        }

        graph_.addNode(new_node);
        graph_.addIndexedEdge(rewire_node_index, static_cast<int>(nodes.size()-1), best_cost);
        edge_costs_.emplace_back(shortest_rewire_distance + current_edge_cost);
    }

    StateType weights_;

    std::vector<CostType> edge_costs_;

    std::vector<CircularObstacle> obstacles_;

    MotionPrimitiveType motion_primitives_;

    std::function<StateType(StateType, ControlType, double)> dynamics_;

    bool complete_{false};

    bool rewire_{true};

    double epsilon_{0.5};

    double dt_{0.1};

    StateType goal_;

    GraphType graph_;
    
    double max_distance_{0.5};
    double bounds_{10.0};
    double search_radius_{2.0};
};

#endif /* _LATTICE_RRT_H_ */