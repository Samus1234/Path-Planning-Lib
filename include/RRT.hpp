#ifndef _RRT_H_
#define _RRT_H_

#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <algorithm>
#include "dag.hpp"

struct CircularObstacle {
    Eigen::Vector2d center;
    double radius;
};

template <typename NodeType, typename CostType>
class RRT {
    using GraphType = DAG<NodeType, CostType>;
public:
    RRT() =  default;
    ~RRT() = default;

    RRT(double max_distance, double bounds, double search_radius)
    : max_distance_(max_distance), bounds_(bounds), search_radius_(search_radius) {

    }

    void setGraph(const GraphType& graph) {
        graph_ = graph;
    }

    const GraphType& getGraph() const {
        return graph_;
    }
    
    const std::vector<CostType>& getEdgeCosts() const {
        return edge_costs_;
    }

    void setObstacles(const std::vector<CircularObstacle>& obstacles) {
        obstacles_ = obstacles;
    }

    void setInitialNode(const NodeType& initial) {
        if (graph_.empty()) {
            graph_.addNode(initial);
            edge_costs_.emplace_back(0);
        } else {
            std::cout << "Graph not empty!" << std::endl;
        }
    }

    void setGoalNode(const NodeType& goal) {
        goal_ = goal;
    }

    void iterate(size_t num_iters) {
        for (size_t i = 0; i < num_iters; i++) {
            auto sampled_point = bounds_*NodeType::Random();
            addNewNode(sampled_point);
            if (complete_) {
                return;
            }
        }
    }

private:

    bool checkCollision(const NodeType& a, const NodeType& b) {
        for (double alpha = 0; alpha <= 1; alpha += 0.1) {
            auto c = alpha * b + (1 - alpha) * a;
            for (const auto& obstacles_ : obstacles_) {
                if ((c - obstacles_.center).norm() <= obstacles_.radius) {
                    return true;
                }
            }
        }
        return false;
    }

    void addNewNode(const NodeType& sampled_point) {
        const auto& nodes = graph_.getNodes();
        int closest_idx = 0;
        double closest_distance = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(nodes.size()); i++) {
            auto d = (sampled_point - nodes[i]).norm();
            if (d < closest_distance) {
                closest_distance = d;
                closest_idx = i;
            }
        }

        auto current_edge_cost = edge_costs_.back();
        auto r = std::min(max_distance_, closest_distance);
        auto closest_node = nodes[closest_idx];
        auto direction = (sampled_point - closest_node).normalized();
        auto new_node = closest_node + r*direction;

        if (checkCollision(closest_node, new_node)) {
            return;
        }
        

        auto shortest_rewire_distance = r + current_edge_cost;
        int rewire_node_index = closest_idx;

        if(rewire_) {
            for (int i = 0; i < static_cast<int>(nodes.size()); i++) {
                auto d = (new_node - nodes[i]).norm();
                if (d < search_radius_) {
                    auto rewired_distance = r + d;
                    if (rewired_distance < shortest_rewire_distance) {
                        shortest_rewire_distance = rewired_distance;
                        rewire_node_index = i;
                    }
                }
            }
        }

        graph_.addNode(new_node);
        graph_.addIndexedEdge(rewire_node_index, static_cast<int>(nodes.size()-1), r);
        edge_costs_.emplace_back(shortest_rewire_distance + current_edge_cost);

        auto goal_distance = (new_node - goal_).norm();
    
        if (goal_distance < epsilon_) {
            complete_ = true;
            std::cout << "Goal node found" << std::endl;
        }
    }

    std::vector<CostType> edge_costs_;

    std::vector<CircularObstacle> obstacles_;

    bool complete_{false};

    bool rewire_{true};

    double epsilon_{0.5};

    NodeType goal_;

    GraphType graph_;
    
    double max_distance_{0.5};
    double bounds_{10.0};
    double search_radius_{2.0};
};

#endif /* _RRT_H_ */