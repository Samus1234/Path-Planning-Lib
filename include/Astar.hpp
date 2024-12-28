#ifndef _ASTAR_H_
#define _ASTAR_H_

#include "path_planning_utils.hpp"
#include <memory>
#include <limits>

template<typename Graph>
class AStar {
    using ValueType = typename Graph::ValueType;
    using NodeType = typename Graph::NodeType;
    using NodePtr = typename Graph::NodePtr;
    using Iterator = typename Graph::Iterator;
    using CostType = float;
    using NodeQueueType = typename std::pair<CostType, NodePtr>;
    static constexpr CostType INF = std::numeric_limits<CostType>::max();
public:
    AStar() = default;
    ~AStar() = default;

    AStar(typename Graph::SharedPtr graph) { setGraph(graph); }

    void setGraph(typename Graph::SharedPtr graph) {
        graph_ = graph;
        initialize();
    }

    CostType heuristicCost(const ValueType& cell_a, const ValueType& cell_b) {
        const char TYPE = 'C';
        CostType dx = cell_b.x - cell_a.x;
        CostType dy = cell_b.y - cell_a.y;
        switch (TYPE) {
        case 'E':
            return std::sqrt(dx*dx + dy*dy);
            break;

        case 'M':
            return std::abs(dx) + std::abs(dy);
            break;

        case 'C':
            return std::max(std::abs(dx), std::abs(dy));
            break;
        
        default:
            throw std::runtime_error("Invalid Measurement norm: " + std::string(1, TYPE));
            break;
        }
    }

    void initialize() {
        for (auto it = graph_->begin(); it != graph_->end(); ++it) {
            shortest_path_map_[it.getNode()] = std::make_pair(INF, INF);
        }
    }

    void setStartAndGoalNodes(const ValueType& start, const ValueType& goal) {
        start_node_ = graph_->getNode(start);
        goal_node_ = graph_->getNode(goal);
        CostType start_cost = heuristicCost(start_node_->data_, goal_node_->data_);
        shortest_path_map_[start_node_].first = 0;
        shortest_path_map_[start_node_].second = start_cost;
        queue_.emplace(start_cost, start_node_);
    }

    void runOnce() {
        if (queue_.empty()) {
            complete_ = true;
            return;
        }
        
        auto [current_g_cost, current_node] = queue_.top();
        queue_.pop();

        if (current_node == goal_node_) {
            complete_ = true;
            return;
        }

        if (current_g_cost > shortest_path_map_[current_node].second) {
            return;
        }
        
        for (const auto& [neighbor, step_cost] : current_node->neighbors_) {
            CostType h_cost = heuristicCost(neighbor->data_, goal_node_->data_);
            CostType tentative_g_cost = (CostType)step_cost + current_g_cost;
            CostType tentative_f_cost =  tentative_g_cost + h_cost;
            if (tentative_f_cost < shortest_path_map_[neighbor].second) {
                shortest_path_map_[neighbor].first = (CostType)tentative_g_cost;
                shortest_path_map_[neighbor].second = tentative_f_cost;
                queue_.emplace(tentative_f_cost, neighbor);
                predecessor_map_[neighbor] = current_node;
            }
        }   
    }

    void run() {
        while(!complete_) {
            runOnce();
        }
    }

    std::stack<ValueType> getPath() {
        if (!complete_) {
            run();
        }

        auto node = goal_node_;

        if (shortest_path_map_[node].second == INF) {
            throw std::runtime_error("Path to goal does not exist!");
        }

        std::stack<ValueType> path_stack;

        path_stack.push(node->data_);

        while (node != start_node_) {
            node = predecessor_map_[node];
            path_stack.push(node->data_);
        }

        return path_stack;
    }



private:
    bool complete_{false};
    NodePtr start_node_{nullptr};
    NodePtr goal_node_{nullptr};
    typename Graph::SharedPtr graph_;
    std::unordered_map<NodePtr, std::pair<CostType, CostType>> shortest_path_map_; // <g_cost, f_cost>
    std::unordered_map<NodePtr, NodePtr> predecessor_map_; // Child -> Parent
    std::priority_queue<NodeQueueType, std::vector<NodeQueueType>, std::greater<NodeQueueType>> queue_;
};

#endif /* _ASTAR_H_ */