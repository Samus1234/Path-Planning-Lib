#ifndef _DAG_ASTAR_H_
#define _DAG_ASTAR_H_

#include <vector>
#include <unordered_map>
#include <queue>
#include <stack>
#include <tuple>
#include <functional>
#include <limits>
#include <memory>

namespace AStar {

template <typename State, typename CostType>
class DAG {
    static State quantizeState(const State& x, const State& res) {
        State rounded_state = Eigen::round(x.array() / res.array());
        State x_q = res.array() * rounded_state.array();
        return x_q;
    }
public:
    using StateVectorType = std::vector<State>;
    // Tuple: parent node index, child node index, primitive index, step cost
    using EdgeVectorType = std::vector<std::tuple<int, int, int, CostType>>;
    // Tuple: child node index, primitive index, step cost
    using NeighborMapType = std::unordered_map<int, std::vector<std::tuple<int, int, CostType>>>;

    DAG(const StateVectorType& nodes, const EdgeVectorType& edges, const State& resolution)
    : nodes_(nodes), edges_(edges), resolution_(resolution) {
        buildNeighborMap();
    }

    ~DAG() = default;

    const NeighborMapType& getNeighborMap() const {
        return neighbor_map_;
    }

    const StateVectorType& getNodes() const {
        return nodes_;
    }

    const EdgeVectorType& getEdges() const {
        return edges_;
    }

    size_t size() const {
        return nodes_.size();
    }

    int getNodeIndex(const State& x) const {
        auto x_q = quantizeState(x, resolution_);
        auto it = std::find(nodes_.begin(), nodes_.end(), x_q);
        if (it != nodes_.end()) {
            return std::distance(nodes_.begin(), it);
        }
        return -1;
    }

private:

    void buildNeighborMap() {
        for (const auto& [parent, child, primitive, step_cost] : edges_) {
            neighbor_map_[parent].emplace_back(child, primitive, step_cost);
        }
    }

    StateVectorType nodes_;
    EdgeVectorType edges_;
    State resolution_;
    NeighborMapType neighbor_map_;
};


template <typename State, typename CostType>
class AStar {
    static constexpr CostType INF = std::numeric_limits<CostType>::max();
public:
    using PathType = std::vector<std::pair<int, int>>; // state and primitive
    using QueueObjectType = std::pair<CostType, int>;

    AStar(DAG<State, CostType>& graph,
          const std::function<CostType(const State&, const State&)>& heuristic)
        : graph_(graph), heuristic_(heuristic) {
        initialize();
    }

    ~AStar() = default;


    void setStartAndGoalNodes(const State& start_node, const State& goal_node) {
        start_node_idx_ = graph_.getNodeIndex(start_node);
        goal_node_idx_ = graph_.getNodeIndex(goal_node);

        CostType start_cost = heuristic_(start_node, goal_node);
        optimal_path_map_[start_node_idx_] = std::make_pair(0, start_cost);
        queue_.emplace(start_cost, start_node_idx_);
    }

    void initialize() {
        for (size_t i = 0; i < graph_.size(); i++) {
            optimal_path_map_[i] = std::make_pair(INF, INF);
        }
    }


    void runOnce() {
        if (queue_.empty()) {
            complete_ = true;
            return;
        }

        auto [current_f_cost, current_node_idx] = queue_.top();
        queue_.pop();

        if (current_node_idx == goal_node_idx_) {
            complete_ = true;
            return;
        }
        
        if (current_f_cost > optimal_path_map_[current_node_idx].second) {
            return;
        }

        auto nodes = graph_.getNodes();
        auto neighbor_map = graph_.getNeighborMap();

        for (const auto& [neighbor_idx, primitive_idx, step_cost] : neighbor_map[current_node_idx]) {
            CostType h_cost = heuristic_(nodes[neighbor_idx], nodes[goal_node_idx_]);
            CostType g_cost = step_cost + optimal_path_map_[current_node_idx].first;
            CostType f_cost = g_cost + h_cost;
            if (f_cost < optimal_path_map_[neighbor_idx].second) {
                optimal_path_map_[neighbor_idx] = {g_cost, f_cost};
                queue_.emplace(f_cost, neighbor_idx);
                predecessor_map_[neighbor_idx] = std::make_pair(current_node_idx, primitive_idx);
            }
        }
    }

    void run() {
        while (!complete_) {
            runOnce();
        }
    }



    PathType findPath(const State& start_node, const State& goal_node) {
        setStartAndGoalNodes(start_node, goal_node);
        run();
        std::stack<std::pair<int, int>> path_stack;
        if (optimal_path_map_[goal_node_idx_].first == INF) {
            throw std::runtime_error("Path to goal does not exist!");
        }

        auto node = goal_node_idx_;

        path_stack.push(std::make_pair(node, -1));

        while (node != start_node_idx_) {
            auto [prev_node, primitive] = predecessor_map_[node];
            node = prev_node;
            path_stack.push(std::make_pair(node, primitive));
        }

        PathType optimal_path;

        while (!path_stack.empty()) {
            optimal_path.emplace_back(path_stack.top());
            path_stack.pop();
        }
        
        return optimal_path;
    }

private:

    DAG<State, CostType> graph_;
    std::function<CostType(const State&, const State&)> heuristic_;

    int start_node_idx_;
    int goal_node_idx_;

    bool complete_{false};

    std::priority_queue<QueueObjectType, std::vector<QueueObjectType>, std::greater<QueueObjectType>> queue_;
    std::unordered_map<int, std::pair<CostType, CostType>> optimal_path_map_;
    std::unordered_map<int, std::pair<int, int>> predecessor_map_;
};

} // End namespace AStar

#endif /* _DAG_ASTAR_H_ */