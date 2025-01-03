#ifndef _DIJKSTRA_H_
#define _DIJKSTRA_H_

#include "graph.hpp"
#include <limits>
#include <memory>

template<typename Graph>
class Dijkstra {
    using ValueType = typename Graph::ValueType;
    using NodeType = typename Graph::NodeType;
    using NodePtr = typename Graph::NodePtr;
    using Iterator = typename Graph::Iterator;
    using CostType = typename Graph::CostType;
    using NodeQueueType = typename std::pair<CostType, NodePtr>;
    static constexpr CostType INF = std::numeric_limits<CostType>::max();
public:
    Dijkstra() = default;
    ~Dijkstra() = default;

    Dijkstra(std::shared_ptr<Graph> graph) { setGraph(graph); }

    bool done() const {
        return node_priority_queue_.empty();
    }

    void setGraph(std::shared_ptr<Graph> graph) {
        graph_ = graph;
        initialize();
    }

    void initialize() {
        for (auto it = graph_->begin(); it != graph_->end(); ++it) {
            shortest_path_[it.getNode()] = INF;
        }
    }

    void setSourceNode(const ValueType& source_data) {
        source_node_ = graph_->getNode(source_data);
        
        if (source_node_ == nullptr) {
            throw std::runtime_error("Source node not found in graph.");
        }
        shortest_path_[source_node_] = 0;
        node_priority_queue_.emplace(0, source_node_);
    }

    void runOnce() {
        if (done()) {
            return;
        }

        auto [current_cost, current_node] = node_priority_queue_.top();
        node_priority_queue_.pop();

        if (current_cost > shortest_path_[current_node]) {
            return;
        }

        for (const auto& [neighbor, cost] : current_node->neighbors_) {
            auto cost_to_go = cost + current_cost;
            if (cost_to_go < shortest_path_[neighbor]) {
                shortest_path_[neighbor] = cost_to_go;
                node_priority_queue_.emplace(cost_to_go, neighbor);
                predecessor_map_[neighbor] = current_node;
            }
        }
    }

    void run() {
        while (!done()) {
            runOnce();
        }
    }

    void printCostToGo() {
        for (const auto& [node, cost] : shortest_path_) {
            if (cost < INF) {
                std::cout << "Cost to go from " <<
                    source_node_->data_ << " to " <<
                    node->data_ << " is " << cost << "\n";
            } else {
                std::cout << "Cost to go from " <<
                    source_node_->data_ << " to " <<
                    node->data_ << " is " << "infinity" << "\n";
            }
        }
    }

    void findPath(const ValueType& target) {
        auto node = graph_->getNode(target);
        if (shortest_path_[node] == INF) {
            std::cout << "No path exists" << std::endl;
            return;
        }

        std::stack<NodePtr> path_stack;
        
        while (node != source_node_) {
            path_stack.push(node);
            node = predecessor_map_[node];
        }
        path_stack.emplace(node);

        while (!path_stack.empty()) {
            auto path_node = path_stack.top();
            path_stack.pop();
            std::cout << path_node->data_ << " -> ";
        }
        std::cout << "END" << std::endl;
    }

private:
    NodePtr source_node_{nullptr};
    std::shared_ptr<Graph> graph_;
    std::unordered_map<NodePtr, CostType> shortest_path_;
    std::unordered_map<NodePtr, NodePtr> predecessor_map_; // Child -> Parent
    std::priority_queue<NodeQueueType, std::vector<NodeQueueType>, std::greater<void>> node_priority_queue_;
};

#endif /* _DIJKSTRA_H_ */