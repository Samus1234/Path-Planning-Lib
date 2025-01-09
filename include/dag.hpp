#ifndef _DAG_H_
#define _DAG_H_

#include <unordered_map>
#include <vector>
#include <tuple>
#include <algorithm>

// Directed Acyclic Graph
template <typename NodeType, typename CostType>
class DAG {
public:
    DAG() = default;
    ~DAG() = default;

    bool empty() const {
        return empty_;
    }

    void addNode(const NodeType& node) {
        nodes_.push_back(node);
        empty_ = false;
    }

    void addEdge(NodeType parent_node, NodeType child_node, CostType cost) {
        auto parent_it = std::find(nodes_.begin(), nodes_.end(), parent_node);
        auto child_it = std::find(nodes_.begin(), nodes_.end(), child_node);
        if (parent_it == nodes_.end()) {
            std::cout << "Parent Node not found in graph" << std::endl;
            return;
        }
        if (child_it == nodes_.end()) {
            std::cout << "Child Node not found in graph" << std::endl;
            return;
        }

        int parent = static_cast<int>(std::distance(nodes_.begin(), parent_it));
        int child = static_cast<int>(std::distance(nodes_.begin(), child_it));
        
        edges_.emplace_back(parent, child, cost);
        neighbor_map_[parent].emplace_back(child, cost);
    }

    void addIndexedEdge(int parent, int child, CostType cost) {       
        edges_.emplace_back(parent, child, cost);
        neighbor_map_[parent].emplace_back(child, cost);
    }

    void buildFullNeighborMap() {
        neighbor_map_.clear();
        for (const auto& [parent, child, cost] : edges_) {
            neighbor_map_[parent].emplace_back(child, cost);
        }
    }

    const std::vector<NodeType>& getNodes() const {
        return nodes_;
    }

    const std::vector<std::tuple<int, int, CostType>>& getEdges() const {
        return edges_;
    }

    std::vector<std::tuple<int, CostType>> getNeighbors(int nodeIndex) const {
        auto it = neighbor_map_.find(nodeIndex);
        if (it != neighbor_map_.end()) {
            return it->second;
        }
        return {};
    }

private:
    bool empty_{true};
    std::vector<NodeType> nodes_;
    std::vector<std::tuple<int, int, CostType>> edges_;
    std::unordered_map<int, std::vector<std::tuple<int, CostType>>> neighbor_map_;
};

#endif /* _DAG_H_ */