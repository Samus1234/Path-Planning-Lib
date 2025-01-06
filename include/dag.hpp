#ifndef _DAG_H_
#define _DAG_H_

#include <unordered_map>
#include <vector>
#include <tuple>
#include <algorithm>

template <typename NodeType, typename CostType>
class DAG {
public:
    DAG() = default;
    ~DAG() = default;

    void addNode(const NodeType& node) {
        nodes_.push_back(node);
    }

    void addEdge(int parent, int child, CostType cost) {
        edges_.emplace_back(parent, child, cost);
    }

    void buildNeighborMap() {
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
    std::vector<NodeType> nodes_;
    std::vector<std::tuple<int, int, CostType>> edges_;
    std::unordered_map<int, std::vector<std::tuple<int, CostType>>> neighbor_map_;
};

#endif /* _DAG_H_ */