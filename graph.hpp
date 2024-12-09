#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <queue>

template<typename T, typename W>
struct GraphNode {
    using ValueType = T;
    using CostType = W;
    using NodePtr = GraphNode*;
    GraphNode() = default;
    ~GraphNode() = default;
    GraphNode(const T& data) : data_(data) {}
    GraphNode(T&& data) : data_(std::move(data)) {}
    T data_{};
    std::unordered_map<NodePtr, CostType> neighbors_;
};

template<typename Graph, bool DFS>
class GraphIterator {
    using ValueType = typename Graph::ValueType;
    using NodeType = typename Graph::NodeType;
    using NodePtr = typename Graph::NodePtr;
public:
    // Iterator traits
    using iterator_category = std::bidirectional_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = ValueType;
    using pointer = ValueType*;
    using reference = ValueType&;

    GraphIterator() = default;

    ~GraphIterator() = default;

    GraphIterator(NodePtr node) : node_(node), starting_node_(node) {
        if (node_) {
            visited_.insert(node_);
            for (auto& [neighbor, _] : node_->neighbors_) {
                if (DFS) {
                    dfs_stack_.push(neighbor);
                } else {
                    bfs_queue_.push(neighbor);
                }
                parent_[neighbor] = node_;
            }
        }
    }

    void printParentMap() const {
        for (const auto& [child, parent] : parent_) {
            std::cout << "Child: " << child->data_ 
                      << ", Parent: " << (parent ? parent->data_ : 'N') << "\n";
        }
    }

    GraphIterator& operator++() {
        while (!dfs_stack_.empty() || !bfs_queue_.empty()) {
            NodePtr curr_node = nullptr;

            if (DFS) {
                curr_node = dfs_stack_.top();
                dfs_stack_.pop();
            } else {
                curr_node = bfs_queue_.front();
                bfs_queue_.pop();
            }

            if (visited_.find(curr_node) == visited_.end()) {
                visited_.insert(curr_node);

                for (auto& [neighbor, _] : curr_node->neighbors_) {
                    if (visited_.find(neighbor) == visited_.end()) {
                        if (DFS) {
                            dfs_stack_.push(neighbor);
                        } else {
                            bfs_queue_.push(neighbor);
                        }
                    }
                    if (parent_.find(neighbor) == parent_.end()) {
                        parent_[neighbor] = curr_node;
                    }
                }

                node_ = curr_node;
                return *this;
            }
        }

        node_ = nullptr;
        return *this;
    }

    GraphIterator& operator--() {
        if (node_ == nullptr) {
            throw std::out_of_range("Cannot decrement: current node is nullptr.");
        }
        if (node_ == starting_node_) {
            throw std::out_of_range("Cannot decrement: current node is the starting node.");
        }

        auto it = parent_.find(node_);
        if (it == parent_.end()) {
            throw std::out_of_range("Cannot decrement: no parent exists for the current node.");
        }

        node_ = it->second;
        return *this;
    }


    GraphIterator operator++(int) {
        GraphIterator it = *this;
        ++(*this);
        return it;
    }

    GraphIterator operator--(int) {
        GraphIterator it = *this;
        --(*this);
        return it;
    }

    reference operator*() {
        return node_->data_;
    }

    pointer operator->() {
        return &(node_->data_);
    }

    bool operator==(const GraphIterator& other) const {
        return node_ == other.node_;
    }

    bool operator!=(const GraphIterator& other) const {
        return !(*this == other);
    }

    NodePtr& getNode() {
        return node_;
    }

private:
    NodePtr node_{nullptr};
    NodePtr starting_node_{nullptr};
    std::stack<NodePtr> dfs_stack_;
    std::queue<NodePtr> bfs_queue_;
    std::unordered_map<NodePtr, NodePtr> parent_;
    std::unordered_set<NodePtr> visited_;
};

template<
    typename T,
    typename W = unsigned,
    bool DIRECTED = true,
    bool DFS = true,
    typename Hash = std::hash<T>>
class Graph {
public:
    using ValueType = T;
    using NodeType = GraphNode<T, W>;
    using NodePtr = NodeType*;
    using CostType = typename NodeType::CostType;
    using Iterator = GraphIterator<Graph, DFS>;

    Graph() = default;
    ~Graph() {
        clear();
    }

    size_t numVertices() const {
        return graph_nodes_.size();
    }

    size_t numEdges() const {
        size_t num_edges = 0;
        for (const auto& [_, node] : graph_nodes_) {
            num_edges += node->neighbors_.size();
        }
        return num_edges;
    }

    void clear() {
        for (auto& [_, node] : graph_nodes_) {
            delete node;
        }
        graph_nodes_.clear();
    }

    NodePtr getNode(const ValueType& data) const {
        auto it = graph_nodes_.find(data);
        if (it != graph_nodes_.end()) {
            return it->second;
        }
        std::cout << "Node not in graph.\n";
        return nullptr;
    }

    void insertNode(const ValueType& data) {
        if (graph_nodes_.find(data) != graph_nodes_.end()) {
            std::cout << "Node already present.\n";
            return;
        }
        graph_nodes_[data] = new NodeType(data);
        if (root_ == nullptr) {
            root_ = graph_nodes_.begin()->second;
        }
    }

    void addEdge(const ValueType& from_data, const ValueType& to_data, const CostType& weight = 1) {
        auto from = getNode(from_data);
        auto to = getNode(to_data);
        if (!from || !to) {
            throw std::runtime_error("One or both nodes do not exist.");
        }
        from->neighbors_[to] = weight;
        if (!DIRECTED) {
            to->neighbors_[from] = weight;
        }
    }

    Iterator begin() {
        if (!root_ && !graph_nodes_.empty()) {
            root_ = graph_nodes_.begin()->second;
        }
        return Iterator(root_);
    }

    Iterator end() {
        return Iterator(nullptr);
    }

private:
    std::unordered_map<ValueType, NodePtr, Hash> graph_nodes_;
    NodePtr root_{nullptr};
};

#endif /* _GRAPH_H_ */
