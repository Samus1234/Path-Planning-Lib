#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <queue>

template<typename T>
struct GraphNode {
    GraphNode() = default;
    ~GraphNode() = default;
    GraphNode(const T& data) : data_(data) {}
    GraphNode(T&& data) : data_(std::move(data)) {}
    T data_{};
    std::unordered_map<GraphNode*, unsigned> neighbours_;
};

template<typename Graph, bool DFS>
class GraphIterator {
    using ValueType = typename Graph::ValueType;
    using NodeType = typename Graph::NodeType;
    using NodePtr = NodeType*;
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
            for (auto& [neighbor, _] : node_->neighbours_) {
                if (DFS) {
                    dfs_stack_.push(neighbor);
                } else {
                    bfs_queue_.push(neighbor);
                }
                // Initialize parent mapping for direct neighbors
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

            // Process the node only if it hasn't been visited yet
            if (visited_.find(curr_node) == visited_.end()) {
                visited_.insert(curr_node);

                // Push neighbors to the stack or queue for further traversal
                for (auto& [neighbor, _] : curr_node->neighbours_) {
                    if (visited_.find(neighbor) == visited_.end()) {
                        if (DFS) {
                            dfs_stack_.push(neighbor);
                        } else {
                            bfs_queue_.push(neighbor);
                        }
                    }
                    // Update the parent map for every neighbor
                    if (parent_.find(neighbor) == parent_.end()) {
                        parent_[neighbor] = curr_node;
                    }
                }

                node_ = curr_node;
                return *this; // Exit after processing one node
            }
        }

        // No more nodes to process
        node_ = nullptr;
        return *this;
    }

    GraphIterator& operator--() {
        if (node_ == nullptr) {
            throw std::out_of_range("Cannot decrement: current node is nullptr.");
        }
        if (node_ == starting_node_) { // Starting node check
            throw std::out_of_range("Cannot decrement: current node is the starting node.");
        }

        // Find the parent of the current node
        auto it = parent_.find(node_);
        if (it == parent_.end()) {
            throw std::out_of_range("Cannot decrement: no parent exists for the current node.");
        }

        node_ = it->second; // Move to the parent
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

private:
    NodePtr node_{nullptr};
    NodePtr starting_node_{nullptr};
    std::stack<NodePtr> dfs_stack_;
    std::queue<NodePtr> bfs_queue_;
    std::unordered_map<NodePtr, NodePtr> parent_;
    std::unordered_set<NodePtr> visited_;
};

template<typename T, bool DIRECTED, bool DFS>
class Graph {
public:
    using ValueType = T;
    using NodeType = GraphNode<T>;
    using NodePtr = NodeType*;
    using Iterator = GraphIterator<Graph, DFS>;

    Graph() = default;
    ~Graph() {
        clear();
    }

    size_t size() const {
        return graph_nodes_.size();
    }

    void clear() {
        for (auto& [_, node] : graph_nodes_) {
            delete node;
        }
        graph_nodes_.clear();
    }

    NodePtr getNode(const T& data) const {
        auto it = graph_nodes_.find(data);
        if (it != graph_nodes_.end()) {
            return it->second;
        }
        std::cout << "Node not in graph.\n";
        return nullptr;
    }

    void insertNode(const T& data) {
        if (graph_nodes_.find(data) != graph_nodes_.end()) {
            std::cout << "Node already present.\n";
            return;
        }
        graph_nodes_[data] = new NodeType(data);
        if (root_ == nullptr) {
            root_ = graph_nodes_.begin()->second;
        }
    }

    void addEdge(const T& from_data, const T& to_data, const unsigned& weight = 1) {
        auto from = getNode(from_data);
        auto to = getNode(to_data);
        if (!from || !to) {
            throw std::runtime_error("One or both nodes do not exist.");
        }
        from->neighbours_[to] = weight;
        if (!DIRECTED) {
            to->neighbours_[from] = weight;
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
    std::unordered_map<T, NodePtr> graph_nodes_;
    NodePtr root_{nullptr};
};

int main() {
    Graph<char, false, true> graph; // Undirected graph

    graph.insertNode('A');
    graph.insertNode('B');
    graph.insertNode('C');
    graph.insertNode('D');
    graph.insertNode('E');

    graph.addEdge('A', 'B');
    graph.addEdge('A', 'D');
    graph.addEdge('B', 'E');
    graph.addEdge('B', 'C');

    auto it = graph.begin();


    std::cout << "Start -> ";

    std::cout << *(it++) << " -> ";
    std::cout << *(it++) << " -> ";
    std::cout << *(it++) << " -> ";
    std::cout << *(it) << " -> ";

    std::cout << "End\n";

    std::cout << "End -> ";

    

    std::cout << *(it--) << " -> ";
    std::cout << *(it--) << " -> ";
    std::cout << *(it) << " -> ";

    std::cout << "Start\n";


    return 0;
}
