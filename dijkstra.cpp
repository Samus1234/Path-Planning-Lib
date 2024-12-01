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

int main(int argc, char** argv) {

    auto graph = std::make_shared<Graph<char>>();

    for (char node = 'A'; node <= 'T'; ++node) {
        graph->insertNode(node);
    }

    graph->addEdge('A', 'B', 3);
    graph->addEdge('A', 'C', 4);
    graph->addEdge('A', 'D', 2);
    graph->addEdge('B', 'E', 6);
    graph->addEdge('B', 'F', 5);
    graph->addEdge('C', 'G', 2);
    graph->addEdge('C', 'H', 7);
    graph->addEdge('D', 'I', 4);
    graph->addEdge('D', 'J', 8);
    graph->addEdge('E', 'K', 3);
    graph->addEdge('E', 'L', 1);
    graph->addEdge('F', 'M', 4);
    graph->addEdge('F', 'N', 6);
    graph->addEdge('G', 'O', 5);
    graph->addEdge('G', 'P', 3);
    graph->addEdge('H', 'Q', 2);
    graph->addEdge('H', 'R', 4);
    graph->addEdge('I', 'S', 6);
    graph->addEdge('I', 'T', 5);
    graph->addEdge('J', 'K', 3);
    graph->addEdge('J', 'L', 7);
    graph->addEdge('K', 'M', 2);
    graph->addEdge('L', 'N', 4);
    graph->addEdge('M', 'O', 3);
    graph->addEdge('N', 'P', 5);
    graph->addEdge('O', 'Q', 6);
    graph->addEdge('P', 'R', 2);
    graph->addEdge('Q', 'S', 7);
    graph->addEdge('R', 'T', 4);

    std::cout << "Number of Vertices: " << graph->numVertices() << " and number of edges: " << graph->numEdges() << std::endl;

    Dijkstra<Graph<char>> dijkstra(graph);

    dijkstra.setSourceNode('A');

    size_t count = 1;
    
    while (!dijkstra.done()) {
        std::cout << "Iteration: " << count++ << "\n";
        dijkstra.printCostToGo();
        std::cout << "<------------------------------------------------>\n";
        dijkstra.findPath('R');
        std::cout << "<------------------------------------------------>\n";
        dijkstra.runOnce();
    }
    

    return 0;
}