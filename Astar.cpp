#include "path_planning_utils.hpp"
#include <limits>
#include <memory>

template<typename Graph>
class Astar {
    using ValueType = typename Graph::ValueType;
    using NodeType = typename Graph::NodeType;
    using NodePtr = typename Graph::NodePtr;
    using Iterator = typename Graph::Iterator;
    using CostType = float;
    using NodeQueueType = typename std::pair<CostType, NodePtr>;
    static constexpr CostType INF = std::numeric_limits<CostType>::max();
public:
    Astar() = default;
    ~Astar() = default;

    Astar(const std::shared_ptr<Graph> graph) { setGraph(graph); }

    void setGraph(const std::shared_ptr<Graph> graph) {
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
            throw std::runtime_error("Invalid Measurement norm!");
            break;
        }
    }

    void initialize() {
        for (auto it = graph_->begin(); it != graph_->end(); ++it) {
            g_cost_[it.getNode()] = INF;
            h_cost_[it.getNode()] = INF;
            f_cost_[it.getNode()] = INF;
        }
    }

    void setStartAndGoal(const ValueType& start, const ValueType& goal) {
        start_node_ = graph_->getNode(start);
        goal_node_ = graph_->getNode(goal);
        
        if (start_node_ == nullptr || goal_node_ == nullptr) {
            throw std::runtime_error("Invalid start and goal nodes!");
        }
        g_cost_[start_node_] = 0;
        node_priority_queue_.emplace(0, start_node_);
    }

    void runOnce() {
        if (node_priority_queue_.empty()) {
            complete_ = true;
            return;
        }

        auto [current_cost, current_node] = node_priority_queue_.top();
        node_priority_queue_.pop();

        if (current_node == goal_node_) {
            complete_ = true;
            return;
        }

        for (const auto& [neighbor, tentative_g_cost] : current_node->neighbors_) {
            auto heuristic_cost = heuristicCost(neighbor->data_, goal_node_->data_);
            CostType cost_to_go = (CostType) tentative_g_cost + current_cost + heuristic_cost;
            if (cost_to_go < f_cost_[neighbor]) {
                g_cost_[neighbor] = (CostType) tentative_g_cost;
                h_cost_[neighbor] = heuristic_cost;
                f_cost_[neighbor] = cost_to_go;
                node_priority_queue_.emplace(cost_to_go, neighbor);
                predecessor_map_[neighbor] = current_node;
            }
        }
    }

    void run() {
        while (!complete_) {
            runOnce();
        }
    }

    void printCostToGo() {
        for (const auto& [node, cost] : f_cost_) {
            if (cost < INF) {
                std::cout << "Cost to go from " <<
                    start_node_->data_ << " to " <<
                    node->data_ << " is " << cost << "\n";
            } else {
                std::cout << "Cost to go from " <<
                    start_node_->data_ << " to " <<
                    node->data_ << " is " << "infinity" << "\n";
            }
        }
    }

    std::stack<ValueType> findPath() {
        if (!complete_) {
            run();
        }
        
        auto node = goal_node_;
        if (f_cost_[node] == INF) {
            throw std::runtime_error("Path does not exist!");
        }

        std::stack<ValueType> path_stack;
        
        while (node != start_node_) {
            path_stack.emplace(node->data_);
            node = predecessor_map_[node];
        }
        path_stack.emplace(node->data_);
        return path_stack;
    }

    void printPath() {

        auto path_stack = findPath();

        while (!path_stack.empty()) {
            auto path_cell = path_stack.top();
            path_stack.pop();
            std::cout << "(" << path_cell.x << ", " << path_cell.y << ")" << " -> ";
        }
        std::cout << "COMPLETE" << std::endl;
    }

private:
    bool complete_{false};
    NodePtr start_node_{nullptr};
    NodePtr goal_node_{nullptr};
    std::shared_ptr<Graph> graph_;
    std::unordered_map<NodePtr, CostType> g_cost_;
    std::unordered_map<NodePtr, CostType> h_cost_;
    std::unordered_map<NodePtr, CostType> f_cost_;
    std::unordered_map<NodePtr, NodePtr> predecessor_map_; // Child -> Parent
    std::priority_queue<NodeQueueType, std::vector<NodeQueueType>, std::greater<void>> node_priority_queue_;
};

int main(int argc, char** argv) {

    auto graph = std::make_shared<PathPlanningUtils::CellGraph>();

    auto grid = PathPlanningUtils::createTestGrid();

    PathPlanningUtils::createGraphFromGrid(grid, graph);

    std::cout << "Number of Vertices: " << graph->numVertices() << " and number of edges: " << graph->numEdges() << std::endl;

    Astar<PathPlanningUtils::CellGraph> planner(graph);

    planner.setStartAndGoal(grid.getStart(), grid.getGoal());

    auto path_stack = planner.findPath();

    while (!path_stack.empty()) {
        auto path_cell = path_stack.top();
        path_stack.pop();
        grid.setPath(path_cell.x, path_cell.y);
    }

    std::cout << "Occupancy Grid:\n";
    grid.display();

    std::cout << "\n" << std::endl;

    planner.printPath();
    
    return 0;
}
