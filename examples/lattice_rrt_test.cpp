#include <iostream>
#include "lattice_RRT.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Function to plot a DAG graph
template <typename NodeType, typename CostType>
void plotDAG(const DAG<NodeType, CostType>& graph,
    const std::vector<CircularObstacle>& circular_obstacles,
    const std::string& plot_path = "plots/graph_plot.png",
    const std::vector<CostType> edge_costs = {}) {
    // Extract nodes and edges
    const auto& nodes = graph.getNodes();
    const auto& edges = graph.getEdges();

    // Vectors to store coordinates for plotting
    std::vector<double> x_coords, y_coords;

    // Fill node coordinates
    for (const auto& node : nodes) {
        x_coords.push_back(node[0]);
        y_coords.push_back(node[1]);
    }

    plt::figure(1);

    // Plot nodes
    plt::scatter(x_coords, y_coords, 15, {{"color", "tab:blue"}}); // tab:blue for the node color

    // Annotate nodes with indices
    // for (size_t i = 0; i < nodes.size(); ++i) {
    //     plt::text(x_coords[i], y_coords[i], std::to_string(edge_costs[i]));
    // }

    // Plot edges
    for (const auto& [parent, child, cost] : edges) {
        auto start = nodes[parent];
        auto end = nodes[child];
        plt::plot({start[0], end[0]}, {start[1], end[1]}, "tab:blue"); // tab:blue for edge color
    }

    for (const auto& obstacle : circular_obstacles) {
        std::vector<double> x, y;
        for (double theta = 0; theta <= 2 * M_PI; theta += 0.1) {
            x.push_back(obstacle.center(0) + obstacle.radius * cos(theta));
            y.push_back(obstacle.center(1) + obstacle.radius * sin(theta));
        }
        plt::plot(x, y, "r-");
    }

    plt::xlim(-10, 10);
    plt::ylim(-10, 10);

    // Set grid and equal aspect ratio
    plt::grid(true);
    plt::axis("equal");

    // Save plot to the specified file path
    plt::save(plot_path);

    // Notify the user
    std::cout << "Plot saved to: " << plot_path << std::endl;
}


int main(int argc, char** argv) {

    std::vector<CircularObstacle> circular_obstacles = {
        {{-2.509198, 3.014286}, 1.597991}, // Obstacle 1
        {{1.973170, -1.879627}, 1.733992}, // Obstacle 2
        {{-2.838328, 3.323523}, 1.401673}, // Obstacle 3
        {{1.161452, -2.588310}, 1.954865}, // Obstacle 4
        {{2.648853, -3.753218}, 1.772737}  // Obstacle 5
    };

    LatticePlanner::State resolution(1e-3, 1e-3, M_PI/180);  // 1 mm and 1 degree

    auto planner = std::make_shared<LatticePlanner::Planner>(M_PI/3, 1.0, resolution);

    auto dynamics = [planner] (LatticePlanner::State x, LatticePlanner::Control u, double dt)
        { return planner->dynamics(x, u, dt); };

    planner->generateMotionPrimitives();

    RRT<LatticePlanner::State, LatticePlanner::Control, double> lattice_rrt;

    lattice_rrt.setInitialNode(LatticePlanner::State(0, 0, 0));

    lattice_rrt.setGoalNode(LatticePlanner::State(2, 0, 0));

    lattice_rrt.setWeights(LatticePlanner::State(1.0, 1.0, 4.0));

    lattice_rrt.setMotionPrimitives(planner->getMotionPrimitives());

    lattice_rrt.setDynamics(dynamics);

    lattice_rrt.setObstacles(circular_obstacles);

    lattice_rrt.iterate(5000);

    const auto& graph = lattice_rrt.getGraph();

    const auto& edge_costs = lattice_rrt.getEdgeCosts();

    plotDAG(graph, circular_obstacles, "plots/graph_plot.png", edge_costs);

    return 0;
}