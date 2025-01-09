#include <iostream>
#include "lattice_planner.hpp"
#include "dag_astar.hpp"

void savePathToCSV(const std::vector<std::pair<int, int>>& optimal_path, 
                   const std::string& file_path) {
    std::ofstream out(file_path);
    out << "state_idx,primitive_idx\n";
    for (const auto& [state_idx, primitive_idx] : optimal_path) {
        out << state_idx << "," << primitive_idx << "\n";
    }
    out.close();
}

int main(int argc, char** argv) {

    using LatticeDAG = AStar::DAG<LatticePlanner::State, double>;

    using LatticeAStar = AStar::AStar<LatticePlanner::State, double>;

    LatticePlanner::State resolution;

    resolution << 1e-3, 1e-3, M_PI/180; // 1 mm and 1 degree

    LatticePlanner::Planner planner(M_PI/3, 1.0, resolution);

    LatticePlanner::State x_init;

    x_init << 0, 0, 0;

    planner.generateMotionPrimitives();

    planner.generateLattice(x_init, 2);

    auto motion_primitives = planner.getMotionPrimitives();

    auto lattice_nodes = planner.getLatticeNodes();

    auto lattice_edges = planner.getLatticeEdges();

    planner.saveLatticeToCSV("data/lattice_nodes.csv", "data/lattice_edges.csv", "data/motion_primitives.csv");

    LatticeDAG lattice_dag(lattice_nodes, lattice_edges, resolution);

    std::function<double(const LatticePlanner::State&, const LatticePlanner::State&)> euclidean_heuristic =
        [] (const LatticePlanner::State& x_1, const LatticePlanner::State& x_2) { return (x_2 - x_1).norm(); };
    
    LatticeAStar lattice_astar(lattice_dag, euclidean_heuristic);

    LatticePlanner::State start, goal;

    start << 0,0,0;

    goal << 0.798,-0.033,-0.15708;

    auto optimal_path = lattice_astar.findPath(start, goal);

    savePathToCSV(optimal_path, "data/optimal_path.csv");
    

    return 0;
}