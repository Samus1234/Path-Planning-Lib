#include <iostream>
#include "lattice_planner.hpp"


int main(int argc, char** argv) {

    std::cout << "Function to exercise lattice planner" << std::endl;

    LatticePlanner::State resolution;

    resolution << 1e-3, 1e-3, M_PI/180; // 1 mm and 1 degree

    LatticePlanner::Planner planner(M_PI/3, 1.0, resolution);

    LatticePlanner::State x_init;

    x_init << 0, 0, 0;

    planner.generateMotionPrimitives();

    planner.generateLattice(x_init, 3);

    auto motion_primitives = planner.getMotionPrimitives();

    auto lattice_nodes = planner.getLatticeNodes();

    auto lattice_edges = planner.getLatticeEdges();

    planner.saveLatticeToCSV("data/lattice_nodes.csv", "data/lattice_edges.csv", "data/motion_primitives.csv");

    return 0;
}