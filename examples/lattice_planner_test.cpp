#include <iostream>
#include "lattice_planner.hpp"


int main(int argc, char** argv) {

    std::cout << "Function to exercise lattice planner" << std::endl;

    LatticePlanner::Planner planner(1e-2, M_PI/5, 1.0);

    LatticePlanner::State x_init;

    x_init << 0, 0, 0;

    planner.generateMotionPrimitives();

    auto motion_primitives = planner.getMotionPrimitives();

    planner.generateLattice(x_init, 2);

    auto lattice_nodes = planner.getLatticeNodes();

    auto lattice_edges = planner.getLatticeEdges();

    planner.saveLatticeToCSV("data/lattice_nodes.csv", "data/lattice_edges.csv", "data/motion_primitives.csv");

    return 0;
}