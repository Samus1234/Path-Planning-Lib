#include <iostream>
#include <memory>
#include "dag.hpp"

void testDAG() {
    auto graph = std::make_shared<DAG<char, int>>();
    graph->addNode('A');
    graph->addNode('B');
    graph->addNode('C');
    graph->addNode('D');
    graph->addNode('E');
    graph->addNode('G');

    graph->addEdge(0, 1, 1);
    graph->addEdge(0, 4, 3);
    graph->addEdge(0, 2, 5);

    graph->addEdge(1, 3, 2);
    graph->addEdge(1, 5, 8);
    
    graph->addEdge(2, 1, 3);
    graph->addEdge(2, 3, 3);

    graph->addEdge(3, 5, 3);

    graph->addEdge(4, 5, 5);

    graph->buildNeighborMap();

    auto neighbors = graph->getNeighbors(0);

    for (const auto& [index, cost] : neighbors) {
        std::cout << index << ", " << cost << std::endl;
    }
    
}

int main(int argc, char** argv) {



    return 0;
}