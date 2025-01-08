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

    graph->addEdge('A', 'B', 1);
    graph->addEdge('A', 'E', 3);
    graph->addEdge('A', 'C', 5);

    graph->addEdge('B', 'D', 2);
    graph->addEdge('B', 'G', 8);
    
    graph->addEdge('C', 'B', 3);
    graph->addEdge('C', 'D', 3);

    graph->addEdge('D', 'G', 3);

    graph->addEdge('E', 'G', 5);

    auto nodes = graph->getNodes();

    for (int i = 0; i < 5; i++) {
        auto neighbors = graph->getNeighbors(i);
        std::cout << "Neighbors of " << nodes[i] << std::endl;
        for (const auto& [index, cost] : neighbors) {
            std::cout << nodes[index] << ", with cost: " << cost << std::endl;
        }
    }
    
    
}

int main(int argc, char** argv) {

    testDAG();

    return 0;
}