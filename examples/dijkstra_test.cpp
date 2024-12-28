#include "dijkstra.hpp"

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