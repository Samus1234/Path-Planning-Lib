#include "Astar.hpp"

int main(int argc, char** argv) {

    auto graph = std::make_shared<PathPlanningUtils::CellGraph>();

    auto grid = PathPlanningUtils::createTestGrid();

    grid.inflateObstacles(1.5f);

    PathPlanningUtils::createGraphFromGrid(grid, graph);

    std::cout << "Number of Vertices: " << graph->numVertices() << " and number of edges: " << graph->numEdges() << std::endl;

    AStar<PathPlanningUtils::CellGraph> planner(graph);

    planner.setStartAndGoalNodes(grid.getStart(), grid.getGoal());

    auto path_stack = planner.getPath();

    while (!path_stack.empty()) {
        auto path_cell = path_stack.top();
        path_stack.pop();
        grid.setPath(path_cell.x, path_cell.y);
    }

    std::cout << "Occupancy Grid:\n";
    grid.display();
    
    return 0;
}