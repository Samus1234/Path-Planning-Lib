#ifndef _PLANNING_UTILS_H_
#define _PLANNING_UTILS_H_

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <memory>
#include "graph.hpp"

namespace PathPlanningUtils {

struct Cell {
    int x, y;
    Cell(int x, int y) : x(x), y(y) {}

    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y;
    }
};

struct CellHash {
    std::size_t operator()(const Cell& cell) const {
        return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
    }
};

using CellGraph = Graph<Cell, unsigned, true, true, CellHash>;

struct AStarNode {
    Cell cell;
    double g_cost; // Cost from start to this cell
    double h_cost; // Heuristic cost from this cell to goal
    double f_cost; // g_cost + h_cost

    AStarNode(const Cell& cell, double g, double h) : cell(cell), g_cost(g), h_cost(h) {
        f_cost = g_cost + h_cost;
    }

    bool operator>(const AStarNode& other) const {
        return f_cost > other.f_cost;
    }
};

class OccupancyGrid {
public:
    OccupancyGrid(size_t width, size_t height) : width_(width), height_(height) {
        grid_.resize(height, std::vector<int>(width, 0));
    }

    size_t width() const {
        return width_;
    }

    size_t height() const {
        return height_;
    }

    void addObstacle(int x, int y) {
        if (isValid(x, y)) {
            grid_[y][x] = 1;
        }
    }

    void removeObstacle(int x, int y) {
        if (isObstacle(x, y)) {
            grid_[y][x] = 0;
        }
    }

    void setPath(int x, int y) {
        if(isFree(x, y)) {
            grid_[y][x] = -1;
        }
    }

    void setStart(int x, int y) {
        start_ = Cell(x, y);
    }

    void setGoal(int x, int y) {
        goal_ = Cell(x, y);
    }

    bool isFree(int x, int y) const {
        return isValid(x, y) && grid_[y][x] == 0;
    }

    bool isObstacle(int x, int y) const {
        return isValid(x, y) && grid_[y][x] == 1;
    }

    bool isValid(int x, int y) const {
        return x >= 0 && y >= 0 && x < width_ && y < height_;
    }

    void display() const {
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (x == start_.x && y == start_.y) {
                    std::cout << "S ";
                } else if (x == goal_.x && y == goal_.y) {
                    std::cout << "G ";
                } else if (grid_[y][x] == -1) {
                    std::cout << "\u2394 ";
                } else if (grid_[y][x] == 1) {
                    std::cout << "\u25A0 ";
                } else {
                    std::cout << ". ";
                }
            }
            std::cout << "\n";
        }
    }

    const Cell& getStart() const {
        return start_;
    }

    const Cell& getGoal() const {
        return goal_;
    }

private:
    size_t width_, height_;
    std::vector<std::vector<int>> grid_;
    Cell start_{0, 0};
    Cell goal_{0, 0};
};

void createGraphFromGrid(const OccupancyGrid& grid, std::shared_ptr<CellGraph> graph) {
    // Add all grid cells to graph
    for (int x = 0; x < grid.width(); ++x) {
        for (int y = 0; y < grid.height(); ++y) {
            if (grid.isFree(x, y)) {
                graph->insertNode(Cell(x, y));
            }
        }
    }
    // Next, add all the edges, i.e., neioghbors
    for (int x = 0; x < grid.width(); ++x) {
        for (int y = 0; y < grid.height(); ++y) {
            if (grid.isFree(x, y)) {
                Cell current_cell(x, y);
                // Vector of primitive motion directions
                const std::vector<std::pair<int, int>> directions = {
                    {0, 1},  {0, -1},
                    {1, 0},  {-1, 0},
                    {1, 1},  {-1, 1},
                    {1, -1},  {-1, -1},
                };
                for (const auto& [dx, dy] : directions) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (grid.isValid(nx, ny) && !grid.isObstacle(nx, ny)) {
                        Cell neighbor_cell(nx, ny);
                        if (dx*dy == 0) {
                            graph->addEdge(current_cell, neighbor_cell, 1000);
                        } else {
                            graph->addEdge(current_cell, neighbor_cell, 1414);
                        }
                    }
                }
            }
        }
    }
}

OccupancyGrid createTestGrid() {
    OccupancyGrid grid(80, 80);

    // Define outer boundaries (walls)
    for (int x = 0; x < 80; ++x) {
        grid.addObstacle(x, 0);      // Top boundary
        grid.addObstacle(x, 79);     // Bottom boundary
    }
    for (int y = 0; y < 80; ++y) {
        grid.addObstacle(0, y);      // Left boundary
        grid.addObstacle(79, y);     // Right boundary
    }

    // Horizontal and vertical walls creating rooms
    for (int x = 10; x < 70; ++x) {
        grid.addObstacle(x, 20);     // Horizontal wall 1
        grid.addObstacle(x, 40);     // Horizontal wall 2
        grid.addObstacle(x, 60);     // Horizontal wall 3
    }
    for (int y = 10; y < 70; ++y) {
        grid.addObstacle(20, y);     // Vertical wall 1
        grid.addObstacle(40, y);     // Vertical wall 2
        grid.addObstacle(60, y);     // Vertical wall 3
    }

    // Doors (gaps in walls for corridors)
    for (int x = 25; x <= 35; x += 10) {
        grid.removeObstacle(x, 20);  // Doors in horizontal wall 1
        grid.removeObstacle(x, 40);  // Doors in horizontal wall 2
        grid.removeObstacle(x, 60);  // Doors in horizontal wall 3
    }
    for (int y = 25; y <= 65; y += 10) {
        grid.removeObstacle(20, y);  // Doors in vertical wall 1
        grid.removeObstacle(40, y);  // Doors in vertical wall 2
        grid.removeObstacle(60, y);  // Doors in vertical wall 3
    }

    // Additional obstacles for complexity
    for (int x = 5; x <= 15; ++x) {
        for (int y = 5; y <= 15; ++y) {
            grid.addObstacle(x, y);  // Top-left room obstacles
        }
    }
    for (int x = 65; x <= 75; ++x) {
        for (int y = 65; y <= 75; ++y) {
            grid.addObstacle(x, y);  // Bottom-right room obstacles
        }
    }
    for (int x = 45; x <= 55; ++x) {
        for (int y = 45; y <= 55; ++y) {
            grid.addObstacle(x, y);  // Central room obstacles
        }
    }

    // Setting start and goal points
    grid.setStart(1, 1);             // Start near the top-left corner
    grid.setGoal(78, 78);            // Goal near the bottom-right corner

    return grid;
}

} // end PathPlanningUtils

#endif /* _PLANNING_UTILS_H_ */