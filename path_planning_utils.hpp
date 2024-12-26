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

class OccupancyGrid {
public:
    OccupancyGrid(size_t width, size_t height) : width_(width), height_(height) {
        grid_.resize(height, std::vector<float>(width, 0.0f)); // Initialize with free space
    }

    size_t width() const { return width_; }
    size_t height() const { return height_; }

    void addObstacle(int x, int y) {
        if (isValid(x, y)) {
            grid_[y][x] = 1.0f; // Fully occupied
        }
    }

    void removeObstacle(int x, int y) {
        if (isObstacle(x, y)) {
            grid_[y][x] = 0.0f; // Reset to free space
        }
    }

    void addDynamicObstacle(const Cell& location) {
        if (isValid(location.x, location.y)) {
            grid_[location.y][location.x] = 1.0f; // Obstacle cost
        }
    }

    void removeDynamicObstacle(const Cell& location) {
        if (isObstacle(location.x, location.y)) {
            grid_[location.y][location.x] = 0.0f; // Free space
        }
    }

    void updateDynamicCost(const Cell& location, float cost) {
        if (isValid(location.x, location.y)) {
            grid_[location.y][location.x] = cost; // Update cost
        }
    }

    void addCost(int x, int y, float cost) {
        if (isValid(x, y) && cost >= 0.0f && cost <= 1.0f) {
            grid_[y][x] = cost; // Set cell to a specific cost
        }
    }

    float getCost(int x, int y) const {
        if (isValid(x, y)) {
            return grid_[y][x];
        } else {
            throw std::out_of_range("Requested cell is out of grid bounds.");
        }
    }

    void setPath(int x, int y) {
        if (isFree(x, y)) {
            grid_[y][x] = -1.0f; // Mark as part of the path
        }
    }

    void setStart(int x, int y) { start_ = Cell(x, y); }
    void setGoal(int x, int y) { goal_ = Cell(x, y); }

    bool isFree(int x, int y) const {
        return isValid(x, y) && grid_[y][x] < 0.5f; // Free if cost < 0.5
    }

    bool isObstacle(int x, int y) const {
        return isValid(x, y) && grid_[y][x] >= 0.5f; // Obstacle if cost >= 0.5
    }

    bool isValid(int x, int y) const {
        return x >= 0 && y >= 0 && x < width_ && y < height_;
    }

    void inflateObstacles(float inflation_radius) {
        // Copy the original grid
        auto temp_grid = grid_;

        // Iterate over every cell
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (temp_grid[y][x] >= 0.9f) { // If the cell is an obstacle
                    // Add costs to cells within the inflation radius
                    for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                        for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (isValid(nx, ny)) {
                                float distance = std::sqrt(dx * dx + dy * dy);
                                if (distance <= inflation_radius) {
                                    float inflation_cost = 1.0f - (distance / inflation_radius);
                                    grid_[ny][nx] = std::max(grid_[ny][nx], inflation_cost);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void display() const {
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (x == start_.x && y == start_.y) {
                    std::cout << "S ";
                } else if (x == goal_.x && y == goal_.y) {
                    std::cout << "G ";
                } else if (grid_[y][x] == -1.0f) {
                    std::cout << "\u2394 ";
                } else if (grid_[y][x] >= 0.9f) {
                    std::cout << "\u25A0 ";
                } else if (grid_[y][x] > 0.0f) {
                    std::cout << "C "; // Cost cell
                } else {
                    std::cout << ". ";
                }
            }
            std::cout << "\n";
        }
    }

    const Cell& getStart() const { return start_; }
    const Cell& getGoal() const { return goal_; }

private:
    size_t width_, height_;
    std::vector<std::vector<float>> grid_;
    Cell start_{0, 0};
    Cell goal_{0, 0};
};

void createGraphFromGrid(const OccupancyGrid& grid, std::shared_ptr<CellGraph> graph) {
    for (int x = 0; x < grid.width(); ++x) {
        for (int y = 0; y < grid.height(); ++y) {
            if (grid.isFree(x, y)) {
                graph->insertNode(Cell(x, y));
            }
        }
    }

    for (int x = 0; x < grid.width(); ++x) {
        for (int y = 0; y < grid.height(); ++y) {
            if (grid.isFree(x, y)) {
                Cell current_cell(x, y);
                const std::vector<std::pair<int, int>> directions = {
                    {0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1},
                };
                for (const auto& [dx, dy] : directions) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (grid.isValid(nx, ny) && grid.isFree(nx, ny)) {
                        float current_cost = grid.getCost(nx, ny);
                        float edge_cost = (dx * dy == 0 ? 1.0f : std::sqrt(2.0f)) + current_cost;
                        graph->addEdge(current_cell, Cell(nx, ny), edge_cost);
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
