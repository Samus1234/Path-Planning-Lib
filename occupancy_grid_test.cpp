#include "graph.hpp"
#include <vector>
#include "matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;


int main(int argc, char** argv) {

    // Data
    std::vector<double> x = {1, 2, 3, 4, 5};
    std::vector<double> y = {1, 4, 9, 16, 25};

    // Plot
    plt::plot(x, y);
    plt::title("Example Plot");
    plt::xlabel("X-axis");
    plt::ylabel("Y-axis");
    plt::grid(true);
    plt::save("/home/sidd/Documents/CPP_Practice/plots/plot.png");

    return 0;

    return 0;
}

