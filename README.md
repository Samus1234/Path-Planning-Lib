# Path Planning Library

A library I wrote to better understand path and motion planning algorithsm, data strctures, and robotics utitlties. I've worked in implementing A*, arc-length parameterization for trajectory tracking, a discrete-time LQR, etc along with whatever data structure I though was interesting!

🚧 **This project is a work in progress** 🚧  
I will be actively working on adding new features to this project as we go. Any feedback would be appreciated. Things I plan on implementing soon include a full stack motion planner for a Quadrotor and an implementation of D-lite* for dynamic path planning.

---

## Directory Structure

```plaintext
.
├── build                     # Compiled binaries and libraries
├── data                      # Example input data files
├── examples                  # Test/example programs
├── include                   # Header files for the library
├── src                       # Source files for the library
├── plots                     # Output plots generated by examples/scripts
├── scripts                   # Python scripts for visualization and analysis
├── CMakeLists.txt            # Build configuration
├── build.sh                  # Script to build the project
├── test.sh                   # Script to run all examples
└── README.md                 # This file
```

---

## Features

- **Path Planning Algorithms**:
  - A* search algorithm
  - Dijkstra’s algorithm
- **Data Structures**:
  - Stacks, queues, binary trees, priority queues
- **Utilities**:
  - Arc-length parameterization
  - Circular buffers
  - Numerical integration
  - Trajectory generation and visualization

---

## Requirements

### Software
- **CMake** (version 3.10 or higher)
- **C++ Compiler**:
  - GCC 9+ or Clang 11+ (supports C++20)
- **Python** (3.10 or later, for scripts)

### Libraries
- **Eigen**: Required for numerical computations
- **Python Development Libraries**: Required for Python embedding

### Installation on Ubuntu
```bash
sudo apt update
sudo apt install cmake g++ libeigen3-dev python3-dev
```

---

## Build Instructions

Run the provided `build.sh` script to clean, configure, and build the project:
```bash
./build.sh
```

---

## Running Examples

### Using the Test Script
Run the provided `test.sh` script to execute all compiled examples:
```bash
./test.sh
```

### Running All Examples with CMake
To execute all compiled examples via CMake:
```bash
make run_all_examples
```

### Running Specific Examples
You can also run individual examples manually. Example:
```bash
./build/Astar_test/Astar_test
./build/arc_length_trajectory_test/arc_length_trajectory_test
```

---

## Visualization with Scripts

Python scripts in the `scripts` directory can be used for plotting and analyzing results.

### Example: Arc Length Trajectory Visualization
```bash
python3 scripts/arc_length_visualization.py
```

---

## Customization

### Adding a New Example
1. Create a new `.cpp` file in the `examples/` directory.
2. Add the file to the project. CMake will automatically detect it and create a corresponding executable.

### Adding a New Header/Source File
1. Add your `.hpp` file to the `include/` directory.
2. Add your `.cpp` file to the `src` directory.
3. Include the new files in your C++ code as needed.

---

## Troubleshooting

### Common Issues

#### Missing Eigen Library
If you encounter an error about `Eigen/Core` not being found, ensure that Eigen is installed:
```bash
sudo apt install libeigen3-dev
```
Alternatively, download and include Eigen manually in the `include/` directory.

#### Permission Denied
Ensure all scripts are executable:
```bash
chmod +x build.sh test.sh
```
