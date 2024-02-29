# Motion-Planning

![1](1.gif) ![2](2.gif)   

This project provides a Python implementation for drone motion planning, integrating functionalities from two main files: `motion_planning.py` and `planning_utils.py`. The project's goal is to demonstrate autonomous drone flight and path planning in a simulated environment using real-world geographic locations.

## `motion_planning.py`

This script serves as the primary execution point for the drone's motion planning. It utilizes the `udacidrone` framework for simulating drone flight and interfaces with the `planning_utils.py` module for path planning utilities.

### Key Features
- Drone state management through a finite state machine (FSM) with states like MANUAL, ARMING, TAKEOFF, WAYPOINT, LANDING, DISARMING, and PLANNING.
- Connection setup with the drone simulation environment using Mavlink.
- Conversion of global coordinates to local waypoints.
- Integration with `planning_utils.py` for generating waypoints based on path planning algorithms.

### Dependencies
- `argparse` for parsing command-line arguments.
- `numpy` for numerical computations.
- `udacidrone` framework for drone simulation and control.
- `msgpack` for data serialization.
- `planning_utils.py` for utility functions and path planning.

## `planning_utils.py`

This utility module provides the necessary functions for creating a grid representation of a 2D configuration space and planning paths within that space.

### Key Features
- Grid creation from obstacle data, considering drone altitude and safety distance.
- Implementation of the A* search algorithm for pathfinding in the grid.
- Utilization of Bresenham's algorithm for line drawing, facilitating path optimization.
- Support for Voronoi graphs and KDTree for efficient nearest-neighbor searches.

### Dependencies
- `numpy` and `networkx` for mathematical operations and graph management.
- `scipy` for spatial algorithms like Voronoi diagrams and KDTree.
- `shapely` for geometric operations.

## Installation and Requirements

To run this project, ensure you have Python 3.x installed along with the following packages: `numpy`, `networkx`, `scipy`, `shapely`, `udacidrone`, and `bresenham`.

1. Clone the repository or download the source files.
2. Install the dependencies using pip:
    - pip install numpy networkx scipy shapely udacidrone bresenham

3. Run the `motion_planning.py` script to start the simulation:
    - python motion_planning.py


## Usage

The `motion_planning.py` script can be executed with optional command-line arguments to customize the drone's flight parameters, such as the target altitude and safety distance. Refer to the script's argument parser for more details.

This project is designed for educational purposes to demonstrate the concepts of motion planning and pathfinding in autonomous drone flights. The provided implementation offers a foundation that can be extended with more sophisticated algorithms and features for real-world applications.


