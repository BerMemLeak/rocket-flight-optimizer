# Rocket Flight Optimizer

This project implements a Rocket Flight Optimizer that searches for the optimal parameters of a rocket to achieve the best flight performance. The optimizer runs simulations of rocket flights, iterating through a range of parameters, and selects the configuration that results in the greatest flight distance.


### The goal of this project is to simulate rocket flights and optimize parameters such as:  

Initial velocity  
Start mass  
Coefficients of drag in X and Z axes  
Angle of pitch (tetta)  
Angle of path (psi)  
Position coordinates (x, y, z)  
Diameter of the rocket  
Fuel consumption rate  
Booster weight and undocking time  
Using a brute-force search, the program iterates through different combinations of these parameters and determines the configuration that results in the furthest flight distance.  

### Features

Parameter Optimization: Searches for optimal rocket configurations based on multiple parameters using brute-force.  
Flight Simulation: Simulates the rocket's flight based on physical parameters.  
Visualization: Offers functions to visualize rocket flight data in 2D and 3D graphs.  
Customizable Parameters: The range and step size for each parameter are customizable.  

### Installation

#### To run this project locally, follow these steps:
```C++
git clone https://github.com/your-username/rocket-flight-optimizer.git
```
```C++
cd rocket-flight-optimizer
```
```C++
//download Boost and gnuplot 
//add them to Cmakelist.txt
```
### Usage

Modify the parameters: The optimizer allows you to customize the search space for the rocket parameters in the constructor of the RocketOptimizer class.  
Visualize the results: The project includes visualization functions to display the rocket flight and the corresponding optimal parameters.  

### Example
To use the optimizer, you can call:  

```C++
RocketOptimizer optimizer(0.1, 1000, 200, 5000, 1000, 0.05, 0.01, 0.1, 0.02, 30, 10, 45, 10, 0, 50, 0, 50, 0, 50, 2.0, 0.5, 300, 100, 10, 2);
optimizer.findOptimalRocket();

vizualize(optimizer.roc_getter());
visualize3D(optimizer.roc_getter());
visualizeSeparateGraphs(optimizer.roc_getter());
```
This example configures the optimizer to search over the provided parameter ranges and visualize the results.  

### Core Components
Rocket Class: Defines a rocket with multiple parameters (velocity, mass, angles, etc.) and methods for simulating its flight.  
RocketOptimizer Class: Manages the brute-force search over the rocket's parameter space and selects the optimal configuration.  
Visualization Functions: Utility functions to visualize rocket flight data in various formats.  
