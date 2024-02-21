# N-Spring-Solver

This repository contains a numerical solver for a N-degree-of-freedom spring mass damper system in C++.

## Introduction

This solver user explicit Euler and 4th-order Runge-Kutta to numerically solve a system of ODEs and obtain the corresponding position and velocity of each mass over time.

## System of Interest 

The spring mass damper system of interest in shown below
![System_of_Interest](https://github.com/0-rion/N-Spring-Solver/assets/92618256/1b2b620c-3a83-4ebc-944b-83086176bde9)
In this system, each mass is connected to two other masses via spring, except for the ones at the end where they are connected to the wall. The damper of each mass is connected to a fixed wall. The mass $m$, spring constant $k$ and damping constant $d$ can take distinct values for each mass.

## Code Structure
This numerical solver contains two c++ files 
- [n_spring_solver.h](https://github.com/0-rion/N-Spring-Solver/blob/main/n_spring_solver.h) \
  This c++ header files contains a class called NSpring. This class contains the neccessary variables and functions for solving the spring mass damper system numerically.
- [main.cpp](https://github.com/0-rion/N-Spring-Solver/blob/main/main.cpp) \
  This file calls the n_spring_solver.h header and read the input file to configure the class NSpring. It then compute the neccesary value and create an output file.

## Input and Output
### Input
The input file to the solver is a .txt file with space separated values. The first line should contain: 
- an integer indicating the time integration scheme to use (0 = explicit Euler, 1 = 4th-order Runge-Kutta)
- T, the total time to solve the over
- ∆t, the time step size
- k, the spring constant
- d, the damping constant
