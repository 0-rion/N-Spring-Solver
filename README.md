# N-Spring-Solver

This repository contains a numerical solver for a N-degree-of-freedom spring mass damper system in C++.

## Introduction

This solver user explicit Euler and 4th-order Runge-Kutta to numerically solve a system of ODEs and obtain the corresponding position and velocity of each mass over time.

## System of Interest 

The spring mass damper system of interest in shown below
![System_of_Interest](https://github.com/0-rion/N-Spring-Solver/assets/92618256/20167886-2539-4923-8560-5087868c2f2a)
In this system, each mass is connected to two other masses via spring, except for the ones at the end where they are connected to the wall. The damper of each mass is connected to a fixed wall. The mass $m$ can take distinct values for each mass, while spring constant $k$ and damping constant $c$ are the same through out the system.

## Code Structure
This numerical solver contains two c++ files 
- [n_spring_solver.h](https://github.com/0-rion/N-Spring-Solver/blob/main/n_spring_solver.h) \
  This c++ header files contains a class called NSpring. This class contains the neccessary variables and functions for solving the spring mass damper system numerically.
- [main.cpp](https://github.com/0-rion/N-Spring-Solver/blob/main/main.cpp) \
  This file calls the n_spring_solver.h header and read the input file to configure the class NSpring. It then compute the neccesary value and create an output file.

## Input and Output
### Input File
The input file to the solver is a .txt file with space separated values. The file should be name as _parameters.txt_. 

The first line should contain: 
- an integer indicating the time integration scheme to use (0 = explicit Euler, 1 = 4th-order Runge-Kutta)
- $T$, the total time to solve the over
- $∆t$, the time step size
- $k$, the spring constant
- $d$, the damping constant
where each value should be separated by a white space. 

Each subsequent line should describe one mass in the system, containing:
- its mass $m_i$
- initial displacement from its equilibrium position $x_i(0)$
- initial velocity $\dot x_i(0)$

A sample input file is provided [here](https://github.com/0-rion/N-Spring-Solver/blob/main/parameters.txt)

### Output File
The solver output a file called _output.txt_ to the directory of the solver. The output file is a space separated file containg the displacement and velocity of each mass at each time step. Each line of the _output.txt_ contains the displacement and velocity of N masses a single time step in the format: $t$ &emsp; $x_1(t)$ &emsp; $\dot x_1(t)$ &emsp; . . . &emsp; $x_N (t$) &emsp; $\dot x_N(t)$.

