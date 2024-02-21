#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "n_spring_solver.h"

int main(){
    vector<double> v_setup(5); // vector to store first line
    vector<double> v_mass; // mass vector
    vector<double> vi_position; // initial position vector
    vector<double> vi_velocity; // initial velocity vector

    ifstream input("parameters.txt"); // open input file
    for(int i=0;i<5;i++){ // read the first line (5 doubles)
        input >> v_setup[i];
    }
    double m,x,v;
    while(input >> m >> x >> v){ // read until the end of file
        v_mass.push_back(m);
        vi_position.push_back(x);
        vi_velocity.push_back(v);
    }

    const int scheme = v_setup[0]; // numerical scheme
    const double T = v_setup[1]; // final time
    const double dt = v_setup[2]; // time step
    const int t_length = T/dt + 1; // length of time range
    const int num_mass = v_mass.size(); // number of masses

    vector<double> v_time(t_length);
    vector<vector<double> > v_position(t_length,vector<double>(v_mass.size()));
    vector<vector<double> > v_velocity(t_length,vector<double>(v_mass.size()));

    // create object with initial condition
    NSpring n_spring(v_setup,v_mass,vi_position,vi_velocity);
    v_time = n_spring.time_range();
    if(scheme == 0){
        n_spring.explicit_euler(v_position,v_velocity);
    }
    else if(scheme == 1){
        n_spring.rk_4(v_position,v_velocity);
    }

    // output to file
    ofstream v_out("output.txt", ios::out | ios::trunc);
    if(v_out.is_open()){
        v_out.precision(10);
        for(int i=0;i<t_length;i++){
            v_out << v_time[i];
            for(int j=0;j<num_mass;j++){
                v_out << " " << v_position[i][j] << " " << v_velocity[i][j];
            }
            v_out << endl;
        }
        v_out.close();
    }

    return -1;
}