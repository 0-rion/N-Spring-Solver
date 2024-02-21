#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
using namespace std;

class NSpring {
    public:

        NSpring(vector<double> setup, vector<double> mass, vector<double> i_position, vector<double> i_velocity){
            v_setup = setup; // first line of input file
            v_mass = mass; // masses
            vi_position = i_position; // initial positions
            vi_velocity = i_velocity; // initial velocities
            T = v_setup[1]; // final time
            dt = v_setup[2]; // time step
            k = v_setup[3]; // spring constant
            d = v_setup[4]; // damping constant
            t_length = T/dt + 1; // length of time step
            num_mass = v_mass.size(); // number of masses
            K = make_K(); // populate matrix K
            M_inverse = make_M_inverse();  // populate mass matrix inverse
            vector<double> temp_damp(num_mass,d);
            v_damping = temp_damp; // populate damping vector (matrix)

        }

        // function to calculate time range
        vector<double> time_range(){ 
            vector<double> v_time(t_length);
            vector<double>::iterator x;
            double val;
            for (x = v_time.begin(), val = 0; x != v_time.end(); x++, val += dt) {
                *x = val;
            }
            return v_time;
        }

        // explicit euler, answer pass by reference
        void explicit_euler(vector<vector<double> > &v_position,vector<vector<double> > &v_velocity){
            v_position[0] = vi_position;
            v_velocity[0] = vi_velocity;
            for(int i=1;i<t_length;i++){
                v_position[i] = vector_vector_sum(v_position[i-1],vector_scalar_product(v_velocity[i-1],dt));
                v_velocity[i] = velocity_euler(v_position[i-1],v_velocity[i-1]);
            }
        }

        // 4th-order Runge-Kutta, answer pass by reference
        void rk_4(vector<vector<double> > &v_position,vector<vector<double> > &v_velocity){
            v_position[0] = vi_position;
            v_velocity[0] = vi_velocity;
            vector< vector<double> > v_pv(t_length,vector<double>(2*num_mass)); // combined column matrix of displacement and velocity
            vector<double> vi(num_mass);
            vi = vi_position;
            vi.insert(vi.end(), vi_velocity.begin(), vi_velocity.end());
            v_pv[0] = vi;
            for(int i=1;i<t_length;i++){
                // v_position[i] = get_4rk_position(v_position[i-1], v_velocity[i-1]);
                v_pv[i] = get_4rk_velocity(v_pv[i-1]);
                v_position[i] = vector<double>(v_pv[i].begin(), v_pv[i].begin() + num_mass);
                v_velocity[i] = vector<double>(v_pv[i].begin() + num_mass, v_pv[i].end());
            }
        }



    private:
    
        // variable initialization
        vector<double> v_setup = vector<double>(5); // vector to store first line
        vector<double> v_mass; // mass vector
        vector<double> vi_position; // initial position vector
        vector<double> vi_velocity; // initial velocity vector

        double T; // final time
        double dt; // time step
        double k; // spring constant
        double d; // damping constant
        int t_length; 
        int num_mass;

        vector<double> v_time; // time range vector
        vector<vector<double> > K; // stiffness matrix
        vector<vector<double> > M_inverse; // mass matrix inverse
        vector<double> v_damping; // damping vector(matrix)

        // element wise sum
        vector<double> vector_vector_sum(vector<double> v1,vector<double> v2){
            std::transform (v1.begin(), v1.end(), v2.begin(), v1.begin(), std::plus<double>());
            return v1;
        }

        // element wise subtraction
        vector<double> vector_vector_subtract(vector<double> v1,vector<double> v2){
            std::transform (v1.begin(), v1.end(), v2.begin(), v1.begin(), std::minus<double>());
            return v1;
        }

        // element wise multiplication
        vector<double> vector_vector_multiply(vector<double> v1,vector<double> v2){
            std::transform (v1.begin(), v1.end(), v2.begin(), v1.begin(), std::multiplies<double>());
            return v1;
        }

        // element wise division
        vector<double> vector_vector_division(vector<double> v1,vector<double> v2){
            std::transform (v1.begin(), v1.end(), v2.begin(), v1.begin(), std::divides<double>());
            return v1;
        }

        // vector times a scalar
        vector<double> vector_scalar_product(vector<double> v,double x){
            for (int i=0;i<v.size();i++){
                v[i] *= x;
            }
            return v;
        }

        // matrix multiplication
        void matrix_multiply(vector<vector<double> > A, vector<vector<double> > B,vector<vector<double> > &C){ 
            int r1 = A[0].size();
            int c2 = B.size();
            int c1 = A.size();
            for(int i = 0; i < r1; i++)
                for(int j = 0; j < c2; j++)
                    for(int k = 0; k < c1; k++)
                    {
                        C[i][j] += A[i][k] * B[k][j];
                    }
        }

        // matrix multiply a column vector
        void matrix_vector_multiply(vector<vector<double> > A, vector<double> v,vector<double> &B){
            for(int i=0;i<v.size();i++){
                for(int j=0;j<v.size();j++){
                    B[i] += A[i][j] * v[j];
                }
            }
        }

        // populate stiffness matrix K
        vector<vector<double> > make_K(){
            vector<vector<double> > K(num_mass,vector<double>(num_mass));
            if(K.size() == 1){ // for single mass
                K[0][0] = 2*k;
            }
            else{ // for more than two masses
                double main_d = 2*k;
                for(int i = 0; i < K.size(); i++){
                    if (i<K.size()-1) {
                        K[i][i+1] = -k;
                    }
                    K[i][i] = main_d;
                    if (i>0) {
                        K[i][i-1] = -k;
                    }
                }
            }
            return K;
        }

        // calculating inverse of mass matrix
        vector<vector<double> > make_M_inverse(){
            vector<vector<double> > M_inverse(num_mass,vector<double>(num_mass));
            for(int i = 0; i < num_mass; i++){
                M_inverse[i][i] = 1/v_mass[i];
            }
            return M_inverse;
        }

        // calculate the velocity at each time step using explict euler
        vector<double> velocity_euler(vector<double> v_position, vector<double> v_velocity){
            vector<double> A(num_mass);
            A = vector_vector_multiply(vector_vector_division(v_damping,v_mass),v_velocity);

            vector<vector<double> > B(num_mass,vector<double>(num_mass));
            matrix_multiply(M_inverse,K,B);;

            vector<double> C(num_mass);
            matrix_vector_multiply(B,v_position,C);

            A = vector_scalar_product(vector_vector_sum(C,A),dt);;

            return vector_vector_subtract(v_velocity,A);
        }

        // get intermediate constant for RK4
        vector<double> get_k_velocity(vector<double> v_pv){
            vector<double> v_p = vector<double>(v_pv.begin(), v_pv.begin()+num_mass);
            vector<double> v_v = vector<double>(v_pv.begin()+num_mass, v_pv.end());
            vector<double> A(num_mass);
            A = vector_vector_multiply(vector_vector_division(v_damping,v_mass),v_v);

            vector<vector<double> > B(num_mass,vector<double>(num_mass));
            matrix_multiply(M_inverse,K,B);

            vector<double> C(num_mass);
            matrix_vector_multiply(B,v_p,C);

            A = vector_vector_sum(C,A);
            A = vector_scalar_product(A,-1.0);

            v_v.insert(v_v.end(), A.begin(), A.end());

            return  vector_scalar_product(v_v,dt);
        }

        // get value for each step of RK4
        vector<double> get_4rk_velocity(vector<double> v_pv){
            vector<double> k1;
            vector<double> k2;
            vector<double> k3;
            vector<double> k4;
            k1 = get_k_velocity(v_pv);
            k2 = get_k_velocity(vector_vector_sum(v_pv,vector_scalar_product(k1,0.5)));
            k3 = get_k_velocity(vector_vector_sum(v_pv,vector_scalar_product(k2,0.5)));
            k4 = get_k_velocity(vector_vector_sum(v_pv,k3));
            vector<double> kk = vector_vector_sum(vector_scalar_product(k4,1.0/6), vector_scalar_product(k1,1.0/6));
            vector<double> ki = vector_scalar_product(vector_vector_sum(k2,k3),1.0/3);
            return vector_vector_sum(v_pv,vector_vector_sum(kk,ki));

        }
};