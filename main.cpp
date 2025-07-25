#include <iostream>
#include <fstream>
#include <random>
#include "electron_sim.hpp"
#include "kalman.hpp"
#define M_PI 3.14159265358979323846
using namespace std;

int main(){
    constexpr double q = -1.602e-19;
    constexpr double m = 9.109e-31; 
    const double dt=1e-11;  //10 ps
    const int steps=1000;

    EMSimulator sim(dt);
    sim.setFeild(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0.01));  //B=0.0 T along z

    ParticleState state;
    state.position = Eigen::Vector3d(0, 0, 0);
    state.velocity = Eigen::Vector3d(1e5, 0, 1e5);  //defining particle inital position and velocity

    KalmanFilter kf(dt); //creating the kalman filter object
    Eigen::VectorXd x0(5); //[x,y,v,theta,omega]
    double omega = std::abs(q * 0.01 / m); // B = 1e-4 T
    x0<<0,1e5/omega,1e6,-M_PI/2,omega; //circular motion from qvB/m
    kf.init(x0);

    ofstream file("motion.csv");
    file<<"t,true_x,true_y,meas_x,meas_y,est_x,est_y\n";

    default_random_engine gen;
    normal_distribution<double> noise(0.0,1e-7); //0.0000001 mm noise

    int i=0;

    for(i=0;i<steps;i++){
        double t=i*dt;

        sim.step(state);  //calcualting next position and velocity using lorentz force

        double noisy_x=state.position(0)+noise(gen);
        double noisy_y=state.position(1)+noise(gen);  //add noise to it to mimic sensor noise

        Eigen::Vector2d meas(noisy_x,noisy_y);
        kf.predict();
        kf.update(meas);          //predicting and updating kalman filter based in noisy measurement 
        Eigen::VectorXd kf_state = kf.state();
        file << t << ","
             << state.position(0) << "," << state.position(1) << ","
             << noisy_x << "," << noisy_y << ","
             << kf_state(0) << "," << kf_state(1) << "\n";    //storing data in the file
    }

    file.close();
    cout<<"Simulation complete! Data saved to data.csv\n";

    return 0;
}