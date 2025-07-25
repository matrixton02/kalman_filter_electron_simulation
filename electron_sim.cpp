#include "electron_sim.hpp"

EMSimulator::EMSimulator(double dt):dt(dt){
    E_feild=Eigen::Vector3d::Zero();
    B_feild=Eigen::Vector3d::UnitZ();
}

void EMSimulator::setFeild(const Eigen::Vector3d& E,const Eigen::Vector3d& B){
    E_feild=E;
    B_feild=B;
}

void EMSimulator::step(ParticleState& state){
    Eigen::Vector3d force = q * (E_feild + state.velocity.cross(B_feild));
    Eigen::Vector3d acceleration = force / m;

    state.position += state.velocity * dt + 0.5 * acceleration * dt * dt;
    state.velocity += acceleration * dt;
}