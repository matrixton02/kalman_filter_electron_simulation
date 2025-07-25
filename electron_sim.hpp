#include <Eigen/Dense>

struct ParticleState{
    Eigen::Vector3d position;  //Creating vector to store postion and velocity
    Eigen::Vector3d velocity;
};

class EMSimulator{
private:
    double dt;
    Eigen::Vector3d E_feild;  //Vector to represent Electric field
    Eigen::Vector3d B_feild;  //Vector to represent Magnetic field
    const double q=-1.602e-19; //charge of electron
    const double m=9.100e-31;  //mass of electron

public:     //fucntion definition of class EMSimulator
    EMSimulator(double dt);
    void setFeild(const Eigen::Vector3d& E,const Eigen::Vector3d& B);
    void step(ParticleState& state);
};