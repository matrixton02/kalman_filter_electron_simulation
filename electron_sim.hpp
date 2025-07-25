#include <Eigen/Dense>

struct ParticleState{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

class EMSimulator{
private:
    double dt;
    Eigen::Vector3d E_feild;
    Eigen::Vector3d B_feild;
    const double q=-1.602e-19;
    const double m=9.100e-31;

public:
    EMSimulator(double dt);
    void setFeild(const Eigen::Vector3d& E,const Eigen::Vector3d& B);
    void step(ParticleState& state);
};