#include <Eigen/Dense>

class KalmanFilter{
private:
    double dt;
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd H;

public:
    KalmanFilter(double dt);
    void init(const Eigen::VectorXd& x0);
    void predict();
    void update(const Eigen::VectorXd& z);
    Eigen::VectorXd state() const;
};