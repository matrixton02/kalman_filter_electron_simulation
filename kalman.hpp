#include <Eigen/Dense>

class KalmanFilter{
private:  //decelaration of vector x P Q R H for our Kalman filter
    double dt;
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd H;

public:  //function declaration for kalman filter
    KalmanFilter(double dt);
    void init(const Eigen::VectorXd& x0);
    void predict();
    void update(const Eigen::VectorXd& z);
    Eigen::VectorXd state() const;
};