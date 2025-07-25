#include "kalman.hpp"
#include <cmath>

using namespace std;

KalmanFilter::KalmanFilter(double dt):dt(dt),x(5),P(5,5),Q(5,5),R(2,2),H(2,5){
    Q=Eigen::MatrixXd::Identity(5,5);
    Q*=1e-16;
    // Q(0,0) = 1e-6;  // x
    // Q(1,1) = 1e-6;  // y
    // Q(2,2) = 1e-1;    // velocity
    // Q(3,3) = 1e-4;   // angle
    // Q(4,4) = 1e2;    // omega

    R = Eigen::Matrix2d::Identity() * 1e-18; 

    H.setZero();
    H(0,0)=1;
    H(1,1)=1;
}

void KalmanFilter::init(const Eigen::VectorXd& x0){
    x=x0;
}

void KalmanFilter::predict(){
    double x_=x(0);
    double y=x(1);
    double v=x(2);
    double theta=x(3);
    double omega=x(4);

    Eigen::VectorXd x_pred(5);

    if(abs(omega)>1e-5){
        x_pred(0)=x_+(v/omega)*(sin(theta+omega*dt)-sin(theta));
        x_pred(1)=y+(v/omega)*(-cos(theta+omega*dt)+cos(theta));
    }
    else{
        x_pred(0)=x_+v*cos(theta)*dt;
        x_pred(1)=y+v*sin(theta)*dt;
    }

    x_pred(2)=v;
    x_pred(3)=theta+omega*dt;
    x_pred(3)=atan2(sin(x_pred(3)),cos(x_pred(3)));  // Normalize angle

    x_pred(4)=omega;

    x=x_pred;

    Eigen::MatrixXd F=Eigen::MatrixXd::Identity(5,5);

    if(abs(omega)>1e-5){
        F(0,2)=(1/omega)*(sin(theta+omega*dt)-sin(theta));
        F(0,3)=(v/omega)*(cos(theta+omega*dt)-cos(theta));
        F(0,4)=(v/(omega*omega))*(sin(theta)-sin(theta+omega*dt)+(v*dt/omega)*cos(theta+omega*dt));
        F(1,2)=(1/omega)*(-cos(theta+omega*dt)+cos(theta));
        F(1,3)=(v/omega)*(sin(theta+omega*dt)-sin(theta));
        F(1,4)=(v/(omega*omega)*(cos(theta+omega*dt)-cos(theta))+(v*dt/omega)*sin(theta+omega*dt));
    }
    else{
        F(0,2)=cos(theta)*dt;
        F(0,3)=-v*sin(theta)*dt;

        F(1, 2)=sin(theta)*dt;
        F(1, 3)=v*cos(theta)*dt;
    }

    P=F*P*F.transpose()+Q;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
    Eigen::VectorXd y=z-H*x;
    Eigen::MatrixXd S=H*P*H.transpose()+R;
    Eigen::MatrixXd K=P*H.transpose()*S.inverse();

    x=x+K*y;
    P=(Eigen::MatrixXd::Identity(5, 5)-K*H)*P;
}

Eigen::VectorXd KalmanFilter::state() const {
    return x;
}