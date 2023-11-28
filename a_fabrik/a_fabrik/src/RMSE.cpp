#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>

int main(int argc, char** argv)
{
    // translation
    Eigen::Vector3d P_t;
    Eigen::Vector3d P_f;

    // rotation
    Eigen::Matrix3d O_t;
    Eigen::Matrix3d O_f;
    Eigen::Matrix3d M;
    Eigen::Vector3d E_O;
    Eigen::Vector3d E_P;

    P_t.setZero();
    P_f.setZero();
    O_t.setZero();
    O_f.setZero();
    M.setZero();
    E_O.setZero();
    E_P.setZero();

    // input (target, final)_ee_translation 
    P_t << 1, 1, 1;
    P_f << 2, 2, 2;
    // input (target, final)_ee_rotation
    O_t << 1, 1, 1,
           1, 1, 1,
           1, 1, 1;
    O_f << 2, 2, 2,
           2, 2, 2,
           2, 2, 2;

    // get RMSE
    E_P = P_f - P_t;

    M = O_f*O_t.transpose();
    E_O << M(2,1) - M(1,2),
           M(0,2) - M(2,0),
           M(1,0) - M(0,1);
    
    double RMSE = pow(pow(E_P.norm(),2) + pow(E_O.norm(),2), 0.5)*(1/pow(6, 0.5));
    ROS_WARN_STREAM(RMSE);
    

    
    
}