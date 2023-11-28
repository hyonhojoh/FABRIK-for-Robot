#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <math.h>
#include <iostream>
#include <chrono>


using namespace std;

Eigen::Matrix3d rpy_to_rotation(double roll, double pitch, double yaw)
{
     Eigen::Matrix3d R_x;
     Eigen::Matrix3d R_y;
     Eigen::Matrix3d R_z;
     R_x << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);
     R_y << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);
     R_z << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;

     return R_z*R_y*R_x;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double wrapToPi(double angle) {
    const double twoPi = 2.0 * M_PI;

    // Use fmod to keep the angle within the range [-2*pi, 2*pi]
    angle = std::fmod(angle, twoPi);

    // Adjust angle to the range [-pi, pi]
    if (angle <= -M_PI) {
        angle += twoPi;
    } else if (angle > M_PI) {
        angle -= twoPi;
    }

    return angle;
}

int main(int argc, char** argv)
{
    // ros::init(argc, argv, "fabrik_test_ik");

    // const string franka_model_path = "/home/home/fabrik_ws/src/franka_description/";
    // vector<string> package_dirs;
    // package_dirs.push_back(franka_model_path);
    // string urdfFileName = package_dirs[0] + "robots/panda/panda.urdf";
    // RobotWrapper robot(urdfFileName, package_dirs, false);

    // const Model& model = robot.model();
    // Data data(robot.model());

    // Vector q(7), v(7);
    // q.setZero();
    // v.setZero();

    // int na =  robot.nv();

    // time start
    auto start = std::chrono::high_resolution_clock::now();
    Eigen::Matrix3d M;
    Eigen::Vector3d E_O;
    Eigen::Vector3d E_P;
    Eigen::Vector3d P_f;
    Eigen::Matrix3d O_f;

    Eigen::Vector3d P_get;
    Eigen::Matrix3d O_get;
    int N_get;
    double RMSE_get = 10.0;

    double RMSE;
    int N = 0;
    auto end = std::chrono::high_resolution_clock::now();

    M.setZero();
    E_O.setZero();
    E_P.setZero();

    // P: position, (x, y, z): rotation
    Eigen::Vector3d P_base;
    Eigen::MatrixXd P(8, 3);

    Eigen::Matrix3d O_base;
    Eigen::MatrixXd x(8, 3);
    Eigen::MatrixXd y(8, 3);
    Eigen::MatrixXd z(8, 3);
    
    Eigen::MatrixXd d(7, 3);
    Eigen::VectorXd l(7);
    Eigen::VectorXd q(7);
    Eigen::MatrixXd cons_rad(7,2);

    Eigen::Vector3d x_1_init;
    Eigen::Vector3d x_3_init;
    Eigen::Vector3d x_5_init;
    Eigen::Vector3d x_7_init;

    Eigen::Vector3d x_0_prime;
    Eigen::Vector3d x_1_prime;
    Eigen::Vector3d x_2_prime;
    Eigen::Vector3d x_3_prime;
    Eigen::Vector3d x_4_prime;
    Eigen::Vector3d x_5_prime;
    Eigen::Vector3d x_6_prime;
    Eigen::Vector3d x_7_prime;

    Eigen::Vector3d z_0_prime;
    Eigen::Vector3d z_1_prime;
    Eigen::Vector3d z_2_prime;
    Eigen::Vector3d z_3_prime;
    Eigen::Vector3d z_4_prime;
    Eigen::Vector3d z_5_prime;
    Eigen::Vector3d z_6_prime;
    Eigen::Vector3d z_7_prime;


    double l_0 = 0.02;
    double l_1 = 0.02;
    double l_2 = 0.07;
    double l_3 = 0.02;
    double l_4 = 0.07;
    double l_5 = 0.02;
    double l_6 = 0.06;
    l << l_0, l_1, l_2, l_3, l_4, l_5, l_6;

    // input initial position, rotation
    P << 0, 0, 0,                                      // 0 - twist
         0, 0, l_0,                                    // 1 - hinge
         0, 0, l_0 + l_1,                              // 2 - twist
         0, 0, l_0 + l_1 + l_2,                        // 3 - hinge
         0, 0, l_0 + l_1 + l_2 + l_3,                  // 4 - twist
         0, 0, l_0 + l_1 + l_2 + l_3 + l_4,            // 5 - hinge
         0, 0, l_0 + l_1 + l_2 + l_3 + l_4 + l_5,      // 6 - twist
         0, 0, l_0 + l_1 + l_2 + l_3 + l_4 + l_5 + l_6; // 7 - P(7)

    q.setZero();
    cons_rad << -M_PI, M_PI,
                -M_PI*5/6, M_PI*5/6,
                -M_PI, M_PI,
                -M_PI*5/6, M_PI*5/6,
                -M_PI, M_PI,
                -M_PI*5/6, M_PI*5/6,
                -M_PI, M_PI;

    P_base << 0, 0, 0;
    O_base << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;

     for (int i=0; i<8; i++){
          x.row(i) << 1, 0, 0;
          y.row(i) << 0, 1, 0;
          z.row(i) << 0, 0, 1;
     }
    
    // set target
    Eigen::Vector3d P_t;
    Eigen::MatrixXd O_t(3,3);
    double roll = M_PI*2/3;
    double pitch = -M_PI*2/3;
    double yaw = M_PI/3;

    P_t << -2, 7, 7;
    O_t = rpy_to_rotation(roll, pitch, yaw);


    ////////////////////////////    Reaching Reaching    ////////////////////////////// 

    // 1st - prismatic, 마지막 target에서 시작하는 joint를 prismatic joint라고 봤음, i = 6
    while (N < 200){
        x.row(7) = O_t.col(0);
        y.row(7) = O_t.col(1);
        z.row(7) = O_t.col(2);
        P.row(7) = P_t;
        
        // set P(7) -> target
        x.row(6) = O_t.col(0);
        y.row(6) = O_t.col(1);
        z.row(6) = O_t.col(2);

        for (int i=0; i<3; i++){
            d(6,i) = P_t(i) - P(6,i);
        }
        
        z_6_prime = z.row(6);

        // equation 12
        l(6) = d.row(6).dot(z.row(6));
        if (l(6) < 6.0){
            l(6) = 6.0;
        }
        else if (l(6) > 10.0){
            l(6) = 10.0;
        }

        P.row(6) = P_t - z_6_prime*l(6);
        
        // 2nd - twist, i = 5
        z.row(5) = z.row(6);
        P.row(5) = P.row(6) - z.row(5)*l(5);
        d.row(5) = P.row(5) - P.row(3);
        z.row(4) = d.row(5).normalized();

        z_5_prime = z.row(5);
        z_4_prime = z.row(4);

        x_5_init = x.row(5);
        x.row(5) = (z_5_prime.cross(z_4_prime)).normalized();
        if (x.row(5).norm() < 0.01){
            x.row(5) = x.row(6);
            x.row(5) = x.row(5) * sgn(x_5_init.dot(x.row(5)));
        }
        // restriction
        x_5_prime = x.row(5);   x_6_prime = x.row(6);
        q(6) = acos(x_5_prime.dot(x_6_prime))*sgn((x_5_prime.cross(x_6_prime)).dot(z_5_prime));
        q(6) = wrapToPi(q(6)); // -pi, pi 사이로 넣어줌
        if (q(6) < cons_rad(6,0)){
            x.row(5) = x.row(6)*cos(cons_rad(6,0)) - y.row(6)*sin(cons_rad(6,0));
        }
        else if (q(6) > cons_rad(6,1)){
            x.row(5) = x.row(6)*cos(cons_rad(6,1)) - y.row(6)*sin(cons_rad(6,1));
        }

        z_5_prime = z.row(5);
        x_5_prime = x.row(5);
        y.row(5) = z_5_prime.cross(x_5_prime);



        // 3rd - hinge + prismatic, i = 4
        d.row(4) = P.row(5) - P.row(4);
        z.row(4) = (d.row(4) - (d.row(4).dot(x.row(5)))*x.row(5)).normalized();

        if (z.row(4).norm() < 0.01){
            z.row(4) = z.row(5);
        }

        l(4) = d.row(4).dot(z.row(4));
        if (l(4) < 7.0){
            l(4) = 7.0;
        }
        else if (l(4) > 11.0){
            l(4) = 11.0;
        }

        P.row(4) = P.row(5) - z.row(4)*l(4);
        x.row(4) = x.row(5);

        // restriction
        z_4_prime = z.row(4);   z_5_prime = z.row(5);   x_4_prime = x.row(4);
        q(5) = acos(z_4_prime.dot(z_5_prime))*sgn((z_4_prime.cross(z_5_prime)).dot(x_4_prime));
        q(5) = wrapToPi(q(5));
        if (q(5) < cons_rad(5,0)){
            z.row(4) = z.row(5)*cos(cons_rad(5,0)) - y.row(5)*sin(cons_rad(5,0));
        }
        else if (q(5) > cons_rad(5,1)){
            z.row(4) = z.row(5)*cos(cons_rad(5,1)) - y.row(5)*sin(cons_rad(5,1));
        }

        P.row(4) = P.row(5) - z.row(4)*l(4);
        
        z_4_prime = z.row(4);
        x_4_prime = x.row(4);
        y.row(4) = z_4_prime.cross(x_4_prime);

        
        
        // 4th - twist, i = 3
        z.row(3) = z.row(4);
        P.row(3) = P.row(4) - z.row(3)*l(3);
        d.row(3) = P.row(3) - P.row(1);
        z.row(2) = d.row(3).normalized();

        z_3_prime = z.row(3);
        z_2_prime = z.row(2);

        x_3_init = x.row(3);
        x.row(3) = (z_3_prime.cross(z_2_prime)).normalized();
        if (x.row(3).norm() < 0.01){
            x.row(3) = x.row(4);
            x.row(3) = x.row(3) * sgn(x_3_init.dot(x.row(3)));
        }
        // restriction
        x_3_prime = x.row(3);   x_4_prime = x.row(4);
        q(4) = acos(x_3_prime.dot(x_4_prime))*sgn((x_3_prime.cross(x_4_prime)).dot(z_3_prime));
        q(4) = wrapToPi(q(4)); // -pi, pi 사이로 넣어줌
        if (q(4) < cons_rad(4,0)){
            x.row(3) = x.row(4)*cos(cons_rad(4,0)) - y.row(4)*sin(cons_rad(4,0));
        }
        else if (q(4) > cons_rad(4,1)){
            x.row(3) = x.row(4)*cos(cons_rad(4,1)) - y.row(4)*sin(cons_rad(4,1));
        }

        z_3_prime = z.row(3);
        x_3_prime = x.row(3);
        y.row(3) = z_3_prime.cross(x_3_prime);



        // 5th - hinge + prismatic, i = 2
        d.row(2) = P.row(3) - P.row(2);
        z.row(2) = (d.row(2) - (d.row(2).dot(x.row(3)))*x.row(3)).normalized();
        if (z.row(2).norm() < 0.01){
            z.row(2) = z.row(3);
        }

        l(2) = d.row(2).dot(z.row(2));
        if (l(2) < 7.0){
            l(2) = 7.0;
        }
        else if (l(2) > 11.0){
            l(2) = 11.0;
        }
        P.row(2) = P.row(3) - z.row(2)*l(2);
        x.row(2) = x.row(3);

        // restriction
        z_2_prime = z.row(2);   z_3_prime = z.row(3);   x_2_prime = x.row(2);
        q(3) = acos(z_2_prime.dot(z_3_prime))*sgn((z_2_prime.cross(z_3_prime)).dot(x_2_prime));
        q(3) = wrapToPi(q(3));
        if (q(3) < cons_rad(3,0)){
            z.row(2) = z.row(3)*cos(cons_rad(3,0)) - y.row(3)*sin(cons_rad(3,0));
        }
        else if (q(3) > cons_rad(3,1)){
            z.row(2) = z.row(3)*cos(cons_rad(3,1)) - y.row(3)*sin(cons_rad(3,1));
        }

        P.row(2) = P.row(3) - z.row(2)*l(2);
        z_2_prime = z.row(2);
        x_2_prime = x.row(2);
        y.row(2) = z_2_prime.cross(x_2_prime);



        // 6th - twist, P(-1)이 존재하지 않으므로 그냥 P(0)로 하겠음
        z.row(1) = z.row(2);
        P.row(1) = P.row(2) - z.row(1)*l(1);
        d.row(1) = P.row(1) - P.row(0); // P(-1)이 없으므로 P(0)로 설정
        z.row(0) = d.row(1).normalized();

        z_1_prime = z.row(1);
        z_0_prime = z.row(0);

        x_1_init = x.row(1);
        x.row(1) = (z_1_prime.cross(z_0_prime)).normalized();
        if (x.row(1).norm() < 0.01){
            x.row(1) = x.row(2);
            x.row(1) = x.row(1) * sgn(x_1_init.dot(x.row(1)));
        }
        // restriction
        x_1_prime = x.row(1);   x_2_prime = x.row(2);
        q(2) = acos(x_1_prime.dot(x_2_prime))*sgn((x_1_prime.cross(x_2_prime)).dot(z_1_prime));
        q(2) = wrapToPi(q(2)); // -pi, pi 사이로 넣어줌
        if (q(2) < cons_rad(2,0)){
            x.row(1) = x.row(2)*cos(cons_rad(2,0)) - y.row(2)*sin(cons_rad(2,0));
        }
        else if (q(4) > cons_rad(4,1)){
            x.row(1) = x.row(2)*cos(cons_rad(2,1)) - y.row(2)*sin(cons_rad(2,1));
        }

        z_1_prime = z.row(1);
        x_1_prime = x.row(1);
        y.row(1) = z_1_prime.cross(x_1_prime);

        

        // 7th - hinge
        d.row(0) = P.row(1) - P.row(0);
        z.row(0) = (d.row(0) - (d.row(0).dot(x.row(1)))*x.row(1)).normalized();
        if (z.row(0).norm() < 0.01){
            z.row(0) = z.row(1);
        }
        P.row(0) = P.row(1) - z.row(0)*l(0);
        x.row(0) = x.row(1);
        // restriction
        z_0_prime = z.row(0);   z_1_prime = z.row(1);   x_0_prime = x.row(0);
        q(1) = acos(z_0_prime.dot(z_1_prime))*sgn((z_0_prime.cross(z_1_prime)).dot(x_0_prime));
        q(1) = wrapToPi(q(1));
        if (q(1) < cons_rad(1,0)){
            z.row(0) = z.row(1)*cos(cons_rad(1,0)) - y.row(1)*sin(cons_rad(1,0));
        }
        else if (q(1) > cons_rad(1,1)){
            z.row(0) = z.row(1)*cos(cons_rad(1,1)) - y.row(1)*sin(cons_rad(1,1));
        }

        P.row(0) = P.row(1) - z.row(0)*l(0);
        z_0_prime = z.row(0);
        x_0_prime = x.row(0);
        y.row(0) = z_0_prime.cross(x_0_prime);


        


        ////////////////////////////    Forward Reaching    //////////////////////////////
        // set base
        P.row(0) = P_base;
        x.row(0) = O_base.col(0);
        y.row(1) = O_base.col(1);
        z.row(2) = O_base.col(2);

        // joint_0 - twist, i = 0
        z.row(1) = z.row(0);
        P.row(1) = P.row(0) - z.row(1)*l(0);
        d.row(0) = (P.row(1) - P.row(3));
        z.row(2) = d.row(0).normalized();

        z_1_prime = z.row(1);
        z_2_prime = z.row(2);
        x.row(1) = z_1_prime.cross(z_2_prime).normalized();

        if (x.row(1).norm() < 0.01){
            x.row(1) = x.row(0);
            x.row(1) = x.row(1) * sgn(x_1_init.dot(x.row(1)));
        }

        x_1_prime = x.row(1);
        x_0_prime = x.row(0);
        q(0) = acos(x.row(1).dot(x.row(0)))*sgn((x_1_prime.cross(x_0_prime).dot(z.row(1))));
        q(0) = wrapToPi(q(0));
        if (q(0) < cons_rad(0,0)){
            x.row(1) = x.row(0)*cos(cons_rad(0,0)) - y.row(0)*sin(cons_rad(0,0));
        }
        else if(q(0) > cons_rad(0,1)){
            x.row(1) = x.row(0)*cos(cons_rad(0,1)) - y.row(0)*sin(cons_rad(0,1));
        }

        z_1_prime = z.row(1);
        x_1_prime = x.row(1);
        y.row(1) = z_1_prime.cross(x_1_prime);



        // joint_1 - hinge, i = 1
        d.row(1) = P.row(1) - P.row(2);
        z.row(2) = (d.row(1) - (d.row(1).dot(x.row(1)))*x.row(2)).normalized();
        if (z.row(2).norm() < 0.01){
            z.row(2) = z.row(1);
        }

        P.row(2) = P.row(1) - z.row(2)*l(1);
        x.row(2) = x.row(1);

        z_2_prime = z.row(2);
        z_1_prime = z.row(1);
        q(1) = acos(z.row(2).dot(z.row(1)))*sgn((z_2_prime.cross(z_1_prime)).dot(x.row(2)));
        q(1) = wrapToPi(q(1));
        if (q(1) < cons_rad(1,0)){
            z.row(2) = z.row(1)*cos(cons_rad(1,0)) + y.row(1)*sin(cons_rad(1,0));
        }
        else if((1) > cons_rad(1,1)){
            z.row(2) = z.row(1)*cos(cons_rad(1,1)) + y.row(1)*sin(cons_rad(1,1));
        }

        P.row(2) = P.row(1) - z.row(2)*l(1);
        z_2_prime = z.row(2);
        x_2_prime = x.row(2);
        y.row(2) = z_2_prime.cross(x_2_prime);



        // joint_2 - twist + prismatic, i = 2
        z.row(3) = z.row(2);
        l(2) = d.row(2).dot(z.row(3));
        if (l(2) < 7.0){
            l(2) = 7.0;
        }
        else if (l(2) > 11.0){
            l(2) = 11.0;
        }
        P.row(3) = P.row(2) - z.row(3)*l(2);
        d.row(2) = (P.row(3) - P.row(5));
        z.row(4) = d.row(2).normalized();

        z_2_prime = z.row(3);
        z_4_prime = z.row(4);
        x.row(3) = z_3_prime.cross(z_4_prime).normalized();

        if (x.row(3).norm() < 0.01){
            x.row(3) = x.row(2);
            x.row(3) = x.row(3) * sgn(x_3_init.dot(x.row(3)));
        }

        x_3_prime = x.row(3);
        x_2_prime = x.row(2);
        q(2) = acos(x.row(3).dot(x.row(2)))*sgn((x_3_prime.cross(x_2_prime).dot(z.row(3))));
        q(2) = wrapToPi(q(2));
        if (q(2) < cons_rad(2,0)){
            x.row(3) = x.row(2)*cos(cons_rad(2,0)) - y.row(2)*sin(cons_rad(2,0));
        }
        else if(q(2) > cons_rad(2,1)){
            x.row(3) = x.row(2)*cos(cons_rad(2,1)) - y.row(2)*sin(cons_rad(2,1));
        }

        z_3_prime = z.row(3);
        x_3_prime = x.row(3);
        y.row(3) = z_3_prime.cross(x_3_prime);



        // joint_3 - hinge, i = 3
        d.row(3) = P.row(3) - P.row(4);
        z.row(4) = (d.row(3) - (d.row(3).dot(x.row(3)))*x.row(4)).normalized();
        if (z.row(4).norm() < 0.01){
            z.row(4) = z.row(3);
        }

        P.row(4) = P.row(3) - z.row(4)*l(3);
        x.row(4) = x.row(3);

        z_4_prime = z.row(4);
        z_3_prime = z.row(3);
        q(3) = acos(z.row(4).dot(z.row(3)))*sgn((z_4_prime.cross(z_3_prime)).dot(x.row(4)));
        q(3) = wrapToPi(q(3));
        if (q(3) < cons_rad(3,0)){
            z.row(4) = z.row(3)*cos(cons_rad(3,0)) + y.row(3)*sin(cons_rad(3,0));
        }
        else if(q(3) > cons_rad(3,1)){
            z.row(4) = z.row(3)*cos(cons_rad(3,1)) + y.row(3)*sin(cons_rad(3,1));
        }

        P.row(4) = P.row(3) - z.row(4)*l(3);
        z_4_prime = z.row(4);
        x_4_prime = x.row(4);
        y.row(4) = z_4_prime.cross(x_4_prime);



        // joint_4 - twist + prismatic, i = 4
        z.row(5) = z.row(4);
        l(4) = d.row(4).dot(z.row(5));
        if (l(4) < 7.0){
            l(4) = 7.0;
        }
        else if (l(4) > 11.0){
            l(4) = 11.0;
        }
        P.row(5) = P.row(4) - z.row(5)*l(4);
        d.row(4) = (P.row(5) - P.row(6));
        z.row(6) = d.row(4).normalized();

        z_4_prime = z.row(5);
        z_6_prime = z.row(6);
        x.row(5) = z_5_prime.cross(z_6_prime).normalized();

        if (x.row(5).norm() < 0.01){
            x.row(5) = x.row(4);
            x.row(5) = x.row(5) * sgn(x_5_init.dot(x.row(5)));
        }

        x_5_prime = x.row(5);
        x_4_prime = x.row(4);
        q(4) = acos(x.row(5).dot(x.row(4)))*sgn((x_5_prime.cross(x_4_prime).dot(z.row(5))));
        q(4) = wrapToPi(q(4));
        if (q(4) < cons_rad(4,0)){
            x.row(5) = x.row(4)*cos(cons_rad(4,0)) - y.row(4)*sin(cons_rad(4,0));
        }
        else if(q(4) > cons_rad(4,1)){
            x.row(5) = x.row(4)*cos(cons_rad(4,1)) - y.row(4)*sin(cons_rad(4,1));
        }

        z_5_prime = z.row(5);
        x_5_prime = x.row(5);
        y.row(5) = z_5_prime.cross(x_5_prime);



        // joint_5 - hinge, i = 5
        d.row(5) = P.row(5) - P.row(6);
        z.row(6) = (d.row(5) - (d.row(5).dot(x.row(5)))*x.row(6)).normalized();
        if (z.row(6).norm() < 0.01){
            z.row(6) = z.row(5);
        }

        P.row(6) = P.row(5) - z.row(6)*l(5);
        x.row(6) = x.row(5);

        z_6_prime = z.row(6);
        z_5_prime = z.row(5);
        q(5) = acos(z.row(6).dot(z.row(5)))*sgn((z_6_prime.cross(z_5_prime)).dot(x.row(6)));
        q(5) = wrapToPi(q(5));
        if (q(5) < cons_rad(5,0)){
            z.row(6) = z.row(5)*cos(cons_rad(5,0)) + y.row(5)*sin(cons_rad(5,0));
        }
        else if (q(5) > cons_rad(5,1)){
            z.row(6) = z.row(5)*cos(cons_rad(5,1)) + y.row(5)*sin(cons_rad(5,1));
        }

        P.row(6) = P.row(5) - z.row(6)*l(5);
        z_6_prime = z.row(6);
        x_6_prime = x.row(6);
        y.row(6) = z_6_prime.cross(x_6_prime);



        // joint_6 - twist + prismatic, i = 6
        x_7_init = x.row(7);

        z.row(7) = z.row(6);

        l(6) = d.row(6).dot(z.row(7));
        if (l(6) < 6.0){
            l(6) = 6.0;
        }
        else if (l(6) > 10.0){
            l(6) = 10.0;
        }

        P.row(7) = P.row(6) - z.row(7)*l(6);

        x.row(7) = x.row(6);
        x.row(7) = x.row(7) * sgn(x_7_init.dot(x.row(7)));
        
        x_7_prime = x.row(7);
        x_6_prime = x.row(6);
        q(6) = acos(x.row(7).dot(x.row(6)))*sgn((x_7_prime.cross(x_6_prime).dot(z.row(7))));
        q(6) = wrapToPi(q(6));
        if (q(6) < cons_rad(6,0)){
            x.row(7) = x.row(6)*cos(cons_rad(6,0)) - y.row(6)*sin(cons_rad(6,0));
        }
        else if(q(6) > cons_rad(6,1)){
            x.row(7) = x.row(6)*cos(cons_rad(6,1)) - y.row(6)*sin(cons_rad(6,1));
        }

        z_7_prime = z.row(7);
        x_7_prime = x.row(7);
        y.row(7) = z_7_prime.cross(x_7_prime);



        // get RMSE
        P_f = P.row(7);
        O_f << x(7,0), y(7,0), z(7,0),
               x(7,1), y(7,1), z(7,1),
               x(7,2), y(7,2), z(7,2);
        
        E_P = P_f - P_t;

        M = O_f*O_t.transpose();
        E_O << M(2,1) - M(1,2),
               M(0,2) - M(2,0),
               M(1,0) - M(0,1);
        
        RMSE = pow(pow(E_P.norm(),2) + pow(E_O.norm(),2), 0.5)*(1.0/pow(6, 0.5));
        
        N++;
        if (RMSE < RMSE_get){
            RMSE_get = RMSE ;
            P_get = P_f;
            O_get = O_f;
            N_get = N;
            end = std::chrono::high_resolution_clock::now();
        }

        ROS_WARN_STREAM("E_P");
        ROS_WARN_STREAM(pow(E_P.norm(),2));
        ROS_WARN_STREAM("E_O");
        ROS_WARN_STREAM(pow(E_O.norm(),2));
       
        ROS_WARN_STREAM("N");
        ROS_WARN_STREAM(N);
        ROS_INFO_STREAM("RMSE");
        ROS_INFO_STREAM(RMSE);
        

        // iterate condition
        if (RMSE < 0.1){
            return 0;
        }
    }
    // time end
    
    std::chrono::duration<double> duration = end - start;
    
    ROS_INFO_STREAM("---------------- Best result ----------------");
    ROS_INFO_STREAM("number of iteration");
    ROS_INFO_STREAM(N_get);
    ROS_INFO_STREAM("Best RMSE");
    ROS_INFO_STREAM(RMSE_get);
    ROS_INFO_STREAM("Best Position");
    ROS_INFO_STREAM(P_get);
    ROS_INFO_STREAM("Best Orientation");
    ROS_INFO_STREAM(O_get);
    ROS_INFO_STREAM("time");
    ROS_INFO_STREAM(duration.count());
    
    
}