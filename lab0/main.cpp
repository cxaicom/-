#define _USE_MATH_DEFINES
#include<cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include<iostream>

int main(){
    Eigen::Vector3d p(2,1,1);
    Eigen::Matrix3d rot;
    double theta = (45.0/180.0)*M_PI;
    rot << cos(theta),(-1.0)*sin(theta),1,
           sin(theta),cos(theta),2,
           0,0,1;
    std::cout<<rot*p<<std::endl;
    return 0;
}