#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <math.h>

using namespace std;

class delta_arm
{
protected:
    double motor_link_len;
    double link_len;

    double base_len;
    double effector_len;

    int motor_num;

    vector<Eigen::Vector3d> base_point_arr;
    vector<Eigen::Vector3d> effector_point_arr;
    vector<Eigen::Vector3d> link_point_arr;
    Eigen::Vector3d effector_pos;

public:
    vector<double> motor_rad;
    vector<double> motor_angle;

    delta_arm(double ml_len,double l_len,int m_num,double b_len,double e_len);
    int set_pos(Eigen::Vector3d &in_pos);
    int inverse_cal();
    int print_data();
};

#endif // ARM_KINEMATICS_H
