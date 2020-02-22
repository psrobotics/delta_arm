#include "arm_kinematics.h"

const double PI=3.1415926535;

delta_arm::delta_arm(double ml_len,double l_len,int m_num,double b_len,double e_len)
{
    motor_link_len=ml_len;
    link_len=l_len;
    motor_num=m_num;

    base_len=b_len;
    effector_len=e_len;

    for(int s=0;s<motor_num;s++)//init the pos arr
    {
        Eigen::Vector3d temp_zero_vec;
        temp_zero_vec<<0,0,0;
        base_point_arr.push_back(temp_zero_vec);
        effector_point_arr.push_back(temp_zero_vec);
        link_point_arr.push_back(temp_zero_vec);
        motor_rad.push_back(0);
        motor_angle.push_back(0);
    }
    for(unsigned int s=0;s<motor_num;s++)//init the fixed base pos
    {
        double rad_interval=2*PI/motor_num;
        base_point_arr[s][0]=base_len*cos(rad_interval*s);
        base_point_arr[s][1]=base_len*sin(rad_interval*s);
    }
    effector_pos<<0,0,0;
}

int delta_arm::set_pos(Eigen::Vector3d &in_pos)
{
    effector_pos=in_pos;
    return 0;
}

int delta_arm::inverse_cal()
{
    double rad_interval=2*PI/motor_num;
    vector<double> l1_tmp;
    vector<double> rad1_tmp;
    vector<double> l_proj_tmp;
    for(unsigned int s=0;s<motor_num;s++)
    {
        effector_point_arr[s][2]=effector_pos[2];//sync the z axis
        effector_point_arr[s][0]=effector_pos[0]+effector_len*cos(rad_interval*s);//x
        effector_point_arr[s][1]=effector_pos[1]+effector_len*sin(rad_interval*s);//y
        Eigen::Vector3d l1_vec_tmp=effector_point_arr[s]-base_point_arr[s];//cal the l1 vec

        l1_tmp.push_back(l1_vec_tmp.norm());//cal the length of l1

        double l1=l1_tmp[s];
        double l2=motor_link_len;
        double l3=link_len;
        //cout<<"l1 len "<<l1_tmp[s]<<" l2 "<<l2<<" l3 "<<l3<<endl;
        //cout<<"acos value "<<(pow(l1,2)+pow(l2,2)-pow(l3,2))/(2*l1*l2)<<endl;
        rad1_tmp.push_back(acos((pow(l1,2)+pow(l2,2)-pow(l3,2))/(2*l1*l2)));//cal the rad

        l_proj_tmp.push_back((effector_point_arr[s].dot(base_point_arr[s]))/base_point_arr[s].norm());//cal the projection len
        double rad1=atan(effector_point_arr[s][2]/abs(l_proj_tmp[s]));
        //cout<<"rad1 val "<<rad1<<endl;
        if(l_proj_tmp[s]<0)
            rad1+=PI/2;
        //cout<<"rad2 val "<<rad1_tmp[s]<<endl;
        motor_rad[s]=rad1-rad1_tmp[s];
        motor_angle[s]=motor_rad[s]/PI*180;//trans to angle set
    }

    return 0;
}

int delta_arm::print_data()
{
    cout<<"//////////////////////////////////////////"<<endl;
    cout<<"base point position: "<<endl;
    for(auto s:base_point_arr)
      cout<<s[0]<<" "<<s[1]<<" "<<s[2]<<" "<<endl;
    cout<<endl;

    cout<<"link point position: "<<endl;
    for(auto s:link_point_arr)
      cout<<s[0]<<" "<<s[1]<<" "<<s[2]<<" "<<endl;
    cout<<endl;

    cout<<"effector point position: "<<endl;
    for(auto s:effector_point_arr)
      cout<<s[0]<<" "<<s[1]<<" "<<s[2]<<" "<<endl;
    cout<<endl;

    cout<<"motor angle set: "<<endl;
    for(auto s:motor_angle)
        cout<<s<<" ";
    cout<<endl;

    cout<<"//////////////////////////////////////////"<<endl;
    return 0;
}
