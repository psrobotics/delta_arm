#include <iostream>
#include <sstream>
#include "arm_kinematics.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QString>
#include <QDebug>
#include <QtCore>
#include <math.h>

using namespace std;

int send_data(QSerialPort *port_sel, delta_arm *arm_sel)
{
    int len=30;//data buffer length
    char *buffer=new char[len]{'\0'};
    stringstream con_tmp;//used for format convert

    con_tmp<<'h'<<arm_sel->motor_angle[0]<<'a'<<arm_sel->motor_angle[1]<<'a'<<arm_sel->motor_angle[2]<<'i';
    con_tmp>>buffer;

    port_sel->write(buffer,20);//sent data
    port_sel->flush();//important, flush the serial port

    cout<<"data sent "<<con_tmp.str()<<endl;
    /*for(int s=0;s<len;s++)
        cout<<buffer[s];
    cout<<endl<<endl;*/

    delete[] buffer;
    return 0;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QStringList m_serial_port_name;
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {
        m_serial_port_name << info.portName();
        cout<<"serial port name: "<<info.portName().toStdString().data()<<endl;
    }

    QSerialPort *m_serialPort = new QSerialPort();//setup a serial port

    if(m_serialPort->isOpen())//if port is opened, then close first
    {
        m_serialPort->clear();
        m_serialPort->close();
    }
    //m_serialPort->setPort(port_info->portName());
    m_serialPort->setPortName(m_serial_port_name[0]);
    m_serialPort->setBaudRate(QSerialPort::Baud115200);//set the bandrate and reading direction
    m_serialPort->setDataBits(QSerialPort::Data8);		//database as 8
   // m_serialPort->setFlowControl(QSerialPort::NoFlowControl);//non flow control
    m_serialPort->setParity(QSerialPort::NoParity);	//no check bit
    m_serialPort->setStopBits(QSerialPort::OneStop); //one stop set

    m_serialPort->open(QIODevice::WriteOnly);
    if(m_serialPort->isOpen() && m_serialPort->isWritable()) cout<<"serial device connected "<<endl<<endl;

    delta_arm *arm_test=new delta_arm(110,170,3,85.221,40);//setup a new delta arm file

    Eigen::Vector3d pos_tmp;
    int st_arr_x[10]={0,0,0,0,0,0,0,0,0,0};
    int st_arr_y[10]={0,0,0,0,0,0,0,0,0,0};
    int st_arr_z[10]={180,270,200,250,180,270,180,270,180,180};
    int st=0;

    int s;
    cin>>s;

    while(true)
    {
        if(st==10)
            st=0;

        pos_tmp[0]=st_arr_x[st];
        pos_tmp[1]=st_arr_y[st];
        pos_tmp[2]=st_arr_z[st];

        //pos_tmp[0]=r_tmp*cos(counter);
        //pos_tmp[1]=r_tmp*sin(counter);

        //cout<<"type in the test pos"<<endl;
        //cin>>pos_tmp[0]>>pos_tmp[1]>>pos_tmp[2];

        arm_test->set_pos(pos_tmp);
        arm_test->inverse_cal();
        arm_test->print_data();
        //cout<<pos_tmp[0]<<" "<<pos_tmp[1]<<" "<<pos_tmp[2]<<endl;

        if(m_serialPort->isOpen() && m_serialPort->isWritable())
        {
                send_data(m_serialPort,arm_test);
                m_serialPort->waitForBytesWritten(700);
        }
        st++;
    }

    return a.exec();
}
