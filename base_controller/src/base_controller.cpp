/******************************************************************
基于串口通信的ROS小车基础控制器，功能如下：
1.实现ros控制数据通过固定的格式和串口通信，从而达到控制小车的移动
2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动
3.发布里程计主题/odm

串口通信说明：
1.写入串口
（1）内容：左右轮速度，单位为mm/s
（2）格式：１3字节,[0x55][0x08][右轮速度４字节][左轮速度４字节][cka1字节][ckb1字节][0x0a]
2.读取串口
（1）内容：小车x,y坐标，方向角，线速度，角速度，角度，单位依次为：mm,mm,rad,mm/s,rad/s, rad
（2）格式：２9字节，[0x55][0x18][Ｘ坐标４字节][Ｙ坐标４字节][方向角４字节][线速度４字节][角速度４字节][前轮转角４字节][cka1字节][ckb1字节][0x0a]
*******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//以下为串口通讯需要的头文件
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
/*****************************************************************************/
float ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
float zhouju = 0.55f;
float D = 0.445f ;    //两轮间距，单位是m
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
static double last_angle = 0;
/****************************************************/
unsigned char data_terminal0=0x0d;  //“/r"字符
unsigned char data_terminal1=0x0a;  //“/n"字符
unsigned char speed_data[13]={0};   //要发给串口的数据
string rec_buffer;  //串口数据接收变量



unsigned short CheckSUM(const char *Buffer, unsigned short length)
{
    unsigned short CK_A = 0;
    unsigned short CK_B = 0;
    char i;
    for(i=2; i<length+2; i++)
    {
        CK_A = CK_A+Buffer[i];
        CK_B = CK_B+CK_A;
    }
    CK_A = CK_A & 0xff;
    CK_B = CK_B & 0xff;

    return  ((CK_A<<8) | CK_B)  ;
}

//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
}right_speed_data,left_speed_data,back_speed_data,angular_f,position_x,position_y,oriention,vel_linear,vel_angular,wheel_angular,body_angular;
/************************************************************/
char receive_data[27];


/********
void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    string port("/dev/ttyTHS2");    //小车串口号
    unsigned long baud = 57600;    //小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //配置串口

    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s
    //linear_temp = 0.09;

    back_speed_data.d = linear_temp;
//    ROS_INFO_STREAM("vel: " << back_speed_data.d);
//    cout<<"速度："<<back_speed_data.d;

    back_speed_data.d *=ratio;
    if(linear_temp == 0){
        angular_f.d = 0;
    } else
        angular_f.d = atan(angular_temp * zhouju/linear_temp);
//    float beta = asin((angular_temp * zhouju)/2/linear_temp);

//    angular_f.d = 0;
//    ROS_INFO_STREAM("ang: " << angular_f.d * 180 / 3.14<<"  "<<"vel:"<<linear_temp);
//    cout<<"角度："<<angular_f.d * 180 / 3.14;
//    angular_f.d *= ratio;

    for(int i=0;i<4;i++){
        speed_data[i]=back_speed_data.data[i];
        speed_data[i+4]=angular_f.data[i];
    }
    //在写入串口的左右轮速度数据后加入”/r/n“

    speed_data[8] = (CheckSUM((const char*)speed_data, 8)>>8) & 0xff;
    speed_data[9] = CheckSUM((const char*)speed_data, 8) & 0xff;
//    ROS_INFO_STREAM("CKA:"<<(float)speed_data[8]<<"CKB:"<<(float)speed_data[9]);
    speed_data[10]=data_terminal0;
    speed_data[11]=data_terminal1;
    //写入数据到串口
    my_serial.write(speed_data,12);



    //将转换好的小车速度分量为左右轮速度
//    left_speed_data.d = linear_temp - 0.5f*angular_temp*D ;
//    right_speed_data.d = linear_temp + 0.5f*angular_temp*D ;

    //存入数据到要发布的左右轮速度消息
//    left_speed_data.d*=ratio;   //放大１０００倍，mm/s
//    right_speed_data.d*=ratio;//放大１０００倍，mm/s

//    for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
//    {
//        speed_data[i]=right_speed_data.data[i];
//        speed_data[i+4]=left_speed_data.data[i];
//    }

    //在写入串口的左右轮速度数据后加入”/r/n“
//    speed_data[8]=data_terminal0;
//    speed_data[9]=data_terminal1;
    //写入数据到串口
//    my_serial.write(speed_data,10);


}
******/



void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    string port("/dev/ttyTHS2");    //小车串口号
    unsigned long baud = 57600;    //小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //配置串口


    back_speed_data.d = cmd_input.linear.x;
//    ROS_INFO_STREAM("vel: " << back_speed_data.d);
//    cout<<"速度："<<back_speed_data.d;

    back_speed_data.d *= ratio;


    angular_f.d = cmd_input.angular.z;
//    ROS_INFO_STREAM("ang: " << angular_f.d * 180 / 3.14);
//    cout<<"wheel："<<angular_f.d * 180 / 3.14;
//    angular_f.d *= ratio;

    speed_data[0] = 0x55;
    speed_data[1] = 0x08;
    for (int i = 2; i < 6; i++) {
        speed_data[i] = back_speed_data.data[i - 2];
        speed_data[i + 4] = angular_f.data[i - 2];
    }
    //在写入串口的左右轮速度数据后加入”/r/n“
    speed_data[10] = (CheckSUM((const char*)speed_data, 8)>>8) & 0xff;
    speed_data[11] = CheckSUM((const char*)speed_data, 8) & 0xff;
    speed_data[12] = 0x0a;
    //写入数据到串口
    my_serial.write(speed_data, 13);
    /*
    //将转换好的小车速度分量为左右轮速度
    left_speed_data.d = linear_temp - 0.5f*angular_temp*D ;
    right_speed_data.d = linear_temp + 0.5f*angular_temp*D ;

    //存入数据到要发布的左右轮速度消息
    left_speed_data.d*=ratio;   //放大１０００倍，mm/s
    right_speed_data.d*=ratio;//放大１０００倍，mm/s

    for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
    {
        speed_data[i]=right_speed_data.data[i];
        speed_data[i+4]=left_speed_data.data[i];
    }

    //在写入串口的左右轮速度数据后加入”/r/n“
    speed_data[8]=data_terminal0;
    speed_data[9]=data_terminal1;
    //写入数据到串口
    my_serial.write(speed_data,10);

     } */
     }


int main(int argc, char **argv)
{

    string port("/dev/ttyTHS2");//小车串口号
    unsigned long baud = 57600;//小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));//配置串口

    ros::init(argc, argv, "base_controller");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
    ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback); //订阅/cmd_vel主题
    ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("odom", 1); //定义要发布/odom主题


    static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
    geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
    nav_msgs::Odometry odom;//定义里程计对象
    geometry_msgs::Quaternion odom_quat; //四元数变量
    //定义covariance矩阵，作用为解决文职和速度的不同测量的不确定性
//    float pose_covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
//                            0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
//                            0,  0,    99999, 0,     0,     0,  // covariance on gps_z
//                            0,  0,    0,     99999, 0,     0,  // large covariance on rot x
//                            0,  0,    0,     0,     99999, 0,  // large covariance on rot y
//                            0,  0,    0,     0,     0,     0.01};  // large covariance on rot z

    float pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e3};

    float pose_covariance1[36] =  {1e-9, 0, 0, 0, 0, 0,
            0, 1e-3, 1e-9, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-9};

    float twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e3};

    float twist_covariance1[36] =  {1e-9, 0, 0, 0, 0, 0,
            0, 1e-3, 1e-9, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-9};
                            //载入covariance矩阵
//    for(int i = 0; i < 36; i++)
//    {
//        odom.pose.covariance[i] = covariance[i];
//    }

    ros::Rate loop_rate(50);//设置周期休眠时间20Hz
    int i =0, j=0;//丢包的数量
    while(ros::ok())
    {

        if(my_serial.available()) {

            string str_in = my_serial.read(my_serial.available());
            unsigned int str_length = str_in.size();
            static unsigned char chrTemp[2000];
//            static unsigned char ucRxCnt = 0;
            static unsigned int usRxLength = 0;

            memcpy(chrTemp,str_in.data(),str_length);
            usRxLength += str_length;
            while (usRxLength >= 29) {
                if (chrTemp[0] != 0x55) {
                    usRxLength--;
                    memcpy(&chrTemp[0], &chrTemp[1], usRxLength);
                    continue;
                }
                memcpy(receive_data, &chrTemp[0], 29);

                unsigned short ck;
                char cka, ckb;
                cka = (CheckSUM(receive_data, 24)>>8) & 0xff;
                ckb = CheckSUM(receive_data, 24) & 0xff;

                if(receive_data[1] == 0x18 && receive_data[26] == cka && receive_data[27] == ckb && receive_data[28] == 0x0a){
                    j++;
                    for (int i = 0; i < 4; i++)//提取X，Y坐标，方向，线速度，角速度
                    {
                        position_x.data[i] = receive_data[i + 2];
                        position_y.data[i] = receive_data[i + 6];
                        oriention.data[i] = receive_data[i + 10];
                        vel_linear.data[i] = receive_data[i + 14];
                        vel_angular.data[i] = receive_data[i + 18];
                        wheel_angular.data[i] = receive_data[i + 22];
                    }
                    //将X，Y坐标，线速度缩小1000倍
                    //cout<<"position_x: "<<position_x.d<<endl;
                    position_x.d /= 1000; //m
                    position_y.d /= 1000; //m
                    vel_linear.d /= 1000; //m/s

                    //里程计的偏航角需要转换成四元数才能发布
                    odom_quat = tf::createQuaternionMsgFromYaw(oriention.d);//将偏航角转换成四元数
                    ROS_INFO_STREAM(
                            "wheel: " << wheel_angular.d * 180 / 3.14159 << "  " << "body_ang:" << oriention.d * 180 / 3.1415926
                                      << "x:" << position_x.d << "y:" << position_y.d);
                    //载入坐标（tf）变换时间戳
                    odom_trans.header.stamp = ros::Time::now();
                    //发布坐标变换的父子坐标系
                    odom_trans.header.frame_id = "odom";
                    odom_trans.child_frame_id = "base_footprint";
                    //tf位置数据：x,y,z,方向
                    odom_trans.transform.translation.x = position_x.d;
                    odom_trans.transform.translation.y = position_y.d;
                    odom_trans.transform.translation.z = 0.0;
                    odom_trans.transform.rotation = odom_quat;
                    //发布tf坐标变化
                    odom_broadcaster.sendTransform(odom_trans);

                    //载入里程计时间戳
                    odom.header.stamp = ros::Time::now();
                    //里程计的父子坐标系
                    odom.header.frame_id = "odom";
                    odom.child_frame_id = "base_footprint";
                    //里程计位置数据：x,y,z,方向
                    odom.pose.pose.position.x = position_x.d;
                    odom.pose.pose.position.y = position_y.d;
                    odom.pose.pose.position.z = 0.0;
                    odom.pose.pose.orientation = odom_quat;
                    //载入线速度和角速度
                    odom.twist.twist.linear.x = vel_linear.d;
                    //odom.twist.twist.linear.y = odom_vy;
                    odom.twist.twist.angular.z = wheel_angular.d;

                    if (vel_linear.d == 0) {
                        for (int i = 0; i < 36; i++) {
                            odom.pose.covariance[i] = pose_covariance[i];
                            odom.twist.covariance[i] = twist_covariance[i];
                        }
                    } else {
                        for (int i = 0; i < 36; i++) {
                            odom.pose.covariance[i] = pose_covariance1[i];
                            odom.twist.covariance[i] = twist_covariance1[i];
                        }
                    }
                    //发布里程计
                    odom_pub.publish(odom);
                    break;

                }else{
                    i++;
                    usRxLength -= 29;
                    memcpy(&chrTemp[0],&chrTemp[29],usRxLength);
                }

            }
            ROS_INFO_STREAM("LOST DATA: "<< i<<"  "<<"accept data:"<<j<<endl);

//            ROS_INFO_STREAM("calculate_a: "<<(float)cka<<"  "<<"calculate_b"<<(float)ckb<<"recieve_a: "<<(float)receive_data[24]<<"recieve_b:"<<(float)receive_data[25]);



//            if (last_angle == 0) {
//                last_angle = oriention.d;
//            }
//
//            if (abs(oriention.d - last_angle) > 10 / 180 * 3.1415 && abs(oriention.d - last_angle) < 350 / 180 * 3.1415)
//                oriention.d = last_angle;
//            else
//                last_angle = oriention.d;
        }
        ros::spinOnce();//周期执行
        loop_rate.sleep();//周期休眠

    }
    return 0;
}
