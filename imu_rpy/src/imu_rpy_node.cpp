#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "math.h"

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

ros::Publisher pub_compass;
double azimuth_deg = 0.0;

void imu_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf2::Quaternion quat;
    tf2::fromMsg(msg->orientation, quat);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    // Convert radians to degrees
    roll = RAD2DEG(roll);
    pitch = RAD2DEG(pitch);
    yaw = RAD2DEG(yaw);
    
    ROS_INFO("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr &msg)
{
    double x = msg->magnetic_field.x;
    double y = msg->magnetic_field.y;
    // 방위각 계산 (라디안)
    double azimuth = atan2(y, x);
    // 라디안을 도로 변환
    azimuth_deg = azimuth * 180.0 / M_PI;
    // 0-360도 범위로 조정
    if (azimuth_deg < 0)
    {
        azimuth_deg += 360.0;
    }
    ROS_INFO("Azimuth: %.2f degrees", azimuth_deg);

    std_msgs::Float32 compass_msg;
    compass_msg.data = azimuth_deg;
    pub_compass.publish(compass_msg);
}

int main(int argc, char **argv)
{
    int count = 0;
    ros::init(argc, argv, "imu_rpy_node");
    ros::NodeHandle nh;
    
    std::string imu_topic       = "/handsfree/imu";
    std::string mag_topic       = "/handsfree/mag";
    std::string compass_topic   = "/compass";
    
    ros::param::get("~imu_topic",       imu_topic);
    ros::param::get("~mag_topic",       mag_topic);
    ros::param::get("~compass_topic",   compass_topic);
    
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 1, imu_Callback);
    ros::Subscriber sub_imu_mag = nh.subscribe(mag_topic, 1, magCallback);
    
    pub_compass = nh.advertise<std_msgs::Float32>(compass_topic, 1);
    
    ros::Rate loop_rate(25.0);
    
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
