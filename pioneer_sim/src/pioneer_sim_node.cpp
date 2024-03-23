#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h" 
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "sensor_msgs/Range.h"
#include <math.h>

#define TSL1401CL_SIZE 320
#define THRESHOLD 0.01

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

int mission_flag = 0;

///////////////////////////////////// line /////////////////////////////////////

double tsl1401cl_data[TSL1401CL_SIZE];
int LineSensor_threshold_Data[TSL1401CL_SIZE];
double line_error_old = 0.0;

void threshold(double tsl1401cl_data[], int ThresholdData[], int tsl1401cl_size, double threshold)
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        if (tsl1401cl_data[i] > threshold)
        {
            ThresholdData[i] = 255;
        }
        else
        {
            ThresholdData[i] = 0;
        }
    }
}

int find_line_center()
{
    double centroid = 0.0;
    double mass_sum = 0.0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        mass_sum += LineSensor_threshold_Data[i];
        centroid += LineSensor_threshold_Data[i] * i;
    }

    if (mass_sum == 0)
    {
        return 0.0;
    }

    centroid = centroid / mass_sum;

    return centroid;
}

void tsl1401cl_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg) 
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        tsl1401cl_data[i] = msg->data[i];
    }
    printf("Threshold Data: \n");

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        printf("%d ", LineSensor_threshold_Data[i]);
    }
    printf("\n");

    double centroid = find_line_center();
    printf("Line Centroid: %f\n", centroid);

    threshold(tsl1401cl_data, LineSensor_threshold_Data, TSL1401CL_SIZE, THRESHOLD);
}

geometry_msgs::Twist lane_control(double line_Kp, double line_Ki, double line_Kd)
{
    geometry_msgs::Twist cmd_vel;

    double line_error = 0.0;
    double line_error_d = 0.0;
    double line_error_sum = 0.0;
    double line_steering_angle = 0.0;
    double line_center = 160.0;
    double offset = -13.0;

    line_error = line_center - find_line_center() + offset;
    line_error_d = line_error - line_error_old;
    line_error_sum += line_error;

    line_steering_angle = (line_Kp * line_error + line_Kd * line_error_d + line_Ki * line_error_sum);

    cmd_vel.linear.x = 0.2;
    cmd_vel.angular.z = line_steering_angle;

    line_error_old = line_error;

    return cmd_vel;
}

///////////////////////////////////// yaw /////////////////////////////////////

double roll, pitch, yaw;
double yaw_error_old = 0.0;
double target_heading_yaw = 0.0;

double Constrain_Yaw(double yaw_deg)
{
    if (yaw_deg > 360)
    {
        yaw_deg = yaw_deg - 360;
    }
    else if (yaw_deg < 0)
    {
        yaw_deg = yaw_deg + 360;
    }

    return yaw_deg;
}

geometry_msgs::Twist target_yaw_control(double yaw_Kp, double yaw_Ki, double yaw_Kd, double target_heading_yaw)
{
    geometry_msgs::Twist cmd_vel;

    double yaw_deg = RAD2DEG(yaw);

    yaw_deg = Constrain_Yaw(yaw_deg);

    double yaw_error = target_heading_yaw - yaw_deg;
    double yaw_error_d = yaw_error - yaw_error_old;
    double yaw_error_sum = 0.0;
    yaw_error_sum += yaw_error;

    if (yaw_error > 180)
    {
        yaw_error = yaw_error - 360;
    }
    else if (yaw_error < -180)
    {
        yaw_error = yaw_error + 360;
    }

    double Steering_Angle = yaw_Kp * yaw_error + yaw_Ki * yaw_error_sum + yaw_Kd * yaw_error_d;

    cmd_vel.linear.x = 0.2;
    cmd_vel.angular.z = Steering_Angle;

    if (fabs(yaw_error) < 1.0)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        mission_flag++;
    }

    yaw_error_old = yaw_error;

    return cmd_vel;
}

void imu1Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    double yaw_deg = Constrain_Yaw(RAD2DEG(yaw));

    printf("%f\n", yaw_deg); 
}

///////////////////////////////////// wall /////////////////////////////////////

double range_front = 0.0;
double range_front_left = 0.0;
double range_front_right = 0.0;

double wall_error_old = 0.0;

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    range_front = msg->range;
    printf("Range_Front_Sonar: %f\n", range_front);
}

void Left_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    range_front_left = msg->range;
    printf("Range_Left_Sonar : %f\n", range_front_left);
}

void Right_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    range_front_right = msg->range;
    printf("Range_Right_Sonar: %f\n\n", range_front_right);
}

geometry_msgs::Twist wall_following(double wall_Kp, double wall_Ki, double wall_Kd)
{
    geometry_msgs::Twist cmd_vel;

    double wall_error = range_front_left - range_front_right;
    double wall_error_d = wall_error - wall_error_old;
    double wall_error_sum = 0.0;
    wall_error_sum += wall_error;

    double steering_control = wall_Kp * wall_error + wall_Ki * wall_error_sum + wall_Kd * wall_error_d;

    if (range_front < 1.5)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    else
    {
        cmd_vel.linear.x = 0.2;
        cmd_vel.angular.z = steering_control;
    }

    wall_error_old = wall_error;

    return cmd_vel;
}

///////////////////////////////////// main /////////////////////////////////////

int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "pioneer_sim_node"); 
    ros::NodeHandle n;

    ros::Subscriber tsl1401cl_sub   = n.subscribe("/tsl1401cl", 10, tsl1401cl_Callback);
    ros::Subscriber imu_sub         = n.subscribe("/imu", 1000, imu1Callback);
    ros::Subscriber front_sonar_sub = n.subscribe("/range_front", 1000, Front_Sonar_Callback);
    ros::Subscriber left_sonar_sub  = n.subscribe("/range_front_left", 1000, Left_Sonar_Callback);
    ros::Subscriber right_sonar_sub = n.subscribe("/range_front_right", 1000, Right_Sonar_Callback);

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30.0);
    /*
    속도가 1.0일때 pid
    double line_Kp = 0.002;
    double line_Ki = 0.0;
    double line_Kd = 0.01;

    double yaw_Kp = 0.02;
    double yaw_Ki = 0.0;
    double yaw_Kd = 0.3;

    double wall_Kp = 0.158;
    double wall_Ki = 0.0;
    double wall_Kd = 1.3;
    */
    
    // map 생성 시 pid
    double line_Kp = 0.0025;
    double line_Ki = 0.0;
    double line_Kd = 0.01;

    double yaw_Kp = 0.02;
    double yaw_Ki = 0.0;
    double yaw_Kd = 0.3;

    double wall_Kp = 0.158;
    double wall_Ki = 0.0;
    double wall_Kd = 1.3;
    
    geometry_msgs::Twist cmd_vel; 

    while (ros::ok())
    {
        printf("flag: %d\n", mission_flag);
        
        switch (mission_flag)
        {
        case 0:

            if (find_line_center() == 0.0)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                mission_flag++;
            }
            break;

        case 1:

            if (find_line_center() != 0)
            {
                cmd_vel = lane_control(line_Kp, line_Ki, line_Kd);
            }
            else
            {
                mission_flag++;
            }
            break;

        case 2:

            if (range_front > 0.9)
            {
                cmd_vel.linear.x = 0.2;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                target_heading_yaw = 270.0;
                mission_flag++;
            }
            break;

        case 3:

            cmd_vel = target_yaw_control(yaw_Kp, yaw_Ki, yaw_Kd, target_heading_yaw);
            break;

        case 4:

            cmd_vel = wall_following(wall_Kp, wall_Ki, wall_Kd);
            
            if (range_front > 1.0)
            {
                cmd_vel.linear.x = 0.2;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                target_heading_yaw = 180.0;
                mission_flag++;
            }
            break;

        case 5:

            cmd_vel = target_yaw_control(yaw_Kp, yaw_Ki, yaw_Kd, target_heading_yaw);
            break;

        case 6:

            bool stop = true;
            for (int i = 0; i < TSL1401CL_SIZE; i++)
            {
                if (LineSensor_threshold_Data[i] != 255)
                {
                    stop = false;
                    break;
                }
            }

            if (stop)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                cmd_vel = lane_control(line_Kp, line_Ki, line_Kd);
            }
            break;
        }
        cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
