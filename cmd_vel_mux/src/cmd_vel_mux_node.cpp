#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

enum CMD_TYPE
{
   VISION_CONTROL = 0,
   YAW_CONTROL,
   SONAR_CONTROL,
   LIDAR_CONTROL,
}type_cmd_vel;
   
//int type_cmd_vel;

geometry_msgs::Twist cmd_vel_line;
geometry_msgs::Twist cmd_vel_yaw;
geometry_msgs::Twist cmd_vel_sonar;

///////////// VISION /////////////
void VISION_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
   cmd_vel_line = *msg;
}

///////////// YAW /////////////
void YAW_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
   cmd_vel_yaw = *msg;
}

///////////// SONAR /////////////
void SONAR_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
   cmd_vel_sonar = *msg;
}

///////////// type_cmd_vel /////////////
void type_cmd_vel_Callback(const std_msgs::Int8::ConstPtr &msg)
{
   //type_cmd_vel = msg->data;
   type_cmd_vel = static_cast<CMD_TYPE>(msg->data);
}

int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "cmd_vel_mux");
    ros::NodeHandle nh;

    std::string line_cmd_vel_topic          = "/cmd_vel/line";
    std::string yaw_cmd_vel_topic          = "/cmd_vel/yaw";
    std::string sonar_cmd_vel_topic         = "/cmd_vel/sonar";
    std::string type_cmd_vel_topic         = "/type_cmd_vel";
    std::string ackermann_cmd_vel_topic    = "/ackermann_steering_controller/cmd_vel";

    ros::param::get("~line_cmd_vel_topic",       line_cmd_vel_topic);
    ros::param::get("~yaw_cmd_vel_topic",       yaw_cmd_vel_topic);
    ros::param::get("~sonar_cmd_vel_topic",    sonar_cmd_vel_topic);
    ros::param::get("~type_cmd_vel_topic",       type_cmd_vel_topic);
    ros::param::get("~ackermann_cmd_vel_topic", ackermann_cmd_vel_topic);

    ros::Subscriber sub_line_cmd_vel       = nh.subscribe(line_cmd_vel_topic, 1, VISION_Callback);
    ros::Subscriber sub_yaw_cmd_vel       = nh.subscribe(yaw_cmd_vel_topic, 1, YAW_Callback);
    ros::Subscriber sub_sonar_cmd_vel       = nh.subscribe(sonar_cmd_vel_topic, 1, SONAR_Callback);
    ros::Subscriber sub_type_cmd_vel       = nh.subscribe(type_cmd_vel_topic, 1, type_cmd_vel_Callback);
    
    ros::Publisher pub_ackermann_cmd_vel    = nh.advertise<geometry_msgs::Twist>(ackermann_cmd_vel_topic, 1);
    
    ros::Rate loop_rate(30.0); // 100HZ

    while (ros::ok())
    {
      geometry_msgs::Twist cmd_vel_rqt_robot_steering;
      //std_msgs::Int8 type_cmd_vel_pub;

        switch (type_cmd_vel)
        {
            case VISION_CONTROL:
                cmd_vel_rqt_robot_steering = cmd_vel_line;
            printf("VISION_CONTROL: Linear = %.2f, Angular = %f\n", cmd_vel_rqt_robot_steering.linear.x, cmd_vel_rqt_robot_steering.angular.z);
            //type_cmd_vel_pub.data = 1;
                break;
            
            case YAW_CONTROL:
                cmd_vel_rqt_robot_steering = cmd_vel_yaw;
                printf("YAW_CONTROL: Linear = %.2f, Angular = %f\n", cmd_vel_rqt_robot_steering.linear.x, cmd_vel_rqt_robot_steering.angular.z);
                break; 

            case SONAR_CONTROL:
                cmd_vel_rqt_robot_steering = cmd_vel_sonar;
                printf("SONAR_CONTROL: Linear = %.2f, Angular = %f\n", cmd_vel_rqt_robot_steering.linear.x, cmd_vel_rqt_robot_steering.angular.z);
                break;

            default:
            printf("NOTICE : type_cmd_vel\n");
                break;
        }

        pub_ackermann_cmd_vel.publish(cmd_vel_rqt_robot_steering);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
