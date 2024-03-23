#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double front_sonar = 0.0;
double left_sonar = 0.0;
double right_sonar = 0.0;

double Kp_wall = 0.0;
double Ki_wall = 0.0;
double Kd_wall = 0.0;

double control_speed_sonar;

void control_speed_sonar_Callback(const std_msgs::Float64::ConstPtr& msg)
{
   control_speed_sonar = msg->data;
}

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  front_sonar = msg->range;
  printf("Front_Sonar Range: [%f]\n", front_sonar);
}

void Left_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  left_sonar = msg->range;
  printf("Left_Sonar Range:  [%f]\n", left_sonar);
}

void Right_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  right_sonar = msg->range;
  printf("Right_Sonar Range: [%f]\n\n", right_sonar);
}

double error_old = 0.0;

geometry_msgs::Twist PID_wall_following(double Kp_wall, double Ki_wall, double Kd_wall)
{
   geometry_msgs::Twist cmd_vel;

   double error = left_sonar - right_sonar;
   double error_d = error - error_old;  
   double error_sum = 0.0;
   error_sum += error;  
  
   double steering_control = Kp_wall * error + Ki_wall * error_sum + Kd_wall * error_d;
   /* 
   if (front_sonar < 1.2)
   {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
   }
   else
   {
      cmd_vel.linear.x = 0.5;
      cmd_vel.angular.z = steering_control;
   }
  */
   cmd_vel.linear.x = control_speed_sonar;
   cmd_vel.angular.z = steering_control;
    
   if(error >= 180)  error = error - 360;
   if(error <=-180)  error = error + 360;

   error_old = error;
  
   return cmd_vel;
}

int main(int argc, char **argv)
{
   int count = 0;
  
   ros::init(argc, argv, "wall_following");
   ros::NodeHandle n;
  
   std::string front_sonar_topic             = "/range_front";
   std::string left_sonar_topic               = "/range_front_left";
   std::string right_sonar_topic              = "/range_front_right";
   std::string sonar_cmd_vel_topic             = "/cmd_vel/sonar";
   std::string control_speed_sonar_topic       = "/control_speed/sonar";
  
   ros::param::get("~front_sonar_topic",            front_sonar_topic);
   ros::param::get("~left_sonar_topic",            left_sonar_topic);
   ros::param::get("~right_sonar_topic",            right_sonar_topic);
   ros::param::get("~sonar_cmd_vel_topic",            sonar_cmd_vel_topic);
   ros::param::get("~control_speed_sonar_topic",      control_speed_sonar_topic);

   ros::param::get("~Kp_wall", Kp_wall);  
   ros::param::get("~Kd_wall", Kd_wall);  
   ros::param::get("~Ki_wall", Ki_wall);
   ros::param::get("~Ki_wall", Ki_wall);
  
   ros::Subscriber front_sonar_sub          = n.subscribe(front_sonar_topic, 1, Front_Sonar_Callback);
   ros::Subscriber left_sonar_sub             = n.subscribe(left_sonar_topic, 1, Left_Sonar_Callback);
   ros::Subscriber right_sonar_sub          = n.subscribe(right_sonar_topic, 1, Right_Sonar_Callback);
   ros::Subscriber sub_control_speed_sonar       = n.subscribe(control_speed_sonar_topic, 1, control_speed_sonar_Callback);
  
   ros::Publisher sonar_cmd_vel_pub = n.advertise<geometry_msgs::Twist>(sonar_cmd_vel_topic, 1);

   ros::Rate loop_rate(30.0);  

   while (ros::ok())
   {
      geometry_msgs::Twist cmd_vel = PID_wall_following(Kp_wall, Ki_wall, Kd_wall);
      sonar_cmd_vel_pub.publish(cmd_vel);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
   }
   return 0;
}
