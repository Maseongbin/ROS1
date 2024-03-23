#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "math.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_waypoint/Target_waypoint_line.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

enum FLAG_TYPE 
{
	Reset = -1,
	Stop,
	Run,
	Arrival
} flag_control;

#define NEW_WAYPOINT_TOR_XY    0.01
#define NEW_WAYPOINT_TOR_THETA 5.0

geometry_msgs::Pose2D start_pose2D;
geometry_msgs::Pose2D cur_pose2D;
geometry_msgs::Pose2D target_pose2D;
geometry_msgs::Pose2D target_waypont_goal;
geometry_msgs::Pose2D target_waypoint_old = cur_pose2D;
geometry_msgs::Pose2D target_waypoint;
move_waypoint::Target_waypoint_line m_target_move;

double target_move_speed = 0.25;

double r, p, y;

typedef struct
{
   double a;
   double b;
   double c;
   double d;
} Line_Equation;

typedef struct
{
   double x;
   double y;
} Vector_2D;

Line_Equation find_waypoint_line_equation(geometry_msgs::Pose2D m_target_pose2D, geometry_msgs::Pose2D m_start_pose2D, double &angle)
{
    Line_Equation line_equ;
    
    line_equ.a = ((m_target_pose2D.y - m_start_pose2D.y) / (m_target_pose2D.x - m_start_pose2D.x));
    line_equ.b = (m_start_pose2D.y - (line_equ.a * m_start_pose2D.x));
    line_equ.c = ((m_target_pose2D.x - m_start_pose2D.x) / (m_target_pose2D.y - m_start_pose2D.y));
    line_equ.d = (m_start_pose2D.x - (line_equ.c * m_start_pose2D.y));
    
    angle = atan2((m_target_pose2D.y - m_start_pose2D.y), (m_target_pose2D.x - m_start_pose2D.x));
    
    return line_equ;
}

Line_Equation find_cross_waypoint_line_equation(Line_Equation wp_line_equ, geometry_msgs::Pose2D m_cur_pose2D)
{
    Line_Equation cross_line_equ;

    cross_line_equ.a = -wp_line_equ.c;
    cross_line_equ.b = (m_cur_pose2D.y - (cross_line_equ.a * m_cur_pose2D.x));
    cross_line_equ.c = -wp_line_equ.a;
    cross_line_equ.d = (m_cur_pose2D.x - (cross_line_equ.c * m_cur_pose2D.y));

    return cross_line_equ;
}

Line_Equation find_waypoint_finish_line_equation(Line_Equation wp_line_equ, geometry_msgs::Pose2D m_target_pose2D, double &angle)
{
   Line_Equation finish_line_equ;

    finish_line_equ.a = -wp_line_equ.c;
    finish_line_equ.b = (m_target_pose2D.y - (finish_line_equ.a * m_target_pose2D.x));
    finish_line_equ.c = -wp_line_equ.a;
    finish_line_equ.d = (m_target_pose2D.x - (finish_line_equ.c * m_target_pose2D.y));
    
    angle = atan(finish_line_equ.a);

    return finish_line_equ;
}

bool check_pass_finish_line(geometry_msgs::Pose2D m_target_pose2D, geometry_msgs::Pose2D m_start_pose2D, geometry_msgs::Pose2D m_cur_pose2D, double ahead_finish_distance, double &xte)
{
   bool pass = false;
   
   Vector_2D vector_a;
   Vector_2D vector_b;
   
   double vector_angle        = 0.0;
   double inner_product, mag_vector_a, mag_vector_b;
   double distance_to_target  = 0.0;
   double cross_product       = 0.0;
   
   vector_a.x = (m_start_pose2D.x - m_target_pose2D.x);
   vector_a.y = (m_start_pose2D.y - m_target_pose2D.y);

   //printf("Vector_a : [%.2lf %.2lf]\n", vector_a.x, vector_a.y);

   vector_b.x = (m_cur_pose2D.x - m_target_pose2D.x);
   vector_b.y = (m_cur_pose2D.y - m_target_pose2D.y);

   //printf("Vector_b : [%.2lf %.2lf]\n", vector_b.x, vector_b.y);

   mag_vector_a  = sqrt(vector_a.x * vector_a.x + vector_a.y * vector_a.y); 
   mag_vector_b  = sqrt(vector_b.x * vector_b.x + vector_b.y * vector_b.y);
   
   inner_product = vector_a.x * vector_b.x + vector_a.y * vector_b.y;
   
   vector_angle  = acos(inner_product / (mag_vector_a * mag_vector_b));
   
   distance_to_target = mag_vector_b * cos(vector_angle);
   
   xte = mag_vector_b * sin(vector_angle);
   
   cross_product = vector_a.x * vector_b.y - vector_a.y * vector_b.x;
   
   if(cross_product < 0)
   {
      xte = -xte;
   }
   else
   {
      xte = xte;
   }

   //printf("mag_vector_a : %.2lf, mag_vector_b : %.2lf, inner_product : %.2lf\n", mag_vector_a, mag_vector_b, inner_product); 
   printf("vector_angle : %.2lf, distance_to_target : %.2lf, XTE : %.2lf\n", RAD2DEG(vector_angle), distance_to_target, xte); 

   if(distance_to_target <= ahead_finish_distance)
   {
      printf("target : reach\n");
      pass = true;
   }
   else
   {
      //printf("target : not reach\n");
      pass = false;
   }
   
   return pass;
}

void poseupdate_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    cur_pose2D.x     = msg->pose.pose.position.x;
    cur_pose2D.y     = msg->pose.pose.position.y;
    cur_pose2D.theta = msg->pose.pose.position.z;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    m.getRPY(r, p, y);

    //printf("Received pose in = X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f\n", tx, ty, tz, RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
}

void flag_control_Callback(const std_msgs::Int8::ConstPtr& msg)
{
   //flag_control = msg->data;
   flag_control = static_cast<FLAG_TYPE>(msg->data);
   //printf("flag_control: %d\n", flag_control);
}

geometry_msgs::Pose2D target_waypoint_new;

void target_waypoint_new_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
   target_waypoint_new = *msg;

   //printf("target_waypoint: x = %f, y = %f, theta = %f\n", target_waypoint.x, target_waypoint.y, target_waypoint.theta);
}

bool flag_new_waypoint = false;

int  flag_waypoint_control;

void target_waypoint_line_Callback(const move_waypoint::Target_waypoint_line::ConstPtr &msg)
{
	start_pose2D  = msg->waypoint_start_pose2d;
	
	target_pose2D = msg->waypoint_target_pose2d;
}

void move_base_simple_goal_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double diff_x = target_waypoint_old.x - msg->pose.position.x;
    double diff_y = target_waypoint_old.y - msg->pose.position.y;
    double diff_theta;
    
    double roll, pitch, yaw;
    double theta;
    
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    tf2::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
    
    theta = RAD2DEG(yaw);
    diff_theta = target_waypoint_old.theta - theta;
     
    if ((fabs(diff_x) <= NEW_WAYPOINT_TOR_XY) && (fabs(diff_y) <= NEW_WAYPOINT_TOR_XY) && (fabs(diff_theta) <= NEW_WAYPOINT_TOR_THETA))
    {   
		flag_new_waypoint   = false;
		printf("no new waypoint!\n");
    }
    else
    {  
		target_waypoint.x         = msg->pose.position.x;
		target_waypoint.y         = msg->pose.position.y;
		target_waypoint.theta     = theta;
		
		target_waypoint_old.x     = msg->pose.position.x;
		target_waypoint_old.y     = msg->pose.position.y;
		target_waypoint_old.theta = theta;
	
		start_pose2D              = cur_pose2D;
		flag_control              = Run;
		
		printf("target_waypoint [%.2lf, %.2lf, %.2lf]\n", target_waypoint.x, target_waypoint.y, target_waypoint.theta);  
		flag_new_waypoint =true;
    }
}


void target_waypoint_Callback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    if (*msg != target_waypoint_old)
    {   
		target_waypoint     = *msg;
		target_waypoint_old = *msg;
		start_pose2D        = cur_pose2D;
        printf("target_waypoint [%.2lf, %.2lf, %.2lf]\n", target_waypoint.x, target_waypoint.y, target_waypoint.theta);
        flag_new_waypoint   = true;
    }
    else
    {
        double diff_x     = target_waypoint_old.x     - cur_pose2D.x;
        double diff_y     = target_waypoint_old.y     - cur_pose2D.y;
        double diff_theta = target_waypoint_old.theta - cur_pose2D.theta;

        if ((fabs(diff_x) <= NEW_WAYPOINT_TOR_XY) && (fabs(diff_y) <= NEW_WAYPOINT_TOR_XY) && (fabs(diff_theta) <= NEW_WAYPOINT_TOR_THETA))
        {   
			target_waypoint     = *msg;
			target_waypoint_old = *msg;
			start_pose2D        = cur_pose2D;
            printf("target_waypoint_New [%.2lf, %.2lf, %.2lf]\n", target_waypoint_new.x, target_waypoint_new.y, target_waypoint_new.theta);  
			flag_new_waypoint   = true;
        }
        else
        {   
			flag_new_waypoint =false;
        }
    }
}

/*
void move_target_Callback(const move_waypoint::target_waypoint::ConstPtr &target_msg)
{
	start_pose2D = target_msg->start_pose2D;
	
	target_pose2D   = target_msg->target_pose2D;
}
*/

int main(int argc, char **argv)
{
    int count    = 0;
	double K_xte = 0.2;

	Line_Equation waypoint_line;
    Line_Equation cross_waypoint_line;
    Line_Equation finish_waypoint_line;
   
    ros::init(argc, argv, "move_waypoint");
    ros::NodeHandle n;

    std::string poseupdate_topic            = "/poseupdate";
    std::string target_waypoint_topic       = "/target_waypoint";
    std::string flag_waypoint_control_topic = "/flag/waypoint_control";
    std::string type_cmd_vel_topic          = "/type_cmd_vel";
    std::string status_waypoint_move_topic  = "/status/waypoint_move";
    std::string target_yaw_topic            = "/target_yaw/radian";
    std::string target_yaw_degree_topic     = "/target_yaw/degree";
    std::string control_speed_yaw_topic     = "/control_speed/yaw";
    std::string target_waypoint_new_topic   = "/target_waypoint_new";
    std::string move_base_simple_goal_topic = "/move_base_simple/goal";
    std::string target_waypoint_goal_topic  = "/target_waypoint_goal";
    std::string target_waypoint_line_topic  = "/wp/target_waypoint_line";
    
    ros::param::get("~K_xte",                       K_xte);
    ros::param::get("~control_speed_yaw_topic",     control_speed_yaw_topic);
    ros::param::get("~poseupdate_topic",            poseupdate_topic);
    ros::param::get("~target_waypoint_topic",       target_waypoint_topic);
    ros::param::get("~target_waypoint_new_topic",   target_waypoint_new_topic);
    ros::param::get("~flag_waypoint_control_topic", flag_waypoint_control_topic);
    ros::param::get("~type_cmd_vel_topic",          type_cmd_vel_topic);
    ros::param::get("~status_waypoint_move_topic",  status_waypoint_move_topic);
    ros::param::get("~target_yaw_topic",            target_yaw_topic);
    ros::param::get("~target_yaw_degree_topic",     target_yaw_degree_topic);
    ros::param::get("~move_base_simple_goal_topic", move_base_simple_goal_topic);
    ros::param::get("~target_waypoint_goal_topic",  target_waypoint_goal_topic);
    ros::param::get("~target_waypoint_line_topic", target_waypoint_line_topic);
    
    ros::Subscriber sub_poseupdate              = n.subscribe(poseupdate_topic, 1, poseupdate_Callback);
    ros::Subscriber sub_target_waypoint         = n.subscribe(target_waypoint_topic, 1, target_waypoint_Callback);
    ros::Subscriber sub_target_waypoint_new     = n.subscribe(target_waypoint_new_topic, 1, target_waypoint_new_Callback);
    ros::Subscriber sub_flag_waypoint_control   = n.subscribe(flag_waypoint_control_topic, 1, flag_control_Callback);
    ros::Subscriber sub_move_base_simple_goal   = n.subscribe(move_base_simple_goal_topic, 1, move_base_simple_goal_Callback);
    //ros::Subscriber sub_custom_msg            = n.subscribe(target_waypoint_line_topic, 1, target_waypoint_line_Callback);
    ros::Subscriber sub_target_waypoint_line    = n.subscribe(target_waypoint_line_topic, 1, target_waypoint_line_Callback);
    
    ros::Publisher pub_target_yaw               = n.advertise<std_msgs::Float64>(target_yaw_topic, 1);
    ros::Publisher pub_target_yaw_degree        = n.advertise<std_msgs::Float64>(target_yaw_degree_topic, 1);
    ros::Publisher pub_status_waypoint_move     = n.advertise<std_msgs::Int8>(status_waypoint_move_topic, 1);
    ros::Publisher pub_type_cmd_vel             = n.advertise<std_msgs::Int8>(type_cmd_vel_topic, 1);    
    ros::Publisher pub_control_speed_yaw        = n.advertise<std_msgs::Float64>(control_speed_yaw_topic, 1);
    ros::Publisher pub_target_waypoint_goal     = n.advertise<std_msgs::Float64>(target_waypoint_goal_topic, 1);
    
    ros::Rate loop_rate(30.0); // 100HZ

   while (ros::ok())
   {
	  std_msgs::Float64 control_speed_yaw;
	  std_msgs::Int8 status_waypoint_move;
	  std_msgs::Int8 type_cmd_vel;
	  std_msgs::Float64 target_yaw;
	  std_msgs::Float64 target_yaw_degree;
      //std_msgs::Float64 target_waypoint_line;
      
      geometry_msgs::Pose2D m_target;
      geometry_msgs::Pose2D m_current;
      geometry_msgs::Pose2D m_start;
        
      double wp_angle;
      double wp_finish_angle = 0.0;
      double ahead_finish_distance;
      double waypoint_finish_line;
      double target_yaw_angle = 0.0;
      double xte;
        
      //printf("Status_move: %d\n",status_move.data);
      //printf("flag_control: %d\n",flag_control);
        
      if(flag_control == Run)
      {
         m_target.x      = target_pose2D.x;
         m_target.y      = target_pose2D.y;
         m_start.x       = start_pose2D.x;
         m_start.y       = start_pose2D.y;
         m_current.x     = cur_pose2D.x;
         m_current.y     = cur_pose2D.y;
         m_current.theta = cur_pose2D.theta;

         printf("Target Pose  : [%.2lf, %.2lf]\n", m_target.x, m_target.y);
         printf("Current Pose : [%.2lf, %.2lf]\n", m_current.x, m_current.y);
         printf("Start Pose   : [%.2lf, %.2lf]\n", m_start.x, m_start.y);
      
         waypoint_line = find_waypoint_line_equation(m_target, m_start, wp_angle);

         if (isnan(waypoint_line.a) == 1)
         {
            //printf("infinity\n");
         }

         cross_waypoint_line  = find_cross_waypoint_line_equation(waypoint_line, m_current);
   
         finish_waypoint_line = find_waypoint_finish_line_equation(waypoint_line, m_target, wp_finish_angle);

         waypoint_finish_line = check_pass_finish_line(m_target, m_start, m_current, ahead_finish_distance, xte);
         
         if (check_pass_finish_line(m_target, m_start, m_current, ahead_finish_distance, xte) == true) 
         {
            status_waypoint_move.data = Arrival;
            pub_status_waypoint_move.publish(status_waypoint_move);

            control_speed_yaw.data = 0.0;
            pub_control_speed_yaw.publish(control_speed_yaw);

            target_yaw_angle = wp_angle;
            target_yaw.data  = target_yaw_angle;
            pub_target_yaw.publish(target_yaw);

            target_yaw_degree.data = RAD2DEG(target_yaw_angle);
            pub_target_yaw_degree.publish(target_yaw_degree);
         
            printf("target_speed : %.2lf\n\n", control_speed_yaw.data);
         }
         
         else
         {
            status_waypoint_move.data = Run;
            pub_status_waypoint_move.publish(status_waypoint_move);
         
            control_speed_yaw.data = target_move_speed;
            pub_control_speed_yaw.publish(control_speed_yaw);       
            
            target_yaw_angle = wp_angle + (K_xte * xte);
            target_yaw.data  = target_yaw_angle;
            pub_target_yaw.publish(target_yaw);

            target_yaw_degree.data = RAD2DEG(target_yaw_angle);
            pub_target_yaw_degree.publish(target_yaw_degree);
            
            printf("target_speed : %.2lf\n", control_speed_yaw.data);
            printf("Target Angle : %.2lf(Radion)\n", target_yaw.data);
            printf("Target Angle : %.2lf(degree)\n\n", target_yaw_degree.data);
         }   
      }
      
      else if(flag_control == Stop)
      {
         status_waypoint_move.data = Stop;
         pub_status_waypoint_move.publish(status_waypoint_move);
         
         control_speed_yaw.data = 0.0;
         pub_control_speed_yaw.publish(control_speed_yaw);
         
         printf("============================================\n");
         printf("================== STOP ====================\n");
         printf("============================================\n\n");

      }
      
      else if(flag_control == Reset)
      {   
         status_waypoint_move.data = Reset;
         pub_status_waypoint_move.publish(status_waypoint_move);
         
         control_speed_yaw.data = 0.0;
         pub_control_speed_yaw.publish(control_speed_yaw);
         
         printf("============================================\n\n");
         printf("================== RESET ===================\n");
         printf("============================================\n\n");

         target_waypoint_new = target_waypoint;
          
      }
      
      else
      {
         
      }
      
      //printf("flag_control: %d\n",flag_control);

      type_cmd_vel.data = 1;

      //pub_target_waypoint_line.publish(target_waypoint_line);
      pub_target_yaw.publish(target_yaw);
      pub_target_yaw_degree.publish(target_yaw_degree);
      pub_status_waypoint_move.publish(status_waypoint_move);
      pub_control_speed_yaw.publish(control_speed_yaw);
      pub_type_cmd_vel.publish(type_cmd_vel);
      pub_target_waypoint_goal.publish(target_waypont_goal);
      
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
   }
   return 0;
}
