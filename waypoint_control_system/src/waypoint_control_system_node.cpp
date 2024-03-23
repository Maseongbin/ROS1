#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
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
#define MAX_WAYPOINT_NUM       300
#define NEW_WAYPOINT_TOR_XY   0.01
#define NEW_WAYPOINT_TOR_THETA 5.0

enum FLAG_TYPE
{
    RESET = -1,
    STOP,
    RUN,
    ARRIVAL
}flag_control;

double target_move_speed   = 0.25;

double roll, pitch, yaw;

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

double tx,ty,tz;

void poseupdate_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    tx = msg->pose.pose.position.x;
    ty = msg->pose.pose.position.y;
    tz = msg->pose.pose.position.z;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
}

move_waypoint::Target_waypoint_line waypoints[MAX_WAYPOINT_NUM];

bool flag_new_waypoint     = false;
bool flag_new_start_id     = false;
bool flag_new_finish_id    = false;

int  flag_waypoint_control = STOP;
int status_waypoint_move   = 0;
int no_waypoints           = -1;
int waypoint_start_id      = -1;
int waypoint_finish_id     = 0;
int waypoint_start_id_old  = -1;
int waypoint_finish_id_old = 0;

void status_waypoint_move_Callback(const std_msgs::Int8::ConstPtr &msg)
{
   status_waypoint_move = msg->data;
}

void waypoint_start_id_Callback(const std_msgs::Int8 &msg)
{
	waypoint_start_id = msg.data;
	
	if(waypoint_start_id != waypoint_start_id_old)
	{
		flag_new_start_id     = true;
		waypoint_start_id_old = waypoint_start_id;
		printf("new start id is received!\n");
	}
	else
	{
		flag_new_start_id = false;
	}
}

void waypoint_finish_id_Callback(const std_msgs::Int8 &msg)
{
	waypoint_finish_id = msg.data;
	
	if(waypoint_finish_id != waypoint_finish_id_old)
	{
		flag_new_finish_id     = true;
		waypoint_finish_id_old = waypoint_finish_id;
		printf("new finish id is received!\n");
	}
	else
	{
		flag_new_finish_id = false;
	}
}

bool read_waypoint_file(std::string File_Name)
{
	bool success = false;
    int result   = -10;
   
	FILE *fp;
    
    fp = fopen(File_Name.c_str(), "r");
    no_waypoints = -1;

    if (fp == NULL)
    {
        ROS_ERROR("Waypoint File does not exist!\n");
        success = false;
    }
    
    else
    {
		printf("File Open Success\n");

		do
		{
			++no_waypoints;
			double x, y, theta;
			result = fscanf(fp, "%lf %lf %lf", &x , &y, &theta);
			waypoints[no_waypoints].waypoint_target_pose2d.x     = x;
			waypoints[no_waypoints].waypoint_target_pose2d.y     = y;
			waypoints[no_waypoints].waypoint_target_pose2d.theta = theta;
			
		}while(result != EOF);

		printf("\n");
      
		waypoints[no_waypoints].waypoint_start_pose2d.x     = 0;
		waypoints[no_waypoints].waypoint_start_pose2d.y     = 0;
		waypoints[no_waypoints].waypoint_start_pose2d.theta = 0;
      
		for(int i = 0; i<= no_waypoints + 1; i++)
		{
			waypoints[i + 1].waypoint_start_pose2d.x     = waypoints[i].waypoint_target_pose2d.x;
			waypoints[i + 1].waypoint_start_pose2d.y     = waypoints[i].waypoint_target_pose2d.y;
			waypoints[i + 1].waypoint_start_pose2d.theta = waypoints[i].waypoint_target_pose2d.theta;
		}
		
		waypoints[0].waypoint_start_pose2d.x     = 0;
		waypoints[0].waypoint_start_pose2d.y     = 0;
		waypoints[0].waypoint_start_pose2d.theta = 0;
		
		/*
		for(int i = 0; i <= no_waypoints; i++)	
		{
			printf("Waypoint_Number : %d\n", i);
			printf("Start  : [%.2lf %.2lf %.2lf]\n", waypoints[i].waypoint_start_pose2d.x, waypoints[i].waypoint_start_pose2d.y, waypoints[i].waypoint_start_pose2d.theta);
			printf("Target : [%.2lf %.2lf %.2lf]\n\n", waypoints[i].waypoint_target_pose2d.x, waypoints[i].waypoint_target_pose2d.y, waypoints[i].waypoint_target_pose2d.theta);
		}
		* */
    }
   
    fclose(fp);
    return success;
}

int main(int argc, char **argv)
{
    int count               = 0;
    int start_waypoint_id   = 0;
    int finish_waypoint_id  = 0;
	int current_waypoint_id = 0;
	
	ros::init(argc, argv, "waypoint_control_system");
	ros::NodeHandle nh;

	std::string poseupdate_topic 	         = "/poseupdate";
    std::string type_cmd_vel_topic 	         = "/type_cmd_vel";
    std::string waypoint_file_path         	 = "/home/amap/sim_catkin_ws/src/waypoint_control_system/data/waypoint1.txt";
    
	std::string target_waypoint_line_topic   = "/wp/target_waypoint_line";
	std::string status_waypoint_move_topic   = "/status/waypoint_move";
	std::string waypoint_start_id_topic      = "/start_waypoint_id";    //int8
	std::string waypoint_finish_id_topic     = "/finish_waypoint_id";   //int8
	
	std::string waypoint_run_command_topic   = "/flag/robot_run";       //bool
	
    ros::param::get("~poseupdate_topic",   poseupdate_topic);
    ros::param::get("~type_cmd_vel_topic", type_cmd_vel_topic);
    ros::param::get("~waypoint_file_path", waypoint_file_path);
    ros::param::get("~status_waypoint_move_topic",  status_waypoint_move_topic);
	ros::param::get("~target_waypoint_line_topic", target_waypoint_line_topic);
	
    ros::Subscriber sub_poseupdate 	          = nh.subscribe(poseupdate_topic, 1, poseupdate_Callback);
    ros::Subscriber sub_status_waypoint_move  = nh.subscribe(status_waypoint_move_topic, 1, status_waypoint_move_Callback);
    ros::Subscriber sub_waypoint_start_id     = nh.subscribe(waypoint_start_id_topic, 1, waypoint_start_id_Callback);
    ros::Subscriber sub_waypoint_finish_id    = nh.subscribe(waypoint_finish_id_topic, 1, waypoint_finish_id_Callback);
    
	ros::Publisher pub_type_cmd_vel	          = nh.advertise<std_msgs::Int8>(type_cmd_vel_topic, 1);  
	
	ros::Publisher pub_custom_target_waypoint = nh.advertise<move_waypoint::Target_waypoint_line>(target_waypoint_line_topic, 1);
	
	read_waypoint_file(waypoint_file_path);
	
	/*
    if (read_waypoint_file(waypoint_file_path)) 
    {
        printf("Waypoint file read successfully.");
    } 
    else 
    {
        printf("Failed to read waypoint file.");
    }
	*/
	
    ros::Rate loop_rate(30.0);
	
	while (ros::ok())
	{	
		// publush first waypoint
		// parameter define
		//input waypoint 0 to defind parameter
		
		move_waypoint::Target_waypoint_line waypoint_line;
		
		waypoint_line = waypoints[current_waypoint_id];
		
		pub_custom_target_waypoint.publish(waypoint_line);
		
		printf("current_waypoint_id : %d\n", current_waypoint_id);
		
		for(int i = 0; i <= no_waypoints; i++)	
		{
			printf("Waypoint_Number : %d\n", i);
			printf("Start  : [%.2lf %.2lf %.2lf]\n", waypoints[i].waypoint_start_pose2d.x, waypoints[i].waypoint_start_pose2d.y, waypoints[i].waypoint_start_pose2d.theta);
			printf("Target : [%.2lf %.2lf %.2lf]\n\n", waypoints[i].waypoint_target_pose2d.x, waypoints[i].waypoint_target_pose2d.y, waypoints[i].waypoint_target_pose2d.theta);
		}
		
		
		if(flag_new_waypoint == true)
		{
			current_waypoint_id = waypoint_start_id;
			flag_new_waypoint = false;
		}
		
		if(status_waypoint_move == RUN)
		{
			printf("==============================\n");
			printf("        Run at waypoint       \n");
			printf("==============================\n\n");
		}
		else if(status_waypoint_move == ARRIVAL)
		{
			current_waypoint_id++;
			
			printf("==============================\n");
			printf("      Arrival at waypoint     \n");
			printf("==============================\n\n");
			ros::Duration(1.0).sleep(); // 1 second delay
		}
		else if(status_waypoint_move == STOP)
		{
			printf("==============================\n");
			printf("        Stop at waypoint      \n");
			printf("==============================\n\n");
		}
		else
		{
			printf("==============================\n");
			printf("       Reset at waypoint      \n");
			printf("==============================\n\n");
		}
		
		if(current_waypoint_id >= no_waypoints)
		{
			current_waypoint_id = no_waypoints;
			
			printf("==============================\n");
			printf("       Finish at waypoint     \n");
			printf("==============================\n\n");
		}
		
		std_msgs::Int8 type_cmd_vel;
		pub_type_cmd_vel.publish(type_cmd_vel);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
