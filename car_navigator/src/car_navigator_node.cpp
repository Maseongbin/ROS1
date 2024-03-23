#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"


// maximum angular velocity 
#define MAX_L_STEER -6.0
#define MAX_R_STEER  6.0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoints_NO 6
#define WayPoint_X_Tor 0.1
#define WayPoint_Y_Tor 0.3

#define SpeedRegion_NO 100


double pos_x = 0.0;
double pos_y = 0.0;
float vision_steering_angular_velocity = 0.0;
float car_linear_x = 0.0;
double roll,pitch,yaw;

float rqt_robot_steering_cmd_angular = 0.0;
float rqt_robot_steering_cmd_angular_old = 0.0;

float rqt_robot_steering_cmd_linear = 0.0;
float rqt_robot_steering_cmd_linear_old = 0.0;

double r,p,y;

float car_speed_base = 0.3; //[m/s]
float car_speed[SpeedRegion_NO] = {car_speed_base,};


int no_region = 4;
int manual_speed_control_flag = 1;  // 1 - manual , 0 - auto
int vision_steering_control_flag = 0;
int hector_odom_flag = 0;
int roi_no = -1;

struct Point 
{ 
	float x; 
	float y; 
	float z;
};


struct WayPoints
{
	float x;
	float y;
	
} ;

struct Current_Pos
{
	float x = 0.0;
	float y = 0.0;
	float yaw_angle = 0.0 ;
} my_pose, my_pose_hector;

struct region
{
	float left;
	float right;
	float top;
	float bottom;
} speed_region[SpeedRegion_NO];

void odomCallback(const nav_msgs::Odometry& msg)
{
	my_pose.x = msg.pose.pose.position.x;
	my_pose.y = msg.pose.pose.position.y;
		
	tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      
   
    m.getRPY(r, p, y);
    my_pose.yaw_angle = y;
	
	ROS_INFO("%.2f %.2f %.2f", msg.pose.pose.position.x, msg.pose.pose.position.y, my_pose.yaw_angle); 
	
}

void hectorslamposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	my_pose_hector.x = msg.pose.pose.position.x; 	 
	my_pose_hector.y = msg.pose.pose.position.y;
	
	tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);      
   
    m.getRPY(r, p, y);
    my_pose_hector.yaw_angle = y;
}


void rqt_robot_cmd_velCallback(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  rqt_robot_steering_cmd_angular = msg.angular.z;
  rqt_robot_steering_cmd_linear  = msg.linear.x ;
             
  if( (rqt_robot_steering_cmd_angular!= rqt_robot_steering_cmd_angular_old) ||  (rqt_robot_steering_cmd_linear!= rqt_robot_steering_cmd_linear_old))
  {
	  //printf("rqt_steering\n");
	  vision_steering_angular_velocity = rqt_robot_steering_cmd_angular;
	  car_linear_x = rqt_robot_steering_cmd_linear;
	  if(vision_steering_angular_velocity  >= MAX_R_STEER)  vision_steering_angular_velocity  = MAX_R_STEER;
      if(vision_steering_angular_velocity  <= MAX_L_STEER)  vision_steering_angular_velocity  = MAX_L_STEER;  
  }
  
  rqt_robot_steering_cmd_angular_old = rqt_robot_steering_cmd_angular;
  rqt_robot_steering_cmd_linear_old  = rqt_robot_steering_cmd_linear ;
   
}



void VisionSteerControlCallback(const std_msgs::Float32& angle)
{
  vision_steering_angular_velocity  = angle.data ;
  
  if(vision_steering_angular_velocity  >= MAX_R_STEER)  vision_steering_angular_velocity  = MAX_R_STEER;
  if(vision_steering_angular_velocity  <= MAX_L_STEER)  vision_steering_angular_velocity  = MAX_L_STEER;  
}


void read_speed_data(void)
{
	FILE *fp;
	int result = 0;
	int i = 0;
		
	fp = fopen("//home//amap//race_data//vision_data/speed_data.txt","r");
	if(fp == NULL) 
	{
		ROS_INFO("Speed Data File does not exit ~~ ");
		ROS_INFO("Speed is set to 0.5[m/sec] ~~ ");
		
		car_speed_base = 0.0;
	}

	result = fscanf(fp,"%f", &car_speed_base);
	result = fscanf(fp,"%d", &no_region);
	
	for(i=0; i< no_region ; i++)
	{
		result = fscanf(fp,"%f", &speed_region[i].left);
		result = fscanf(fp,"%f", &speed_region[i].top);
		result = fscanf(fp,"%f", &speed_region[i].right);
		result = fscanf(fp,"%f", &speed_region[i].bottom);
		result = fscanf(fp,"%f", &car_speed[i]);
	}
	
	//result = fscanf(fp,"%f", &m_Kd);
	//result = fscanf(fp,"%f", &m_Ki);
	
	//printf("PID: %.4f  %.4f   %.4f \n",m_Kp,m_Kd,m_Ki);
	
	fclose(fp);
	
}

float speed_check(void)
{
	
	float min_y, max_y;
	float min_x, max_x;
	float speed;
	int i;
    roi_no = -1; 
    
    min_y = -3.7;
    max_y = -3.4;
    min_x =  0.5;
    max_x =  0.6;
        
    if(hector_odom_flag==1)
	{
	  my_pose.x = my_pose_hector.x;
	  my_pose.y = my_pose_hector.y;
    }  
      
    if( (my_pose.x >= min_x) && (my_pose.x <= max_x)  && (my_pose.y >= min_y) && (my_pose.y  <= max_y ) )
	{
       speed = 0;
       return speed;
    }	
    else 
    {		
      
	 for(i=0; i< no_region ; i++)
	 {
		
		min_x = (speed_region[i].bottom >= speed_region[i].top) ? speed_region[i].top : speed_region[i].bottom;
        max_x = (speed_region[i].bottom >= speed_region[i].top) ? speed_region[i].bottom : speed_region[i].top;
        min_y = (speed_region[i].left >= speed_region[i].right) ? speed_region[i].right : speed_region[i].left;
        max_y = (speed_region[i].left >= speed_region[i].right) ? speed_region[i].left : speed_region[i].right;
        
		if( (my_pose.x >= min_x) && (my_pose.x <= max_x)  && (my_pose.y >= min_y) && (my_pose.y  <= max_y ) )
		{
		  roi_no = i;	
		  speed = car_speed[i];
		  return speed;
		}		 
		else 
		{
			speed = car_speed_base;		
		}
	 }
	 
   }	
	 return speed;
	
}


int main(int argc, char **argv)
{
  int count = 0;
  read_speed_data();
  
  ros::init(argc, argv, "car_navigation");

  ros::NodeHandle n;
  
  
  ros::param::get("/vision_control_flag",  vision_steering_control_flag);
  ros::param::get("/manual_speed_control_flag",  manual_speed_control_flag);
  ros::param::get("/hector_odom_flag",  hector_odom_flag);
  
  std::string odom_sub_topic = "/ackermann_steering_controller/odom";
  
   
  ros::Subscriber sub1 = n.subscribe("/Car_Control_cmd/Vison_SteerAngle_Float16",100, &VisionSteerControlCallback);  
  ros::Subscriber sub2 = n.subscribe(odom_sub_topic, 10, &odomCallback);
  ros::Subscriber sub3 = n.subscribe("/rqt_robot_steering/cmd_vel",100,&rqt_robot_cmd_velCallback);
  ros::Subscriber sub4 = n.subscribe("/poseupdate",30,&hectorslamposeCallback);
   
  ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);
  //ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("marker/node", 1);
  ros::Publisher target_pos_pub = n.advertise<geometry_msgs::Pose2D>("pos_goal", 1);
  ros::Publisher manual_speed_control_flag_pub = n.advertise<std_msgs::Int8>("manual_speed_flag",1); 
  ros::Publisher roi_no_pub = n.advertise<std_msgs::Int8>("speed_roi_no",1);
  //n.setParam("test_mode",1);
  
  ros::Rate loop_rate(30);  // 10
  
  geometry_msgs::Twist msg_cmd;
  std_msgs::Int8 speed_roi_no;
  std_msgs::Int8 speed_manual_flag;
  if(manual_speed_control_flag == 1)  car_linear_x = 0.0;
  
  while (ros::ok())
  {
		 
	if(manual_speed_control_flag == 1)  msg_cmd.linear.x = car_linear_x;
	else                                msg_cmd.linear.x = speed_check();    
	 
	if(vision_steering_control_flag  == 1)
	{ 
	  if(msg_cmd.linear.x>=0)      msg_cmd.angular.z =  vision_steering_angular_velocity ;
	  else                         msg_cmd.angular.z = -vision_steering_angular_velocity ;
    }
    
    pub_cmd_vel.publish(msg_cmd);
    speed_roi_no.data = roi_no;
    roi_no_pub.publish(speed_roi_no);
	speed_manual_flag.data = manual_speed_control_flag;
	manual_speed_control_flag_pub.publish(speed_manual_flag);
	ROS_INFO(" X : %6.3lf  Y : %6.3lf  A : %.5lf ",my_pose.x, my_pose.y,vision_steering_angular_velocity);
	 
	loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }
  return 0;
}
