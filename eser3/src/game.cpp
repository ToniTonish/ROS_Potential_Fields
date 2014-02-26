#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <fstream>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <math.h>

#define angle_increment 0.00436736317351
#define SOGLIA 0.2
#define ANGULAR_V 0.5
#define LINEAR_V 0.5

 double goal_x;
 double goal_y;

double robot_pose_x = 0.0;
double robot_pose_y = 0.0;
double robot_pose_orientation = 0.0;

const double k_obstacle = 2.0;
const double k_goal = 25.0;

double f_tot;

bool check = true;
bool linear_movement = true;
bool angular_movement_left = true;
bool angular_movement_right = true;

void subPose(const nav_msgs::Odometry::ConstPtr& pos)
{
	// setto le posizioni iniziali del robot 
	robot_pose_x = pos->pose.pose.position.x;
	robot_pose_y = pos->pose.pose.position.y;
	robot_pose_orientation = tf::getYaw(pos->pose.pose.orientation);	

	printf("posx posy posw: [%f] [%f] [%f]\n", robot_pose_x, robot_pose_y, robot_pose_orientation*180/M_PI);

	check = false;
}

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	double f_repx, f_repy;
	double f_attrx, f_attry;
	double angle_from_goal, distance_from_goal;
	double angle_tot;

	// questo mi serve per rilevare gli ostacoli e calcolare la forza repulsiva data da essi
	// e fare la somma per calcolarmi le componenti repulsive totali.
	for(int i=0;i<1080;i++) {
			f_repx += cos((i*angle_increment)-(3*M_PI/4))/(k_obstacle*(pow(scan->ranges[i], 2))); 
			f_repy += sin((i*angle_increment)-(3*M_PI/4))/(k_obstacle*(pow(scan->ranges[i], 2))); 
	}

	//calcolo la distanza del robot dal goal
	distance_from_goal = sqrt(pow(goal_x-robot_pose_x, 2)+(pow(goal_y-robot_pose_y, 2)));

	//calcolo l'angolo tra la posizione frontale del robot e il goal
	angle_from_goal = atan2(goal_y-robot_pose_y, goal_x-robot_pose_x) - robot_pose_orientation;
	
	if (angle_from_goal < -M_PI)
		angle_from_goal += 2*M_PI;
	else if (angle_from_goal > M_PI)
		angle_from_goal -= 2*M_PI;
		
	printf("-----------------------------\n");
	printf("distance_from_goal: %f angle_from_goal: %f\n", distance_from_goal, angle_from_goal*180/M_PI);
	printf("-----------------------------\n");	

	f_attrx = cos(angle_from_goal)*k_goal*distance_from_goal;
	f_attry = sin(angle_from_goal)*k_goal*distance_from_goal;
	
	//printf("FORZA ATTRATTIVA X: %f\n", f_attrx);

	angle_tot = atan2(f_attry-f_repy, f_attrx-f_repx);
	if (angle_tot < -M_PI)
		angle_tot += 2*M_PI;
	else if (angle_tot > M_PI)
		angle_tot -= 2*M_PI;
		
	f_tot = sqrt(pow(f_attrx-f_repx, 2)+pow(f_attry-f_repy, 2));
	angle_tot = atan2(f_attry-f_repy, f_attrx-f_repx);

	//printf("forza totale: %f angolo totale: %f\n", f_tot, angle_tot*180/M_PI);

	//printf("-----------------------------\n");
	//printf("angolo totale: %f angolo robot: %f\n", angle_tot*180/M_PI, robot_pose_orientation*180/M_PI);
	//printf("differenza: %f\n", angle_tot);
	//printf("-----------------------------\n");
	if (fabs(angle_tot) > SOGLIA) {
		linear_movement = false;
		if (angle_tot > 0) {
			angular_movement_left = true;
			angular_movement_right = false;
		} else {
			angular_movement_right = true;
			angular_movement_left = false;
		}
	} else {
		linear_movement = true;
		angular_movement_left = false;
		angular_movement_right = false;
	}

} 

int main(int argc, char** argv) {
	ros::init(argc, argv, "game");

	ros::NodeHandle n; 
	ros::Subscriber read_position;
 	read_position = n.subscribe("/base_pose_ground_truth", 10, subPose);
 	
 	goal_x = atof(argv[1]);
 	goal_y = atof(argv[2]);
 	
	while(check)
		ros::spinOnce();

	read_position.~Subscriber();

	ros::Publisher twist_pub;
	ros::Subscriber sensor_distance;
	geometry_msgs::Twist msg;

	twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  	sensor_distance = n.subscribe("/base_scan", 10, laserCallBack);

	ros::Rate loop_rate(10);
	
	double t1, t2;
	double space=0;

	while(ros::ok()) {
	
		if (fabs(robot_pose_x-goal_x) < 0.5 && fabs(robot_pose_y-goal_y) < 0.5) {
			printf("GOAL RAGGIUNTO!!\n");
			ros::shutdown();
		}

		if (robot_pose_orientation > M_PI)
			robot_pose_orientation -= 2*M_PI;

		printf("-----------------------------\n");
		printf("posizioni robot:\n");
		printf("robot_pose_x: %f\n", robot_pose_x);
		printf("robot_pose_y: %f\n", robot_pose_y);
		printf("robot_pose_orientation: %f\n", robot_pose_orientation*180/M_PI);
		printf("-----------------------------\n");

		if (f_tot < 100) {
			
			msg.linear.x = LINEAR_V;
			msg.angular.z = 0.0;
			
			robot_pose_x += LINEAR_V*space*cos(robot_pose_orientation);
			robot_pose_y += LINEAR_V*space*sin(robot_pose_orientation);
		} else {
			if (linear_movement) {
				
				msg.linear.x = LINEAR_V;
				msg.angular.z = 0.0;
			
				robot_pose_x += LINEAR_V*space*cos(robot_pose_orientation);
				robot_pose_y += LINEAR_V*space*sin(robot_pose_orientation);
			}
	
			if (angular_movement_left) {
				msg.linear.x = 0.0;
				msg.angular.z = ANGULAR_V;
				// printf("******GIRA left*******\n");
				robot_pose_orientation += ANGULAR_V*space;
			}
	
			if (angular_movement_right) {
				msg.linear.x = 0.0;
				msg.angular.z = -ANGULAR_V;
				// printf("******GIRA right*******\n");
				robot_pose_orientation -= ANGULAR_V*space;
			}
		}
		
		twist_pub.publish(msg);
		
		t1 = ros::Time::now().toSec();
		
		loop_rate.sleep();
		
		t2 = ros::Time::now().toSec();
		
		space = t2 - t1;

		// printf("spazio percorso: %f\n", space);
		ros::spinOnce();
	}
  	return(0);
}
