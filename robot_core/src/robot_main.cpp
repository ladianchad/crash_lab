#include "ros/ros.h"
#include "panda_msgs/Depth.h"
#include <iostream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include <stdlib.h>
#include <time.h>
using namespace std;


void normal_todo();
void end_fun(const std_msgs::Empty& msg);
void start_todo();
void start_fun(const std_msgs::Empty& msg);
void motor_pub(double l_sp,double r_sp,double duration);
ros::ServiceClient *client;
ros::Publisher *speed_pub ;
int robot_state = 1;

int main(int argc,char** argv){
	ros::init(argc,argv,"ROBOT_CORE");
	ros::NodeHandle nh;
	ros::ServiceClient Depth_cl = nh.serviceClient<panda_msgs::Depth>("depth_server");
	client = &Depth_cl;
	ros::Publisher sp_pub = nh.advertise<std_msgs::Float32MultiArray>("/speed_set", 1);
	speed_pub = &sp_pub;
	ros::Subscriber sub1 = nh.subscribe("/end_core", 1 ,end_fun);
	ros::Subscriber sub2 = nh.subscribe("/start_core", 1 ,start_fun);
	cout<<"ROBOT READY!!!"<<endl;
	while(ros::ok()){
		switch(robot_state) {
			case 1 :
				start_todo();
			break;
			case 2:
				normal_todo();
			break;
			case 3 :
			break;
			default :
			break;
		}
		ros::spinOnce();
	}
	return 0;
}
void start_fun(const std_msgs::Empty& msg){
	cout<<"MOTORT SERVICE START!!"<<endl;
	motor_pub(0,0,0.1);
	robot_state = 2;
};
void end_fun(const std_msgs::Empty& msg){
	cout<<"ROBOT SERVICE END!!"<<endl;
	motor_pub(0,0,0.1);
};
void normal_todo(){
	panda_msgs::Depth srv;
	static time_t looking_time = time(NULL);
	static time_t go_time = time(NULL);
	time_t now_time = time(NULL);
	if(client->call(srv)){
		cout<<"LEFT SPEED : "<<srv.response.left<<" RIGHT SPEED : "<<srv.response.right<<endl;
		if(srv.response.left==0 && srv.response.right==0){
			motor_pub(srv.response.left,srv.response.right,7);
			motor_pub(-20,20,2);
			go_time = time(NULL);
		}
		else if(srv.response.left == -20 && srv.response.right == 20){
			motor_pub(srv.response.left,srv.response.right,1.5);
			motor_pub(0,0,0.5);
			go_time = time(NULL);
		}
		else{
			static bool look_up = false;
			if(srv.response.result){
				motor_pub(srv.response.left,srv.response.right,0.1);
				look_up = false;
				go_time = time(NULL);
			}
			else{
				int random_look = rand();
				cout<<"random_look : "<<random_look<<endl;
				srand(random_look);
				if((random_look%10 == 0 || look_up) && now_time - looking_time > 10){
					cout<<"ROBOT LOOK UP!!"<<endl;
					static int count = 1;
					motor_pub(0,0,0.5);
					motor_pub(count*20,-count*20,0.5);
					motor_pub(0,0,0.5);
					motor_pub(-count*20,count*20,0.5);
					motor_pub(0,0,0.5);
					count = count * -1;
					if(!look_up)
						look_up = true;
					else{
						look_up = false;
						looking_time = time(NULL);
					}
				}
				else{
					if(now_time -  go_time > 15){
						cout<<"ROBOT TURN"<<endl;
						go_time = time(NULL);
						motor_pub(-15,15,1.5);
					}
					else
						motor_pub(srv.response.left,srv.response.right,0.1);
				}
			}
		}
	}
}

void start_todo(){
	motor_pub(0,0,0.1);
}

void motor_pub(double l_sp,double r_sp,double duration){
	std_msgs::Float32MultiArray speed_msg;
	speed_msg.data.clear();
	speed_msg.data.push_back(l_sp);
	speed_msg.data.push_back(r_sp);
	speed_pub->publish(speed_msg);
	ros::Duration(duration).sleep();
}
