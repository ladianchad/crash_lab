#include "ros/ros.h"
#include "panda_msgs/Depth.h"
#include <iostream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "panda_msgs/tilt.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "panda_msgs/Yolo.h"
#include <stdlib.h>
#include <time.h>
#include <string>
using namespace std;


void normal_todo();
void end_fun(const std_msgs::Empty& msg);
void start_todo();
void ui_interaction();
void start_fun(const std_msgs::Empty& msg);
void motor_pub(double l_sp,double r_sp,double duration);
void stop(double l_pre,double r_pre,double time);
void uiCb(const std_msgs::String &msg);
void yoloCb(const panda_msgs::Yolo& msg);
ros::ServiceClient *client;
ros::Publisher *speed_pub ;
ros::Publisher *tilt_pub;
ros::Publisher *camera_pub;
ros::Publisher *ui_pub1,*ui_pub2;
ros::Publisher *yolo_pub;
int robot_state = 1;
panda_msgs::tilt tilt_msg;
std_msgs::UInt16 camera_servo_msgs;
std_msgs::String ui_msg,from_ui;
std_msgs::Int32 yolo_mode;
panda_msgs::Yolo yolo_result;
bool is_ui_msg = false;

int main(int argc,char** argv){
	ros::init(argc,argv,"ROBOT_CORE");
	ros::NodeHandle nh;
	ros::ServiceClient Depth_cl = nh.serviceClient<panda_msgs::Depth>("depth_server");
	client = &Depth_cl;
	ros::Publisher sp_pub = nh.advertise<std_msgs::Float32MultiArray>("/speed_set", 1);
	ros::Publisher t_pub = nh.advertise<panda_msgs::tilt>("/tilting/tilt", 1);
	ros::Publisher s_pub = nh.advertise<std_msgs::UInt16>("/servo", 1);
	ros::Publisher u_pub1 = nh.advertise<std_msgs::String>("/panda/ui/contrl", 1);
	ros::Publisher u_pub2 = nh.advertise<std_msgs::String>("TerminalToPyqt", 1);
	ros::Publisher y_pub = nh.advertise<std_msgs::Int32>("/yolo/mode", 1);
	speed_pub = &sp_pub;
	tilt_pub = &t_pub;
	camera_pub = &s_pub;
	ui_pub1 = &u_pub1;
	ui_pub2 = &u_pub2;
	yolo_pub = &y_pub;
	ros::Subscriber sub1 = nh.subscribe("/end_core", 1 ,end_fun);
	ros::Subscriber sub2 = nh.subscribe("/start_core", 1 ,start_fun);
	ros::Subscriber sub3 = nh.subscribe("TerminalToPyqt1", 1 ,uiCb);
	ros::Subscriber sub4 = nh.subscribe("/yolo_result", 1 ,yoloCb);
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
				ui_interaction();
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
void ui_interaction(){
	//2to 0
	//lastvideo to
	static bool is_plastic = false;
	static bool find = false;
	if(is_ui_msg){
		if(from_ui.data=="2"){
			ui_msg.data = "0";
			ui_pub2->publish(ui_msg);
		}
		else if(from_ui.data=="nocup"){
			ui_msg.data = "5";
			ui_pub2->publish(ui_msg);	
		}
		else if(from_ui.data=="4"){
			ui_msg.data = "6";
			ui_pub2->publish(ui_msg);	
		}
		else if(from_ui.data=="donation"){
			ui_msg.data = "7";
			ui_pub2->publish(ui_msg);	
		}
		else if(from_ui.data=="lastvideo"){
			ui_msg.data = "lastvideo1";
			ui_pub2->publish(ui_msg);	
		}
		else if(from_ui.data=="lastvideo2"){
			ui_msg.data = "7";
			ui_pub2->publish(ui_msg);	
		}
		else if(from_ui.data=="putcup"){
			ui_msg.data = "putcup1";
			ui_pub2->publish(ui_msg);	
		}
		else if(from_ui.data=="endvideo"){
			if(is_plastic)
				ui_msg.data = "plastic";
			else
				ui_msg.data = "paper";
			ui_pub2->publish(ui_msg);	
		}
		else if(from_ui.data=="displaycam"){
			yolo_mode.data = 1;
			yolo_pub->publish(yolo_mode);
			ros::Duration(1).sleep();
			find = true;
		}else if(from_ui.data=="end"){
			ui_msg.data = "wait";
			ui_pub2->publish(ui_msg);
			robot_state = 2;	
		}
		cout<<ui_msg.data<<endl;
		is_ui_msg = false;
	}
	if(find){
		if(yolo_result.result==2){
			ui_msg.data = "detect1";
			ui_pub2->publish(ui_msg);
			is_plastic = true;
			find = false;
		}else if(yolo_result.result==3){
			ui_msg.data = "detect1";
			ui_pub2->publish(ui_msg);
			is_plastic = false;
			find = false;
		}else{
			find = true;
		}
		cout<<ui_msg.data<<endl;
	}
	ros::spinOnce();
}

void normal_todo(){
	panda_msgs::Depth srv;
	yolo_mode.data = 0;
	yolo_pub->publish(yolo_mode);
	static time_t looking_time = time(NULL);
	static time_t go_time = time(NULL);
	time_t now_time = time(NULL);
	if(client->call(srv)){
		cout<<"LEFT SPEED : "<<srv.response.left<<" RIGHT SPEED : "<<srv.response.right<<endl;
		if(srv.response.left==0 && srv.response.right==0){
			tilt_msg.ID = 1;
			tilt_msg.Position = 120;
			tilt_pub->publish(tilt_msg);
			camera_servo_msgs.data = 45;
			camera_pub->publish(camera_servo_msgs);
			motor_pub(srv.response.left,srv.response.right,7);
			ui_msg.data = "3";
			ui_pub2->publish(ui_msg);
			from_ui.data = "hi1";
			ui_pub2->publish(from_ui);
			cout<<"UI interaction start"<<endl;
			robot_state = 3;
		}
		else if(srv.response.left == -10 && srv.response.right == 10){
			stop(0,0,1);
			motor_pub(srv.response.left,srv.response.right,1.5);
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
					stop(srv.response.left,srv.response.right,1);
					motor_pub(count*20,-count*20,0.5);
					stop(count*20,-count*20,1);
					motor_pub(-count*20,count*20,0.5);
					stop(-count*20,count*20,1);
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
						stop(srv.response.left,srv.response.right,1);
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
	static bool start = false;
	motor_pub(0,0,0.1);
	tilt_msg.ID = 1;
	tilt_msg.Position = 660;
	tilt_pub->publish(tilt_msg);
	camera_servo_msgs.data = 10;
	if(!start){
		ui_msg.data = "wait";
		ui_pub2->publish(ui_msg);
		start = true;
	}
	camera_pub->publish(camera_servo_msgs);
}

void motor_pub(double l_sp,double r_sp,double duration){
	std_msgs::Float32MultiArray speed_msg;
	speed_msg.data.clear();
	speed_msg.data.push_back(l_sp);
	speed_msg.data.push_back(r_sp);
	speed_pub->publish(speed_msg);
	ros::Duration(duration).sleep();
}

void stop(double l_pre,double r_pre,double time){
	double l_step = l_pre/10;
	double r_step = r_pre/10;
	double time_step = time/10;
	for(int i=0;i<10;i++){
		motor_pub(l_pre - l_step*(i+1),r_pre - r_step*(i+1),time_step);
	}
}

void uiCb(const std_msgs::String &msg){
	cout<<"Message Data : "<<msg.data<<endl;
	from_ui = msg;
	is_ui_msg = true;
}

void yoloCb(const panda_msgs::Yolo& msg){
	yolo_result = msg;
}