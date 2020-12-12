#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include "panda_msgs/Yolo.h"
#include "panda_msgs/Depth.h"

using namespace std;
using namespace cv;



Mat opencvImage;
Mat filtered;
panda_msgs::Yolo yolo_result;

void imageCb_1(const sensor_msgs::ImageConstPtr& msg);
Mat image_filter(Mat &input_image, int filter_size,Mat* temp_image);
void find_sector(float dst[3][5],Mat &input_image);
Mat low_pass_filter(Mat &input_image,double tau);
bool operate(panda_msgs::Depth::Request &req,panda_msgs::Depth::Response &res);
double get_distance(int u,int v,Mat &input_image);
void imageCb_2(const panda_msgs::Yolo& msg);

int main(int argc,char** argv){
	ros::init(argc,argv,"POINTCLOUD_NODE");
	ros::NodeHandle nh;
	ros::Subscriber sub1 = nh.subscribe("/camera/depth/image_rect_raw", 1 ,imageCb_1);
	ros::Subscriber sub2 = nh.subscribe("/yolo_result", 1 ,imageCb_2);
	ros::ServiceServer depth_server = nh.advertiseService("depth_server",operate);
	while(filtered.empty())
		ros::spinOnce();
	ROS_INFO("DEPTH SERVER RUNING!!");
	while(ros::ok()){
		ros::spinOnce();
	}
	return 0;
}
bool operate(panda_msgs::Depth::Request &req,panda_msgs::Depth::Response &res){
	ROS_INFO("SEVICE ACCEPTED!!");
	bool is_person = false;
	static struct Person
	{
		double angle = 0;
		double dist = 0;
		bool comming = false;
	} person;
	is_person = yolo_result.result;
	if(is_person){
		cout<<"X : "<<yolo_result.u<<" Y : "<<yolo_result.v<<endl;
		 double now_dist  = get_distance(yolo_result.u-50,yolo_result.v,filtered);
		 if(abs(now_dist - person.dist)>20 && now_dist > person.dist){
		 	person.comming = true;
		 	cout<<"DON'T GO!!!"<<endl;
		 }
		 else if(abs(now_dist - person.dist)>20 && now_dist < person.dist){
		 	person.comming = true;
		 	cout<<"PERSON IS COMMING"<<endl;
		 }
		 else{
		 	person.comming = true;
		 	cout<<"PERSON IS STOP!!"<<endl;
		 }
		 person.dist = (0.1*now_dist + person.dist*0.9);
		 person.angle = (yolo_result.u - 390)/180.0;
		 cout<<"PRESON ANGLE : "<<person.angle<<endl;
	}
	else{
		cout<<"No Person!!"<<endl;
		person.angle = 0;
		person.dist = 0;
		person.comming = false;
	}
	
	float sector[3][5]{};
	find_sector(sector,filtered);
	bool is_stop = false;
	double dist = 0;
	double weight = 0;
	double min_sector[5] {};
	double sig[5] {};
	double left_velo,right_velo;
	left_velo = 30;
	right_velo = 30;
	for(int i = 0;i<5;i++){
		double min = 3000;
		for(int j =0;j<3;j++){
			if(sector[j][i] < min && sector[j][i] != 0)
				min = sector[j][i];
		}
		min_sector[i] = min;
	}
	for(int i=0;i<5;i++){
		sig[i] = -(min_sector[i]-1200)/300/(abs(i-2)*2+1);
		cout<<"SIG : "<<sig[i]<<"\t";
	}
	cout<<endl;
	if(!is_person || !person.comming){
		double right_weight = sig[0]+sig[1]+sig[2];
		double left_weight = sig[2]+sig[3]+sig[4];
		left_weight = (1/(1+pow(2.7,left_weight))-0.5)*2;
		if(left_weight < 0)
			left_weight = 0;
		else if(left_weight<0.1)
			left_weight = 0.1;
		right_weight = (1/(1+pow(2.7,right_weight))-0.5)*2;
		if(right_weight < 0)
			right_weight = 0;
		else if(right_weight<0.1)
			right_weight = 0.1;
		cout<<"LEFT WEIGHT : "<<left_weight<<endl;
		cout<<"RIGHT WEIGHT : "<<right_weight<<endl;
		res.left = left_weight*left_velo;
		res.right = right_weight*right_velo;
		if(res.right == 0 && res.left == 0){
			res.left = -10;
			res.right = 10;
		}
		res.result = false;
	}
	else{
		double right_weight = (sig[1]+sig[2] +sig[3])/2*1.88;
		double left_weight = (sig[1]+sig[2]+sig[3])/2*1.88;
		if(person.angle<0){
			left_velo = -15*(1/(1+pow(2.7,1-abs(person.angle))));
			right_velo = 15*(1/(1+pow(2.7,1-abs(person.angle))));
		}
		else{
			left_velo = 10*(1/(1+pow(2.7,1-abs(person.angle))));
			right_velo = -10*(1/(1+pow(2.7,1-abs(person.angle))));
		}
		cout<<"LEFT VELO : "<<left_velo;
		cout<<" RIGHT VELO : "<<right_velo<<endl;
		left_velo += 20;
		right_velo += 20;
		left_weight = (1/(1+pow(2.7,left_weight))-0.5)*2;
		if(left_weight < 0)
			left_weight = 0;
		else if(left_weight<0.1)
			left_weight = 0.1;
		right_weight = (1/(1+pow(2.7,right_weight))-0.5)*2;
		if(right_weight < 0)
			right_weight = 0;
		else if(right_weight<0.1)
			right_weight = 0.1;
		cout<<"LEFT WEIGHT : "<<left_weight<<endl;
		cout<<"RIGHT WEIGHT : "<<right_weight<<endl;
		res.left = left_weight*left_velo;
		res.right = right_weight*right_velo;
		res.result = true;
	}
	return true;
}

void imageCb_2(const panda_msgs::Yolo& msg){
	yolo_result = msg;
}
void imageCb_1(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	try
     {
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
     }
     catch (cv_bridge::Exception& e)
      {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }
    cv_ptr->image.convertTo(opencvImage,CV_32F);
    static Mat mean_filter[20];
    static bool start = false;
	if(!start){
		for(int i=0;i<20;i++)
			mean_filter[i] = Mat::zeros(Size(640, 480), CV_32F);
		start = true;
	}
	filtered = low_pass_filter(opencvImage,0.98);
	filtered = image_filter(opencvImage,20,mean_filter);
	imshow("depth_img",filtered);
    waitKey(1);
}


Mat image_filter(Mat &input_image, int filter_size,Mat* temp_image){
	Mat output_image = Mat::zeros(Size(640, 480), CV_32F);
	static int index =0;
	
	temp_image[index] = input_image;
	index = (++index)%filter_size;
	for(int i =0;i<filter_size;i++){
		output_image += temp_image[i];
	}
	output_image /= filter_size;
	return output_image;
}
Mat low_pass_filter(Mat &input_image,double tau){
	static Mat pre_img;
	static bool start = false;
	Mat output_image;
	if(!start)
		pre_img = input_image.clone();
	output_image = (pre_img*tau + input_image*(1-tau));
	pre_img = output_image;
	return output_image;
}
void find_sector(float dst[3][5],Mat &input_image){
	float* data = (float*)input_image.data;
	int hight = input_image.rows;
	int width = input_image.cols;
	int sector_count[3][5] {};
	for(int i =0;i<hight;i++){
		for(int j=0;j<width;j++){
			int sector_x = j/128;
			int sector_y = i/160;
			float dist = data[i*width + j];
			if(dist>50&&dist<3000){
				dst[sector_y][sector_x] = (dst[sector_y][sector_x]*sector_count[sector_y][sector_x] + dist)/(sector_count[sector_y][sector_x]+1);
				sector_count[sector_y][sector_x]++;
			}
		}
	}
	for (int i = 0;i<3;i++){
		for (int j=0; j<5;j++){
			cout<<"["<<i<<","<<j<<"] "<<dst[i][j]<<"\t";
		}
		cout<<endl;
	}
	cout<<endl<<endl;
}

double get_distance(int u,int v,Mat &input_image){
	float* data = (float*)input_image.data;
	int hight = input_image.rows;
	int width = input_image.cols;
	int cut_u[2],cut_v[2];
	double distance = 0;
	int count = 0;
	if(u<50){
		cut_u[0] = 0;
		cut_u[1] = 100;
	}
	else if(u>590){
		cut_u[0] = 540;
		cut_u[1] = 640;
	}
	else{
		cut_u[0] = u-50;
		cut_u[1] = u+50;
	}
	if(v<50){
		cut_v[0] = 0;
		cut_v[1] = 100;
	}
	else if(v>430){
		cut_v[0] = 380;
		cut_v[1] = 480;
	}
	else{
		cut_v[0] = v-50;
		cut_v[1] = v+50;	
	}
	for(int i =cut_v[0];i<cut_v[1];i++){
		for(int j=cut_u[0];j<cut_u[1];j++){
			float dist = data[i*width + j];
			if(dist>3000)
				dist = 3000;
			if(dist>50){
				distance = (distance *count + dist)/(count +1);
				count++;
			}
		}
	}
	cout<<"count : "<<count<<endl;
	cout<<"DIST : "<<distance<<endl;
	return distance;
}