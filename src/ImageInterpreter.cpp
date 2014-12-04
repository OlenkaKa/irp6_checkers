#include "ros/ros.h"
#include "irp6_checkers/ImageData.h"
#include "irp6_checkers/Chessboard.h"
#include "Checkers.hpp"
#include <iostream>

using namespace std;

class ImageInterpreter
{
public:
	ImageInterpreter();
	void callback(const irp6_checkers::ImageData& msg);
private:
	ros::NodeHandle _nh;
	ros::Subscriber _image_data;
	ros::Publisher _chessboard;
}; 

ImageInterpreter::ImageInterpreter()
{
	_chessboard = _nh.advertise<irp6_checkers::Chessboard>("chessboard", 1000);
	_image_data = _nh.subscribe("image_data", 1000, &ImageInterpreter::callback, this);
	ROS_INFO("INIT2");
}

//void ImageInterpreter::callback(const irp6_checkers::ImageData::ConstPtr& msg)
void ImageInterpreter::callback(const irp6_checkers::ImageData& msg)
{
	ROS_INFO("[ImageInterpreter] ------> New data received.");
	// TODO
	ROS_INFO("[ImageInterpreter] Received circles:");
	int size = msg.Circles.size();
	for(int i=0; i<size; ++i)
	{
		cout<<"Color:\t"<<msg.Circles[i].color<<"; Point:\t["<<msg.Circles[i].x<<", "<<msg.Circles[i].y<<"] \n";
	}
	ROS_INFO("[ImageInterpreter] Received fields:");
	size = msg.WhiteFields.size();
	for(int i=0; i<size; ++i)
	{
		cout<<"Point:\t["<<msg.WhiteFields[i].x<<", "<<msg.WhiteFields[i].y<<"] \n";
	}
	cout<<"MAX: "<<msg.MaxChessboardField.x<<", "<<msg.MaxChessboardField.y<<endl;
	cout<<"MIN: "<<msg.MinChessboardField.x<<", "<<msg.MinChessboardField.y<<endl;
	
	
	ROS_INFO("[ImageInterpreter] Send chessboard.");
	irp6_checkers::Chessboard chessboard;
	irp6_checkers::Checker data;
	data.type=1;
	data.x=2;
	data.y=3;
	chessboard.Chessboard.push_back(data);
	data.type=2;
	data.x=5;
	data.y=6;
	chessboard.Chessboard.push_back(data);
	_chessboard.publish(chessboard);
	ros::spinOnce();
	
	ROS_INFO("[ImageInterpreter] <------ End of data.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_interpreter");
	ros::NodeHandle _nh;
	Checkers::Chessboard b;
	ImageInterpreter interpreter;
	ros::spin();
	return 0;
  /*
  ros::init(argc, argv, "image_interpreter");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("image_data", 1000, callback);
  ros::spin();
  return 0;
  */
}
