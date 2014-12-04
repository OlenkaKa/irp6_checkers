#include "ros/ros.h"
#include "irp6_checkers/ImageData.h"
#include "irp6_checkers/Chessboard.h"
//#include "irp6_checkers/test.h"
//#include "irp6_checkers/Checkers.hpp"
#include <iostream>

using namespace std;

class MoveDetermination
{
public:
	MoveDetermination();
	void callback(const irp6_checkers::Chessboard& msg);
private:
	ros::NodeHandle _nh;
	ros::Subscriber _chessboard;
//	Test t;
//	Checkers::Chessboard _current_chessbaord;
}; 

MoveDetermination::MoveDetermination()
{
	_chessboard = _nh.subscribe("chessboard", 1000, &MoveDetermination::callback, this);
}

void MoveDetermination::callback(const irp6_checkers::Chessboard& msg)
{
	ROS_INFO("[MoveDetermination] ------> New data received.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_determination");
	ros::NodeHandle _nh;
	MoveDetermination determination;
	ros::spin();
	return 0;
}
