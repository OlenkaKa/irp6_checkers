#include "ros/ros.h"
#include "irp6_checkers/Chessboard.h"
#include "Checkers.hpp"
#include <iostream>

class MoveDetermination
{
public:
	MoveDetermination();
	void callback(const irp6_checkers::Chessboard& msg);
private:
	ros::NodeHandle _nh;
	ros::Subscriber _chessboard_sub;
	Checkers::Chessboard _chessboard;
}; 

MoveDetermination::MoveDetermination()
{
	_chessboard_sub = _nh.subscribe("chessboard", 1000, &MoveDetermination::callback, this);
}

void MoveDetermination::callback(const irp6_checkers::Chessboard& msg)
{
	ROS_INFO("[MoveDetermination] ------> New data received.");
	_chessboard = Checkers::Chessboard(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_determination");
	MoveDetermination determination;
	ros::spin();
	return 0;
}
