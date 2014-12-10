#include "ros/ros.h"
#include "irp6_checkers/Chessboard.h"
#include "Checkers.hpp"
#include <iostream>

class MoveDetermination
{
public:
	MoveDetermination(Checkers::Player player);
	void callback(const irp6_checkers::Chessboard& msg);
private:
	ros::NodeHandle _nh;
	ros::Subscriber _chessboard_sub;
	Checkers::Chessboard _chessboard;

	Checkers::AI _ai;
}; 

MoveDetermination::MoveDetermination(Checkers::Player player) : _ai(player)
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
	MoveDetermination determination(Checkers::PLAYER_2);
	ros::spin();
	return 0;
}
