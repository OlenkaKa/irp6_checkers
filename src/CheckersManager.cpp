#include "ros/ros.h"
#include "irp6_checkers/ChessboardChange.h"
#include "Basics.hpp"
#include <iostream>

class CheckersManager
{
public:
	CheckersManager();
	void callback(const irp6_checkers::ChessboardChange& msg);
private:
	ros::NodeHandle _nh;
	//ros::Publisher _chessboard_change_pub;
	ros::Subscriber _chessboard_change_sub;

	Checkers::Player _current_player;
}; 

CheckersManager::CheckersManager() : _current_player(Checkers::PLAYER_1)
{
	_chessboard_change_sub = _nh.subscribe("chessboard_change", 1000, &CheckersManager::callback, this);
}

void CheckersManager::callback(const irp6_checkers::ChessboardChange& msg)
{
	ROS_INFO("[CheckersManager] ------> New data received.");
	// TODO
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "checkers_manager");
	CheckersManager manager;
	ros::spin();
	return 0;
}
