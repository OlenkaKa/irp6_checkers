#include "ros/ros.h"
#include "irp6_checkers/Chessboard.h"
#include "irp6_checkers/ChessboardChange.h"
#include "Checkers.hpp"
#include <iostream>

class ChessboardChangeFinder
{
public:
	ChessboardChangeFinder();
	void callback(const irp6_checkers::Chessboard& msg);
private:
	ros::NodeHandle _nh;
	ros::Publisher _chessboard_change_pub;
	ros::Subscriber _chessboard_sub;

	Checkers::Chessboard _prev_chessboard;
	//bool _find_changes;
}; 

ChessboardChangeFinder::ChessboardChangeFinder()
{
	_chessboard_change_pub = _nh.advertise<irp6_checkers::ChessboardChange>("chessboard_change", 1000);
	_chessboard_sub = _nh.subscribe("chessboard", 1000, &ChessboardChangeFinder::callback, this);
	_prev_chessboard.initGame();
}

void ChessboardChangeFinder::callback(const irp6_checkers::Chessboard& msg)
{
	ROS_INFO("[ChessboardChangeFinder] ------> New data received.");
	Checkers::Chessboard new_chessboard = Checkers::Chessboard(msg);
	if(new_chessboard == _prev_chessboard)
		return;
	irp6_checkers::ChessboardChange data;
	// TODO
	_prev_chessboard = new_chessboard;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_determination");
	ChessboardChangeFinder finder;
	ros::spin();
	return 0;
}
