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
}; 

ChessboardChangeFinder::ChessboardChangeFinder()
{
	_chessboard_change_pub = _nh.advertise<irp6_checkers::ChessboardChange>("chessboard_change", 1000);
	_chessboard_sub = _nh.subscribe("chessboard", 1000, &ChessboardChangeFinder::callback, this);
}

void ChessboardChangeFinder::callback(const irp6_checkers::Chessboard& msg)
{
	ROS_INFO("[ChessboardChangeFinder] ------> New data received.");
	_prev_chessboard.clear();
	std::vector<irp6_checkers::Checker>::const_iterator end_it = msg.Chessboard.end();
	for(std::vector<irp6_checkers::Checker>::const_iterator it = msg.Chessboard.begin(); it != end_it; ++it)
	{
		Checkers::FieldValue type;
		switch((*it).type)
		{
		case irp6_checkers::Checker::PAWN_1:
			type = Checkers::PAWN_1;
			break;
		case irp6_checkers::Checker::PAWN_2:
			type = Checkers::PAWN_2;
			break;
		case irp6_checkers::Checker::KING_1:
			type = Checkers::KING_1;
			break;
		case irp6_checkers::Checker::KING_2:
			type = Checkers::KING_2;
			break;
		}
		_prev_chessboard.addChecker(Checkers::Position((*it).x,(*it).y), type);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_determination");
	ChessboardChangeFinder finder;
	ros::spin();
	return 0;
}
