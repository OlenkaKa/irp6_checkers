#include "ros/ros.h"
#include "irp6_checkers/Control.h"
#include <iostream>

class MoveManager
{
public:
	MoveManager();
	//void callback(const irp6_checkers::ChessboardChange& msg);
	void temp();
private:
	ros::NodeHandle nh_;
	ros::ServiceClient control_client_;
}; 

MoveManager::MoveManager()
{
	control_client_ = nh_.serviceClient<irp6_checkers::Control>("irp6_control");
}

void MoveManager::temp()
{
	irp6_checkers::Control srv;
	irp6_checkers::ControlElem data;
	data.X = 8;
	data.Y = 3;
	data.Action = irp6_checkers::ControlElem::PUT_CHECKER;
	srv.request.Controls.push_back(data);
	if (control_client_.call(srv))
	{
		ROS_INFO("[MoveManager] Send message: %d!", srv.response.Status);
	}
	else
	{
		ROS_ERROR("[MoveManager] Failed to call service irp6_control");
	}
}

/*
void CheckersManager::callback(const irp6_checkers::Chessboard& msg)
{
	ROS_INFO("[ChessboardChangeFinder] ------> New data received.");
	// TODO
}
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_manager");
	MoveManager manager;
	sleep(10);
	manager.temp();
	ros::spin();
	return 0;
}
