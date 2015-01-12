#include "ros/ros.h"
#include "irp6_checkers/ImageData.h"
#include "irp6_checkers/Control.h"
#include "Checkers.hpp"
#include "CheckersPoints.h"
#include <iostream>

using namespace std;

class CheckersManager
{
public:
	CheckersManager(double meter_per_pixel_x, double meter_per_pixel_y);
	void callback(const irp6_checkers::ImageData& msg);
	void createChessboard();
	void play();
	void temp();
	bool endGame();
	bool moveRobot(Checkers::Move_Ptr move);
	
private:
	// ROS
	ros::NodeHandle nh_;
	ros::ServiceClient control_client_;
	ros::Subscriber image_data_sub_;

	// Checkers
	Checkers::Player player_;
	Checkers::Chessboard prev_chessboard_;
	Checkers::Chessboard chessboard_;
	Checkers::AI ai_;
	irp6_checkers::ImageData image_data_;
	
	// Control
	CheckersPoints checker_points_;

	// Properties
	irp6_checkers::Point start_image_pos_;
	double meter_per_pixel_x_;
	double meter_per_pixel_y_;
}; 

CheckersManager::CheckersManager(double meter_per_pixel_x, double meter_per_pixel_y) :
	player_(Checkers::PLAYER_1), ai_(Checkers::PLAYER_2), 
	meter_per_pixel_x_(0.00038835), meter_per_pixel_y_(0.0004)
{
	start_image_pos_.x = 605;
	start_image_pos_.y = 669;
	control_client_ = nh_.serviceClient<irp6_checkers::Control>("irp6_control");
	image_data_sub_ = nh_.subscribe("image_data", 1, &CheckersManager::callback, this);
	chessboard_.initGame();
}

void CheckersManager::callback(const irp6_checkers::ImageData& msg)
{
	if(msg.WhiteFieldsNum != 32)
		return;
	image_data_ = msg;
}

void CheckersManager::createChessboard()
{
	prev_chessboard_ = chessboard_;
	chessboard_.clear();
	
	int width = image_data_.MaxCorner.x-image_data_.MinCorner.x;
	int deltaX = width/8;
	int height = image_data_.MaxCorner.y-image_data_.MinCorner.y;
	int deltaY = height/8;

	std::vector<irp6_checkers::ColorPoint>::const_iterator end_it = image_data_.CheckerFields.end();
	for(std::vector<irp6_checkers::ColorPoint>::const_iterator it = image_data_.CheckerFields.begin(); it != end_it; ++it)
	{
		int checker_x = 0, checker_y = 0;
		for(int i = image_data_.MinCorner.x+deltaX; i < (*it).x; i += deltaX)
			++checker_x;
		for(int i = image_data_.MinCorner.y+deltaY; i < (*it).y; i += deltaY)
			++checker_y;
		checker_y = 7 - checker_y;	// bo odwrócony obraz
		
		Checkers::FieldValue checker_type;
		switch((*it).color)
		{
		case irp6_checkers::ColorPoint::COLOR_GREEN:
			checker_type = Checkers::PAWN_2;
			break;
		case irp6_checkers::ColorPoint::COLOR_RED:
			checker_type = Checkers::PAWN_1;
			break;
		case irp6_checkers::ColorPoint::COLOR_BLUE:
			checker_type = Checkers::KING_2;
			break;
		case irp6_checkers::ColorPoint::COLOR_YELLOW:
			checker_type = Checkers::KING_1;
			break;
		default:
			cout<<"Invalid checker color.\n";
			break;
		}
		if(!(checker_x >= 8 || checker_y >= 8 || checker_x < 0 || checker_y < 0))
		{
			cout<<"Jestem tu\n";
			chessboard_.addChecker(Checkers::Position(checker_x,checker_y), checker_type);
			irp6_checkers::Point point;
			point.x = (*it).x;
			point.y = (*it).y;
			checker_points_.addChecker(Checkers::Position(checker_x,checker_y), point);
		}
		else if(checker_type == Checkers::KING_2)
		{
			irp6_checkers::Point point;
			point.x = (*it).x;
			point.y = (*it).y;
			checker_points_.addFreeKing(point);
		}
	}
	cout<<"Chessboard:\n";
	cout<<chessboard_;
}


void CheckersManager::temp()
{
	irp6_checkers::Control srv;
	irp6_checkers::ControlElem data;
	data.X = 0.0;
	data.Y = 0.1;
	data.Action = irp6_checkers::ControlElem::TOUCH_CHESSBOARD;
	srv.request.Controls.push_back(data);
	
	if (control_client_.call(srv))
	{
		ROS_INFO("[CheckersManager] Send message: %d!", srv.response.Status);
	}
	else
	{
		ROS_ERROR("[CheckersManager] Failed to call service irp6_control");
	}
}

void CheckersManager::play()
{
	while(true)
	{
		if(player_ == Checkers::PLAYER_1)
		{				
			cout<<"I'm waiting for your move. Press key when finished.\n";
			getchar();
			sleep(2);
			ros::spinOnce();
			createChessboard();
			if(!Checkers::Chessboard::legalMove(player_, prev_chessboard_, chessboard_))
				cout<<"Incorrect move!\n";
			else 
				cout<<"Good move!\n";
		}
		else	// robot move
		{
			/*
			if(endGame())
			{
				cout<<"Congrtulation, you win!!! :)\n";
				break;
			}
			* */
			ros::spinOnce();
			createChessboard();
			Checkers::Move_Ptr move = ai_.determineMove(chessboard_);
			cout<<"Decision:\n"<<move<<endl;
			moveRobot(move);
		}
		player_ = !player_;
	}
}

bool CheckersManager::endGame()
{
	if(chessboard_.win() || chessboard_.draw())
		return true;
	return false;
}

bool CheckersManager::moveRobot(Checkers::Move_Ptr move)
{
	//cout<<"\nI cannot move now ;( Sorry....\n";
	irp6_checkers::Control srv;
	irp6_checkers::ControlElem elem;
	
	irp6_checkers::Point start_image = checker_points_.getChecker(move->getStartPos());
	//irp6_checkers::Point end_image = checker_points_.getChecker(move->getFinalPos());
	
	// go to piece
	elem.X = -(start_image.y - start_image_pos_.y)*meter_per_pixel_y_;
	elem.Y = (start_image.x - start_image_pos_.x)*meter_per_pixel_x_;
	elem.Action = irp6_checkers::ControlElem::GET_CHECKER;
	cout<<"1:\t"<<elem.X<<" "<<elem.Y<<endl;
	cout<<"Poniewaz:\n";
	cout<<start_image_pos_.x<<" "<<start_image_pos_.y<<endl;
	cout<<start_image.x<<" "<<start_image.y<<endl;
	srv.request.Controls.push_back(elem);
	
	// move piece
	elem.X = (move->getFinalPos().getY() - move->getStartPos().getY())*0.04;
	elem.Y = (move->getFinalPos().getX() - move->getStartPos().getX())*0.04;
	elem.Action = irp6_checkers::ControlElem::PUT_CHECKER;
	cout<<"2:\t"<<elem.X<<" "<<elem.Y<<endl;
	//cout<<"Poniewaz:\n";
	//cout<<start_image.x<<" "<<start_image.y<<endl;
	//cout<<end_image.x<<" "<<end_image.y<<endl;
	srv.request.Controls.push_back(elem);
	
	if (control_client_.call(srv))
	{
		ROS_INFO("[CheckersManager] Send message: %d!", srv.response.Status);
	}
	else
	{
		ROS_ERROR("[CheckersManager] Failed to call service irp6_control");
	}
	return true;
}

/********************************************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "checkers_manager");
	CheckersManager manager(2,2);
	cout<<"Hello, let's play :D!\n";
	sleep(15);
	manager.play();
	//manager.temp();
	return 0;
}
