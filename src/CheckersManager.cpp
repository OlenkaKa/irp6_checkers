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
	irp6_checkers::Point box_image_pos_;
	double meter_per_pixel_x_;
	double meter_per_pixel_y_;
}; 

CheckersManager::CheckersManager(double meter_per_pixel_x, double meter_per_pixel_y) :
	player_(Checkers::PLAYER_1), ai_(Checkers::PLAYER_2), 
	meter_per_pixel_x_(0.00043), meter_per_pixel_y_(0.000425532)
{
	start_image_pos_.x = 612;
	start_image_pos_.y = 656;
	box_image_pos_.x = 0;
	box_image_pos_.y = 0;
	control_client_ = nh_.serviceClient<irp6_checkers::Control>("irp6_control");
	image_data_sub_ = nh_.subscribe("image_data", 1, &CheckersManager::callback, this);
	chessboard_.initGame();
}

void CheckersManager::callback(const irp6_checkers::ImageData& msg)
{
	//if(msg.WhiteFieldsNum != 32)
	//	return;
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
		int checker_x = -1, checker_y = -1;
		for(int i = image_data_.MinCorner.x; i < (*it).x; i += deltaX)
			++checker_x;
		for(int i = image_data_.MinCorner.y; i < (*it).y; i += deltaY)
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
	irp6_checkers::Control srv;
	irp6_checkers::ControlElem elem;
	
	// getting data from move
	std::list<Checkers::Position> next_chessboard_pos = move->getNextPos();
	std::list<Checkers::Position> taking_chessboard_pos = move->getTakingPos();
	Checkers::Position start_chessboard_pos = move->getStartPos();
	Checkers::Position final_chessboard_pos = move->getFinalPos();
	
	irp6_checkers::Point first_image_pos = checker_points_.getChecker(start_chessboard_pos);
	irp6_checkers::Point second_image_pos;
	irp6_checkers::Point final_image_pos;
	
	// get checker
	elem.X = -(first_image_pos.y - start_image_pos_.y)*meter_per_pixel_y_;
	elem.Y = (first_image_pos.x - start_image_pos_.x)*meter_per_pixel_x_;
	elem.Action = irp6_checkers::ControlElem::GET_CHECKER;
	cout<<"1:\t"<<elem.X<<" "<<elem.Y<<endl;
	cout<<"Poniewaz:\n";
	cout<<start_image_pos_.x<<" "<<start_image_pos_.y<<endl;
	cout<<first_image_pos.x<<" "<<first_image_pos.y<<endl;
	srv.request.Controls.push_back(elem);
	
	// reaching next positions
	Checkers::Position first_chessboard_pos = start_chessboard_pos;
	Checkers::Position second_chessboard_pos;
	double delta_robot_x = 0;	// to get image final position
	double delta_robot_y = 0;
	auto end_it = next_chessboard_pos.end();
	for(auto it=next_chessboard_pos.begin(); ; )
	{
		second_chessboard_pos = *it;
		elem.X = (second_chessboard_pos.getY() - first_chessboard_pos.getY())*0.04;
		delta_robot_x += elem.X;
		elem.Y = (second_chessboard_pos.getX() - first_chessboard_pos.getX())*0.04;
		delta_robot_y += elem.Y;
		if(++it == end_it)
		{
			elem.Action = irp6_checkers::ControlElem::PUT_CHECKER;
			srv.request.Controls.push_back(elem);
			break;
		}
		else
		{
			elem.Action = irp6_checkers::ControlElem::TOUCH_CHESSBOARD;
			srv.request.Controls.push_back(elem);
			first_chessboard_pos = second_chessboard_pos;
		}
	}
	
	// getting final image position
	final_image_pos.x = first_image_pos.x + (delta_robot_y/meter_per_pixel_x_);
	final_image_pos.y = first_image_pos.y + (-delta_robot_x/meter_per_pixel_y_);
	first_image_pos = final_image_pos;
	cout<<"Current pixel pos:\t"<<first_image_pos.x<<" "<<first_image_pos.y<<endl;
	
	// getting and dropping taking positions
	end_it = taking_chessboard_pos.end();
	for(auto it=taking_chessboard_pos.begin(); it!=end_it; ++it)
	{
		second_image_pos = checker_points_.getChecker(*it);
		cout<<"Now pixel pos:\t"<<first_image_pos.x<<" "<<first_image_pos.y<<endl;
		cout<<"Taking pixel pos:\t"<<second_image_pos.x<<" "<<second_image_pos.y<<endl;
		//getChecker(first_image_pos, second_image_pos);
		// get checker
		elem.X = -(second_image_pos.y-first_image_pos.y)*meter_per_pixel_y_;
		elem.Y = (second_image_pos.x-first_image_pos.x)*meter_per_pixel_x_;
		elem.Action = irp6_checkers::ControlElem::GET_CHECKER;
		srv.request.Controls.push_back(elem);
		cout<<"Result control:\t"<<elem.X<<" "<<elem.Y<<endl;
		
		// move and drop
		elem.X = -(box_image_pos_.y - second_image_pos.y)*meter_per_pixel_y_;
		elem.Y = (box_image_pos_.x - second_image_pos.x)*meter_per_pixel_x_;
		elem.Action = irp6_checkers::ControlElem::DROP_CHECKER;
		srv.request.Controls.push_back(elem);
		first_image_pos = box_image_pos_;
	}
	
	// change to king
	if(chessboard_.getFieldValue(start_chessboard_pos) == Checkers::PAWN_2 && chessboard_.checkEndLine(player_, final_chessboard_pos))
	{
		// get checker
		elem.X = -(final_image_pos.y-first_image_pos.y)*meter_per_pixel_y_;
		elem.Y = (final_image_pos.x-first_image_pos.x)*meter_per_pixel_x_;
		elem.Action = irp6_checkers::ControlElem::GET_CHECKER;
		srv.request.Controls.push_back(elem);
		
		// move and drop
		elem.X = -(box_image_pos_.y - final_image_pos.y)*meter_per_pixel_y_;
		elem.Y = (box_image_pos_.x - final_image_pos.x)*meter_per_pixel_x_;
		elem.Action = irp6_checkers::ControlElem::DROP_CHECKER;
		srv.request.Controls.push_back(elem);
		
		// get checker
		irp6_checkers::Point king_image_pos = checker_points_.getFreeKing();
		elem.X = -(king_image_pos.y-box_image_pos_.y)*meter_per_pixel_y_;
		elem.Y = (king_image_pos.x-box_image_pos_.x)*meter_per_pixel_x_;
		elem.Action = irp6_checkers::ControlElem::GET_CHECKER;
		srv.request.Controls.push_back(elem);
		
		// move and put
		elem.X = -(final_image_pos.y - king_image_pos.y)*meter_per_pixel_y_;
		elem.Y = (final_image_pos.x - king_image_pos.x)*meter_per_pixel_x_;
		elem.Action = irp6_checkers::ControlElem::PUT_CHECKER;
		srv.request.Controls.push_back(elem);
	}
	
	if (control_client_.call(srv))
	{
		ROS_INFO("[CheckersManager] Send message: %d!", srv.response.Status);
	}
	else
	{
		ROS_ERROR("[CheckersManager] Failed to call service irp6_control");
	}
	srv.request.Controls.clear();
	
	/*
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
	*/
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
