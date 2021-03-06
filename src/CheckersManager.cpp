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
	bool receive_image_data_;

	// Properties
	irp6_checkers::Point start_image_pos_;
	irp6_checkers::Point box_image_pos_;
	double meter_per_pixel_x_;
	double meter_per_pixel_y_;
}; 

CheckersManager::CheckersManager(double meter_per_pixel_x, double meter_per_pixel_y) :
	player_(Checkers::PLAYER_1), ai_(Checkers::PLAYER_2), 
	receive_image_data_(false)
	// zakomentowane byly ok
	//meter_per_pixel_x_(0.000434783), meter_per_pixel_y_(0.000434783)
	///meter_per_pixel_x_(0.000415), meter_per_pixel_y_(0.000425)
{
	// zakomentowane byly ok
	nh_.getParam("start_image_pos_x", start_image_pos_.x);
	nh_.getParam("start_image_pos_y", start_image_pos_.y);
	nh_.getParam("meter_per_pixel_x", meter_per_pixel_x_);
	nh_.getParam("meter_per_pixel_y", meter_per_pixel_y_);
	//start_image_pos_.x = 643;
	//start_image_pos_.y = 682;
	///start_image_pos_.y = 660;
	box_image_pos_.x = -30;
	box_image_pos_.y = 450;
	control_client_ = nh_.serviceClient<irp6_checkers::Control>("irp6_control");
	image_data_sub_ = nh_.subscribe("image_data", 1, &CheckersManager::callback, this);
	prev_chessboard_.initGame();
	chessboard_.initGame();
	int player = 1;
	nh_.getParam("start_player", player);
	if(player == 2)
		player_ = Checkers::PLAYER_2;
}

void CheckersManager::callback(const irp6_checkers::ImageData& msg)
{
	//cout<<"-----> Fields number: "<<msg.WhiteFieldsNum<<endl;
	if(msg.WhiteFieldsNum > 32)
		return;
	receive_image_data_ = true;
	image_data_ = msg;
	//meter_per_pixel_x_ = 0.32/(image_data_.MaxCorner.x - image_data_.MinCorner.x);
	//meter_per_pixel_y_ = 0.32/(image_data_.MaxCorner.y - image_data_.MinCorner.y);
}

void CheckersManager::createChessboard()
{
	receive_image_data_ = false;
	ros::Rate loop_rate(10);
	while(ros::ok() && !receive_image_data_)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	// box coordinates
	box_image_pos_.x = image_data_.MinCorner.x - 400;
	box_image_pos_.y = (image_data_.MaxCorner.y-image_data_.MinCorner.y)/2;
	
	// chessboard info
	chessboard_.clear();
	checker_points_.clear();
	
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
			cout<<"Problem z określeniem koloru figury!\n";
			break;
		}
		/*
		cout<<"\n%%%%%%%%%%%%%%%\n";
		switch(checker_type)
		{
		case Checkers::PAWN_1:
			cout<<"PAWN 1\n";
			break;
		case Checkers::PAWN_2:
			cout<<"PAWN 2\n";
			break;
		case Checkers::KING_1:
			cout<<"KING 1\n";
			break;
		case Checkers::KING_2:
			cout<<"KING 2\n";
			break;
		}
		cout<<checker_x<<endl<<checker_y<<"\n%%%%%%%%%%%%%%%\n";
		*/

		if(!(checker_x >= 8 || checker_y >= 8 || checker_x < 0 || checker_y < 0))
		{
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
	//cout<<"Chessboard:\n";
	//cout<<chessboard_;
	//std::vector<Checkers::Position> a,b,c,d;
	//chessboard_.getCheckers(a,b,c,d);
	//cout<<"\n%%%%%%%%%%%%%%%\n"<<a.size()<<endl<<b.size()<<endl<<c.size()<<endl<<d.size()<"\n****************************\n";
}


void CheckersManager::temp()
{
	irp6_checkers::Control srv;
	irp6_checkers::ControlElem data;
	data.X = 0.0;
	data.Y = 0.1;
	data.Action = irp6_checkers::ControlElem::TOUCH_CHESSBOARD;
	srv.request.Controls.push_back(data);
	
	if (!control_client_.call(srv))
		ROS_ERROR("Failed to call service irp6_control");
}

void CheckersManager::play()
{
	while(ros::ok())
	{
		if(player_ == Checkers::PLAYER_1)
		{				
			//cout<<"Twoja kolej. Wciśnij \"Enter\" po zakończeniu ruchu.\n";
			cout<<"TURA CZŁOWIEKA\n\n";
			cout<<"Sytuacja:\n";;
			cout<<prev_chessboard_<<endl;
			
			//vector<Checkers::Move_Ptr> m;
			//chessboard_.findMoves(player_, m);
			//int t = 1;
			//cout<<"Możliwe ruchy:\n";
			//for(auto i = m.begin(); i!=m.end(); ++i)
			//{
			//	cout<<t<<". "<<(*i)<<endl;
			//	++t;
			//}
			//getchar();
			//sleep(2);
			ros::Rate loop_rate(10);
			while(ros::ok())
			{
				//cout<<"Czekam na poprawny ruch.\n";
				//cout<<"Wczesniej:\n";
				//cout<<prev_chessboard_<<endl;
				//cout<<"Teraz:\n";
				//cout<<chessboard_<<endl;

				/*if(chessboard_ == prev_chessboard_)
				{
					createChessboard();
				}
				else */if(!Checkers::Chessboard::legalMove(player_, prev_chessboard_, chessboard_))
				{
					createChessboard();
				}
				else
					break;
				loop_rate.sleep();
			}
			
			//if(!Checkers::Chessboard::legalMove(player_, prev_chessboard_, chessboard_))
			//	cout<<"Niepoprawny ruch!\n";
			//else
			prev_chessboard_.clear();
			prev_chessboard_ = chessboard_;
				
			if(chessboard_.win(player_))
			{
				cout<<"Wygrałeś, gratuluję! Dziękuję za grę!\n";
				return;
			}
		}
		
		else	// robot move
		{
			cout<<"TURA ROBOTA\n\n";
			sleep(3);
			ros::Rate loop_rate(10);
			while(ros::ok())
			{
				createChessboard();
				if(chessboard_ == prev_chessboard_)
					break;				
				loop_rate.sleep();
			}
			cout<<"Sytuacja:\n"<<chessboard_<<endl;
			
			vector<Checkers::Move_Ptr> m;
			chessboard_.findMoves(player_, m);
			int t = 1;
			cout<<"Możliwe ruchy:\n";
			for(auto i = m.begin(); i!=m.end(); ++i)
			{
				cout<<t<<". "<<(*i)<<endl;
				++t;
			}
			Checkers::Move_Ptr move = ai_.determineMove(chessboard_);
			cout<<"Decyzja:\n"<<move<<endl<<endl;
			moveRobot(move);
			
			chessboard_.move(move);
			prev_chessboard_.clear();
			prev_chessboard_ = chessboard_;
			
			if(chessboard_.win(player_))
			{
				cout<<"Wygrałem. Dziękuję za grę!\n";
				return;
			}
		}
		if(chessboard_.draw())
		{
			cout<<"Remis. Dziękuję za grę!\n";
			return;
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
	//cout<<"1:\t"<<elem.X<<" "<<elem.Y<<endl;
	//cout<<"Poniewaz:\n";
	//cout<<start_image_pos_.x<<" "<<start_image_pos_.y<<endl;
	//cout<<first_image_pos.x<<" "<<first_image_pos.y<<endl;
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
	//cout<<"Current pixel pos:\t"<<first_image_pos.x<<" "<<first_image_pos.y<<endl;
	
	// getting and dropping taking positions
	end_it = taking_chessboard_pos.end();
	for(auto it=taking_chessboard_pos.begin(); it!=end_it; ++it)
	{
		second_image_pos = checker_points_.getChecker(*it);
		//cout<<"Now pixel pos:\t"<<first_image_pos.x<<" "<<first_image_pos.y<<endl;
		//cout<<"Taking pixel pos:\t"<<second_image_pos.x<<" "<<second_image_pos.y<<endl;
		//getChecker(first_image_pos, second_image_pos);
		// get checker
		elem.X = -(second_image_pos.y-first_image_pos.y)*meter_per_pixel_y_;
		elem.Y = (second_image_pos.x-first_image_pos.x)*meter_per_pixel_x_;
		elem.Action = irp6_checkers::ControlElem::GET_CHECKER;
		srv.request.Controls.push_back(elem);
		//cout<<"Result control:\t"<<elem.X<<" "<<elem.Y<<endl;
		
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
	
	if (!control_client_.call(srv))
		ROS_ERROR("Failed to call service irp6_control");
	
	return true;
}

/********************************************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "checkers_manager");
	CheckersManager manager(2,2);
	cout<<"*******************\n"
		<<"Zagrajmy w warcaby!\n"
		<<"*******************\n";;
	sleep(15);
	manager.play();
	return 0;
}
