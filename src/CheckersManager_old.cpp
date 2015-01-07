#include "ros/ros.h"
#include "irp6_checkers/ImageData.h"
#include "irp6_checkers/Control.h"
#include "Checkers.hpp"
#include <iostream>

using namespace std;

/********************************************************************/
struct ImagePoint
{
	ImagePoint(double xx = 0, double yy = 0);
	double x;
	double y;
};

ImagePoint::ImagePoint(double xx, double yy): x(xx), y(yy)
{
}

class CheckersPoints
{
public:
	CheckersPoints();
	void addPoint(ImagePoint point);
	void addPoint(Checkers::Position pos, ImagePoint point);
	ImagePoint getPoint();
	ImagePoint getPoint(Checkers::Position pos);
private:
	std::map<Checkers::Position, ImagePoint> checkers_;
	std::list<ImagePoint> free_kings_;
};

CheckersPoints::CheckersPoints()
{
}

void CheckersPoints::addPoint(ImagePoint point)
{
	free_kings_.push_back(point);
}

void CheckersPoints::addPoint(Checkers::Position pos, ImagePoint point)
{
	checkers_.insert(std::make_pair(pos, point));
}

ImagePoint CheckersPoints::getPoint()
{
	if(free_kings_.size() == 0)
		return ImagePoint(-1, -1);
	ImagePoint result = free_kings_.front();
	free_kings_.pop_front();
	return result;
}

ImagePoint CheckersPoints::getPoint(Checkers::Position pos)
{
	auto it = checkers_.find(pos);
	if(it == checkers_.end())
		return ImagePoint(-1, -1);
	return it->second;
}

/********************************************************************/
class CheckersManager
{
public:
	CheckersManager(double meter_per_pixel_x, double meter_per_pixel_y);
	void callback(const irp6_checkers::ImageData& msg);
	void play();
	void temp();
	
private:
	// ROS
	ros::NodeHandle nh_;
	ros::ServiceClient control_client_;
	ros::Subscriber image_data_sub_;

	// Checkers
	Checkers::Player player_;
	Checkers::Chessboard chessboard_;
	Checkers::AI ai_;
	
	// Control
	CheckersPoints checker_points_;

	// Properties
	double meter_per_pixel_x_;
	double meter_per_pixel_y_;
	ImagePoint box_;
}; 

CheckersManager::CheckersManager(double meter_per_pixel_x, double meter_per_pixel_y) : 
	player_(Checkers::PLAYER_2), ai_(Checkers::PLAYER_1), 
	meter_per_pixel_x_(meter_per_pixel_x), meter_per_pixel_y_(meter_per_pixel_y)	// opponent start game
{
	control_client_ = nh_.serviceClient<irp6_checkers::Control>("irp6_control");
	image_data_sub_ = nh_.subscribe("image_data", 1000, &CheckersManager::callback, this);
	chessboard_.initGame();
}

void CheckersManager::callback(const irp6_checkers::ImageData& msg)
{
	ROS_INFO("[CheckersManager] ------> New data received.");
	if(msg.WhiteFieldsNum < 32)
	{
		cout<<"Too few white fields: "<<msg.WhiteFieldsNum<<".\n";
		return;
	}
	chessboard_.clear();
	int width = msg.MaxCorner.x-msg.MinCorner.x;
	int deltaX = width/8;
	int height = msg.MaxCorner.y-msg.MinCorner.y;
	int deltaY = height/8;

	std::vector<irp6_checkers::ColorPoint>::const_iterator end_it = msg.CheckerFields.end();
	for(std::vector<irp6_checkers::ColorPoint>::const_iterator it = msg.CheckerFields.begin(); it != end_it; ++it)
	{
		int checker_x = 0, checker_y = 0;
		for(int i = msg.MinCorner.x+deltaX; i < (*it).x; i += deltaX)
			++checker_x;
		for(int i = msg.MinCorner.y+deltaY; i < (*it).y; i += deltaY)
			++checker_y;
		checker_y = 7 - checker_y;	// bo odwrócony obraz
		
		Checkers::FieldValue checker_type;
		switch((*it).color)
		{
		case irp6_checkers::ColorPoint::COLOR_GREEN:
			checker_type = Checkers::PAWN_1;
			break;
		case irp6_checkers::ColorPoint::COLOR_RED:
			checker_type = Checkers::PAWN_2;
			break;
		case irp6_checkers::ColorPoint::COLOR_BLUE:
			checker_type = Checkers::KING_1;
			break;
		case irp6_checkers::ColorPoint::COLOR_YELLOW:
			checker_type = Checkers::KING_2;
			break;
		default:
			cout<<"Invalid checker color.\n";
			break;
		}
		if(!(checker_x >= 8 || checker_y >= 8 || checker_x < 0 || checker_y < 0))
		{
			chessboard_.addChecker(Checkers::Position(checker_x,checker_y), checker_type);
			checker_points_.addPoint(Checkers::Position(checker_x,checker_y), ImagePoint((*it).x,(*it).y));
		}
		else if(checker_type == Checkers::KING_1)
		{
			checker_points_.addPoint(ImagePoint((*it).x,(*it).y));
		}
	}
	cout<<"Chessboard:\n";
	cout<<chessboard_;
	cout<<"************************************************************\n";
	ImagePoint img = checker_points_.getPoint();
	while(img.x >= 0)
	{
		cout<<"["<<img.x<<", "<<img.y<<"]\n";
		img = checker_points_.getPoint();
	}
	cout<<"************************************************************\n";
	ros::spinOnce();
	ROS_INFO("[CheckersManager] <------ End of data.");
}


void CheckersManager::temp()
{
	irp6_checkers::Control srv;
	irp6_checkers::ControlElem data;
	data.X = 0.08;
	data.Y = 0.03;
	data.Action = irp6_checkers::ControlElem::GET_CHECKER;
	srv.request.Controls.push_back(data);
	data.X = -0.03;
	data.Y = 0.05;
	data.Action = irp6_checkers::ControlElem::PUT_CHECKER;
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
		}
		else
		{
		}
		break;
	}
}

/********************************************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "checkers_manager");
	CheckersManager manager(2,2);
	sleep(25);
	manager.play();
	manager.temp();
	ros::spin();
	return 0;
}
