#include "ros/ros.h"
#include "irp6_checkers/ImageData.h"
#include "irp6_checkers/Chessboard.h"
#include "Checkers.hpp"
#include <iostream>

using namespace std;

class ImageInterpreter
{
public:
	ImageInterpreter(double meter_per_pixel);
	void callback(const irp6_checkers::ImageData& msg);
	void changeMeterPerPixel(double new_value);
private:
	ros::NodeHandle nh_;
	ros::Publisher chessboardPub_;
	ros::Subscriber image_dataSub_;

	double meter_per_pixel_;
}; 

ImageInterpreter::ImageInterpreter(double meter_per_pixel) : meter_per_pixel_(meter_per_pixel)
{
	chessboardPub_ = nh_.advertise<irp6_checkers::Chessboard>("chessboard", 1000);
	image_dataSub_ = nh_.subscribe("image_data", 1000, &ImageInterpreter::callback, this);
	ROS_INFO("[ImageInterpreter] Initialize");
}

void ImageInterpreter::callback(const irp6_checkers::ImageData& msg)
{
	if(msg.WhiteFieldsNum < 32)
	{
		cout<<"Too few white fields: "<<msg.WhiteFieldsNum<<".\n";
		return;
	}
	irp6_checkers::Chessboard chessboard;
	int width = msg.MaxCorner.x-msg.MinCorner.x;
	int deltaX = width/8;
	int height = msg.MaxCorner.y-msg.MinCorner.y;
	int deltaY = height/8;

	std::vector<irp6_checkers::ColorPoint>::const_iterator end_it = msg.CheckerFields.end();
	for(std::vector<irp6_checkers::ColorPoint>::const_iterator it = msg.CheckerFields.begin(); it != end_it; ++it)
	{
		irp6_checkers::Checker checker;
		checker.x = 0;
		for(int i = msg.MinCorner.x+deltaX; i < (*it).x; i += deltaX)
			++checker.x;
		checker.y = 0;
		for(int i = msg.MinCorner.y+deltaY; i < (*it).y; i += deltaY)
			++checker.y;
		checker.y = 7 - checker.y;
		switch((*it).color)
		{
		case irp6_checkers::ColorPoint::COLOR_GREEN:
			checker.type = irp6_checkers::Checker::PAWN_1;
			break;
		case irp6_checkers::ColorPoint::COLOR_RED:
			checker.type = irp6_checkers::Checker::PAWN_2;
			break;
		case irp6_checkers::ColorPoint::COLOR_BLUE:
			checker.type = irp6_checkers::Checker::KING_1;
			break;
		case irp6_checkers::ColorPoint::COLOR_YELLOW:
			checker.type = irp6_checkers::Checker::KING_2;
			break;
		default:
			cout<<"Invalid checker color.\n";
			break;
		}
		chessboard.Chessboard.push_back(checker);
	}
	cout<<"Chessboard:\n";
	Checkers::Chessboard cc(chessboard);
	cout<<cc;
	chessboardPub_.publish(chessboard);
	ros::spinOnce();
	ROS_INFO("[ImageInterpreter] <------ End of data.");
}

void ImageInterpreter::changeMeterPerPixel(double new_value)
{
	meter_per_pixel_ = new_value;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_interpreter");
	ImageInterpreter interpreter(0.3);
	ros::spin();
	return 0;
}
