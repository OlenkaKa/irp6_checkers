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
	bool moveRobot();
	
private:
	// ROS
	ros::NodeHandle nh_;
	ros::ServiceClient control_client_;
	ros::Subscriber image_data_sub_;

	irp6_checkers::ImageData image_data_;

	// Properties
	irp6_checkers::Point start_image_pos_;
	double meter_per_pixel_x_;
	double meter_per_pixel_y_;
}; 

CheckersManager::CheckersManager(double meter_per_pixel_x, double meter_per_pixel_y) :
	meter_per_pixel_x_(0.00038835), meter_per_pixel_y_(0.0004)
{
	start_image_pos_.x = 594;
	start_image_pos_.y = 649;
	control_client_ = nh_.serviceClient<irp6_checkers::Control>("irp6_control");
	image_data_sub_ = nh_.subscribe("image_data", 1, &CheckersManager::callback, this);
}

void CheckersManager::callback(const irp6_checkers::ImageData& msg)
{
	//if(msg.WhiteFieldsNum != 32)
	//	return;
	image_data_ = msg;
}

bool CheckersManager::moveRobot()
{
	ros::spinOnce();
	
	irp6_checkers::Control srv;
	irp6_checkers::ControlElem elem;
	
	
	std::vector<irp6_checkers::ColorPoint>::const_iterator end_it = image_data_.CheckerFields.end();
	for(std::vector<irp6_checkers::ColorPoint>::const_iterator it = image_data_.CheckerFields.begin(); it != end_it; ++it)
	{
			irp6_checkers::ColorPoint start_image = (*it);
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
			
			//elem.X = 0.004;
			//elem.Y = 0.004;
			//elem.Action = irp6_checkers::ControlElem::PUT_CHECKER;
			//srv.request.Controls.push_back(elem);
			
			break;
	}
	
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
	manager.moveRobot();
	//manager.temp();
	return 0;
}
