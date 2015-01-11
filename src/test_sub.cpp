#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include <boost/thread/thread.hpp>
#include <iostream>

using namespace std;

class CheckersManager
{
public:
	CheckersManager();
	~CheckersManager();
	void callback(const std_msgs::String& msg);
	void play();
	void temp();
	
private:
	// ROS
	ros::NodeHandle nh_;
	ros::Subscriber image_data_sub_;
	ros::CallbackQueue image_info;
	//ros::AsyncSpinner* spinner;
}; 

CheckersManager::CheckersManager()
{
	image_data_sub_ = nh_.subscribe("chatter", 1000, &CheckersManager::callback, this);
	//spinner = new ros::AsyncSpinner(4);
	//spinner->start();
}

CheckersManager::~CheckersManager()
{
	//delete spinner;
}

void CheckersManager::callback(const std_msgs::String& msg)
{
	ROS_INFO("New data received.");
}


void CheckersManager::temp()
{
}

void CheckersManager::play()
{
	for(int i=0; i<1000000; i++)
	{
		cout<<"I'm working!\n";
		//int a = 15+7;
		//int b = a%2;
		sleep(10);
		image_info.callOne(ros::WallDuration(0));
		//ros::getGlobalCallbackQueue()->callOne(ros::WallDuration(0));
	}
}

/********************************************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "checkers_manager");
	CheckersManager manager;
	cout<<"Hello, let's play :D!\n";
	//boost::thread* thr = new boost::thread(boost::bind(&CheckersManager::play, &manager));
	//thr->join();
	manager.play();
	//ros::waitForShutdown();
	//delete thr;
	return 0;
}
