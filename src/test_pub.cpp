#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "Checkers.hpp"

using namespace std;
using namespace Checkers;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  Chessboard c1, c2;
  c1.initGame();
  vector<Move_Ptr> m;
  c1.findMoves(PLAYER_1, m);
  c1.move(m[0], c2);
  if(Chessboard::legalMove(PLAYER_1,c1,c2))
	cout<<"OK\n";
  else
	cout<<"NIE!!!!!\n";
  Chessboard c3,c4;
  c3.initGame();
  c4.initGame();
  if(c3==c4)
	cout<<"OK\n";
  else
	cout<<"NIE!!!!!\n";
  c3.move(m[0]);
  c4.move(m[0]);
  if(c3==c4)
	cout<<"OK\n";
  else
	cout<<"NIE!!!!!\n";
  return 0;
}
