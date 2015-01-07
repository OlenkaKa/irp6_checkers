#include "CheckersPoints.h"

using namespace std;

CheckersPoints::CheckersPoints()
{
}

void CheckersPoints::addFreeKing(const irp6_checkers::Point& point)
{
	free_kings_.push_back(point);
}

void CheckersPoints::addChecker(const Checkers::Position& pos, const irp6_checkers::Point& point)
{
	checkers_.insert(std::make_pair(pos, point));
}

irp6_checkers::Point CheckersPoints::getFreeKing()
{
	if(free_kings_.size() == 0)
	{
		irp6_checkers::Point result;
		result.x = result.y = -1;
		return result;
	}
	return free_kings_.front();
/*
	if(free_kings_.size() == 0)
		return irp6_checkers::Point(-1, -1);
	irp6_checkers::Point result = free_kings_.front();
	free_kings_.pop_front();
	return result;
*/
}

irp6_checkers::Point CheckersPoints::getChecker(const Checkers::Position& pos) const
{
	auto it = checkers_.find(pos);
	if(it == checkers_.end())
	{
		irp6_checkers::Point result;
		result.x = result.y = -1;
		return result;
	}
	return it->second;
}
