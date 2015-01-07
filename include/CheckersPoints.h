#ifndef _CHECKERSPOINTS_H_
#define _CHECKERSPOINTS_H_

#include "irp6_checkers/Point.h"
#include <map>
#include <list>

class CheckersPoints
{
public:
	CheckersPoints();
	void addFreeKing(const irp6_checkers::Point& point);
	void addChecker(const Checkers::Position& pos, const irp6_checkers::Point& point);
	irp6_checkers::Point getFreeKing();
	irp6_checkers::Point getChecker(const Checkers::Position& pos) const;
private:
	std::map<Checkers::Position, irp6_checkers::Point> checkers_;
	std::list<irp6_checkers::Point> free_kings_;
};

#endif	// _CHECKERSPOINTS_H_

