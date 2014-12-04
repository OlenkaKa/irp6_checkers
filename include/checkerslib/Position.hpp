#ifndef _CHECKERS_POSITION_HPP_
#define _CHECKERS_POSITION_HPP_

#include <iostream>

namespace Checkers {

class Position
{
public:

	Position();
	Position(int x, int y);
	Position(const Position& pos);
	bool operator<(const Position& pos) const;
	bool operator==(const Position& pos) const;
	bool operator!=(const Position& pos) const;
	Position operator=(const Position& pos);
	Position operator+=(const Position& pos);
	Position operator-=(const Position& pos);
	Position operator+(const Position& pos) const;
	Position operator-(const Position& pos) const;
	int getX() const;
	int getY() const;
	void setX(int x);
	void setY(int y);
	bool valid();
	friend std::ostream& operator<<(std::ostream &os, const Position& pos);

private:

	int _x;
	int _y;
};

}	// Checkers

#endif	// _CHECKERS_POSITION_HPP_
