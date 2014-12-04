#include "Position.hpp"
#include "Chessboard.hpp"

#include <exception>

namespace Checkers {

Position::Position() : _x(Chessboard::CHESSBOARD_SIZE), _y(Chessboard::CHESSBOARD_SIZE)
{
}

Position::Position(int x, int y) : _x(x), _y(y)
{
}

Position::Position(const Position& pos) : _x(pos._x), _y(pos._y)
{
}

bool Position::operator<(const Position& pos) const
{
	if(_y > pos._y)
		return false;
	if(_y < pos._y)
		return true;
	if(_x >= pos._x)
		return false;
	return true;
}

bool Position::operator==(const Position& pos) const
{
	if(_x == pos._x && _y == pos._y)
		return true;
	return false;
}

bool Position::operator!=(const Position& pos) const
{
	return !(*this == pos);
}

Position Position::operator=(const Position& pos)
{
	_x = pos._x;
	_y = pos._y;
	return *this;
}

Position Position::operator+=(const Position& pos)
{
	_x += pos._x;
	_y += pos._y;
	return *this;
}

Position Position::operator-=(const Position& pos)
{
	_x -= pos._x;
	_y -= pos._y;
	return *this;
}

Position Position::operator+(const Position& pos) const
{
	return Position(_x+pos._x, _y+pos._y);
}

Position Position::operator-(const Position& pos) const
{
	return Position(_x-pos._x, _y-pos._y);
}

int Position::getX() const
{
	return _x;
}

int Position::getY() const
{
	return _y;
}

void Position::setX(int x)
{
	_x = x;
}

void Position::setY(int y)
{
	_y = y;
}

bool Position::valid()
{
	if(_x < 0 || _x >= Chessboard::CHESSBOARD_SIZE || _y < 0 || _y >= Chessboard::CHESSBOARD_SIZE)
		return false;
	return true;
}

std::ostream& operator<<(std::ostream &os, const Position& pos)
{
	os<<'('<<pos.getX()<<", "<<pos.getY()<<')';
	return os;
}

}	// Checkers
