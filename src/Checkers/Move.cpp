#include "Chessboard.hpp"

namespace Checkers {

Move::Move(const Position& start_pos, Type type) : _start_pos(start_pos), _type(type)
{
	if(Chessboard::getFieldColor(_start_pos) != BLACK)
		;//throw std::exception("Pawn is not on black field.");
}

Move::Move(const Move& move) : _start_pos(move._start_pos), _type(move._type)
{
}

Move::~Move()
{
}

Move::Type Move::getType()
{
	return _type;
}

Position Move::getStartPos() const
{
	return _start_pos;
}

std::ostream& operator<<(std::ostream &os, const Move& move)
{
	return move._ostream(os);
}

std::ostream& operator<<(std::ostream &os, const Move_Ptr& move)
{
	os<<*move;
	return os;
}

}	// Checkers
