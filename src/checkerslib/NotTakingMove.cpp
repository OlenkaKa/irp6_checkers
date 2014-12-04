#include "Chessboard.hpp"

namespace Checkers {

NotTakingMove::NotTakingMove(const Position& start_pos, const Position& final_pos) : 
	Move(start_pos, NOT_TAKING), _final_pos(final_pos)
{
	if( abs(_start_pos.getX()-_final_pos.getX()) != abs(_start_pos.getY()-_final_pos.getY()) )
		;//throw std::exception("Invalid not taking move.");
}

NotTakingMove::~NotTakingMove()
{
}

Position NotTakingMove::getFinalPos() const
{
	return _final_pos;
}

std::list<Position> NotTakingMove::getNextPos() const
{
	std::list<Position> result;
	result.push_back(_final_pos);
	return result;
}

std::list<Position> NotTakingMove::getTakingPos() const
{
	return std::list<Position>();
}

int NotTakingMove::countTakingPieces() const
{
	return 0;
}

Move* NotTakingMove::clone() const
{
	return new NotTakingMove(*this);
}

std::ostream& NotTakingMove::_ostream(std::ostream &os) const
{
	os<<_start_pos<<" -> "<<_final_pos;
	return os;
}

};	// Checkers
