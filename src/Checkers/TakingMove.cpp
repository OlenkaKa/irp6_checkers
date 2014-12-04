#include "TakingMove.hpp"
#include "Chessboard.hpp"

namespace Checkers {

TakingMove::TakingMove(const Position& start_pos) : Move(start_pos, TAKING)
{
}

TakingMove::~TakingMove()
{
}

bool TakingMove::addMove(const Position& next_pos, const Position& taking_pos)
{
	Position prev_pos = getFinalPos();
	int prev_x = prev_pos.getX(), prev_y = prev_pos.getY();
	int next_x = next_pos.getX(), next_y = next_pos.getY();
	int taking_x = taking_pos.getX(), taking_y = taking_pos.getY();

	if(abs(next_x-prev_x) != abs(next_y-prev_y)
		|| abs(next_x-taking_x) != abs(next_y-taking_y)
		|| !(next_x > taking_x && taking_x > prev_x || prev_x > taking_x && taking_x > next_x))
		return false;
		
	_next_pos.push_back(next_pos);
	_taking_pos.push_back(taking_pos);
	return true;
}

void TakingMove::removeLastMove()
{
	if(!_next_pos.empty())
	{
		_next_pos.pop_back();
		_taking_pos.pop_back();
	}
}

std::list<Position> TakingMove::getNextPos() const
{
	return _next_pos;
}

std::list<Position> TakingMove::getTakingPos() const
{
	return _taking_pos;
}

Position TakingMove::getFinalPos() const
{
	if(_next_pos.empty())
		return _start_pos;
	return _next_pos.back();
}

int TakingMove::countTakingPieces() const
{
	return _taking_pos.size();
}

Move* TakingMove::clone() const
{
	return new TakingMove(*this);
}

std::ostream& TakingMove::_ostream(std::ostream &os) const
{
	os<<_start_pos;

	auto end_it = _next_pos.end();
	auto it=_next_pos.begin(), it2=_taking_pos.begin();
	for(; it!=end_it; ++it, ++it2)
		os<<" -> "<<(*it)<<" [rm: "<<(*it2)<<']';

	return os;
}

};	// Checkers
