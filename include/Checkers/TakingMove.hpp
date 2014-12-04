#ifndef _CHECKERS_TAKINGMOVE_HPP_
#define _CHECKERS_TAKINGMOVE_HPP_

#include "Move.hpp"

#include <iostream>
#include <list>

namespace Checkers {

class TakingMove : public Move
{
public:

	TakingMove(const Position& start_pos);
	virtual ~TakingMove();
	bool addMove(const Position& next_pos, const Position& taking_pos);
	void removeLastMove();
	Position getFinalPos() const;
	std::list<Position> getNextPos() const;
	std::list<Position> getTakingPos() const;
	Move* clone() const;
	int countTakingPieces() const;
	
protected:

	std::list<Position> _next_pos;
	std::list<Position> _taking_pos;

	std::ostream& _ostream(std::ostream &os) const;
};

}	// Checkers

#endif	// _CHECKERS_TAKINGMOVE_HPP_
