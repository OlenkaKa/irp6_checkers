#ifndef _CHECKERS_NOTTAKINGMOVE_HPP_
#define _CHECKERS_NOTTAKINGMOVE_HPP_

#include "Move.hpp"

#include <iostream>

namespace Checkers {

class NotTakingMove : public Move
{
public:

	NotTakingMove(const Position& start_pos, const Position& final_pos);
	virtual ~NotTakingMove();
	Position getFinalPos() const;
	std::list<Position> getNextPos() const;
	std::list<Position> getTakingPos() const;
	int countTakingPieces() const;
	Move* clone() const;
	
protected:

	Position _final_pos;

	std::ostream& _ostream(std::ostream &os) const;
};

}	// Checkers

#endif	// _CHECKERS_NOTTAKINGMOVE_HPP_
