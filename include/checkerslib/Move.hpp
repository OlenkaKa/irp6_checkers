#ifndef _CHECKERS_MOVE_HPP_
#define _CHECKERS_MOVE_HPP_

#include "Position.hpp"
#include <list>
#include <memory>
#include <boost/shared_ptr.hpp>

namespace Checkers {

class Move
{
public:

	enum Type
	{
		NOT_TAKING,
		TAKING
	};

	Move(const Position& start_pos, Type type);
	Move(const Move& move);
	virtual ~Move();
	Type getType();
	Position getStartPos() const;
	virtual Position getFinalPos() const = 0;
	virtual std::list<Position> getTakingPos() const = 0;
	virtual std::list<Position> getNextPos() const = 0;
	virtual int countTakingPieces() const = 0;
	virtual Move* clone() const = 0;
	friend std::ostream& operator<<(std::ostream &os, const Move& move);

protected:

	Type _type;
	Position _start_pos;

	virtual std::ostream& _ostream(std::ostream &os) const = 0;
};

typedef boost::shared_ptr<Move> Move_Ptr;
std::ostream& operator<<(std::ostream &os, const Move_Ptr& move);

}	// Checkers

#endif	// _CHECKERS_MOVE_HPP_
