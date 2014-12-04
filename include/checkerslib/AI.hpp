#ifndef _CHECKERS_AI_HPP_
#define _CHECKERS_AI_HPP_

#include "Chessboard.hpp"

namespace Checkers {

class AI
{
public:

	//const int INFINITY = 100000;
	
	AI(Player player);
	Move_Ptr determineMove(const Chessboard& board);
	int h(const Chessboard& board);

private:

	Player _player;
	int _min_max(Player curr_player, const Chessboard& board, int deep, int alpha, int beta);
};

}	// Checkers

#endif	// _CHECKERS_AI_HPP_
