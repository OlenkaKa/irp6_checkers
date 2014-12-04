#include "AI.hpp"
#include "NotTakingMove.hpp"
#include "TakingMove.hpp"
#include <climits>

namespace Checkers {

AI::AI(Player player) : _player(player)
{
}

Move_Ptr AI::determineMove(const Chessboard& board)
{
	static int deep = 4;

	std::vector<Move_Ptr> moves;
	board.findMoves(_player, moves);
	if(moves.empty())
		return Move_Ptr();

	Move_Ptr max_move = moves.front();
	Chessboard board_after_move;
	board.move(max_move, board_after_move);
	int max_result = _min_max(!_player, board_after_move, deep, INT_MIN, INT_MAX);

	//std::cout<<"RESULTS\n";
	//std::cout<<max_move<<'\t'<<max_result<<std::endl;

	auto end_it = moves.end();
	for(auto it=++(moves.begin()); it!=end_it; ++it)
	{
		board.move(*it, board_after_move);
		int new_result = _min_max(!_player, board_after_move, deep, INT_MIN, INT_MAX);
		std::cout<<*it<<'\t'<<new_result<<std::endl;
		if(max_result < new_result)
		{
			max_result = new_result;
			max_move = *it;
		}
	}
	std::cout<<'\n';
	return max_move;
	/*
	static int deep = 6;

	std::vector<Move_Ptr> taking_moves, not_taking_moves;
	board.findMoves(_player, taking_moves, not_taking_moves);

	std::vector<Move_Ptr>* allowed_moves = taking_moves.empty() ? &not_taking_moves : &taking_moves;
	Move_Ptr max_move = allowed_moves->front();
	Chessboard board_after_move;
	board.move(max_move, board_after_move);
	int max_result = _min_max(!_player, board_after_move, max_move, deep, -INFINITY, INFINITY);

	auto end_it = allowed_moves->end();
	for(auto it=++(allowed_moves->begin()); it!=end_it; ++it)
	{
		board.move(*it, board_after_move);
		int new_result = _min_max(!_player, board_after_move, *it, deep, -INFINITY, INFINITY);
		if(max_result < new_result)
		{
			max_result = new_result;
			max_move = *it;
		}
	}
	return max_move;
	*/
}

int AI::h(const Chessboard& board)
{
	// TODO
	int result = 0;
	std::vector<Position> pawns_1, pawns_2, kings_1, kings_2;
	board.getCheckers(pawns_1, pawns_2, kings_1, kings_2);
	std::vector<Position> *player_pawns, *opponent_pawns, *player_kings, *opponent_kings;
	if(_player == PLAYER_1)
	{
		player_pawns = &pawns_1;
		player_kings = &kings_1;
		opponent_pawns = &pawns_2;
		opponent_kings = &kings_2;
	}
	else	// player == PLAYER_2
	{
		player_pawns = &pawns_2;
		player_kings = &kings_2;
		opponent_pawns = &pawns_1;
		opponent_kings = &kings_1;
	}
	
	// points for pieces
	result += player_pawns->size();
	result += 5*player_kings->size();
	result -= opponent_pawns->size();
	result -= 5*opponent_kings->size();

	// points for taking pieces
	// for player
	std::vector<Move_Ptr> taking_moves;
	std::vector<Move_Ptr> not_taking_moves;
	board.findMoves(_player, taking_moves, not_taking_moves);
	auto end_it = taking_moves.end();
	for(auto it=taking_moves.begin(); it!=end_it; ++it)
		result += 3*(*it)->countTakingPieces();
	// for opponent
	taking_moves.clear();
	not_taking_moves.clear();
	board.findMoves(!_player, taking_moves, not_taking_moves);
	end_it = taking_moves.end();
	for(auto it=taking_moves.begin(); it!=end_it; ++it)
		result -= 3*(*it)->countTakingPieces();

	/*
	// points for levels
	// for player
	auto end_it_pos = player_pawns->end();
	for(auto it=player_pawns->begin(); it!=end_it_pos; ++it)
	{
		switch((*it).getY() - )
		{
			case 
		}
	}
	*/

	return result;
}

int AI::_min_max(Player curr_player, const Chessboard& board, int deep, int alpha, int beta)
{
	if(deep <= 0 || board.win())
		return h(board);

	std::vector<Move_Ptr> moves;
	board.findMoves(curr_player, moves);
	if(moves.empty())
		return 0;	// draw game value

	if(curr_player == _player)	// MAX player
	{
		auto end_it = moves.end();
		for(auto it=moves.begin(); it!=end_it; ++it)
		{
			Chessboard board_after_move;
			board.move(*it, board_after_move);
			alpha = std::max(alpha, _min_max(!curr_player, board_after_move, deep-1, alpha, beta));
			if(alpha >= beta)
				return beta;
		}
		return alpha;
	}
	else	// MIN player
	{
		auto end_it = moves.end();
		for(auto it=moves.begin(); it!=end_it; ++it)
		{
			Chessboard board_after_move;
			board.move(*it, board_after_move);
			beta = std::min(beta, _min_max(!curr_player, board_after_move, deep-1, alpha, beta));
			if(alpha >= beta)
				return alpha;
		}
		return beta;
	}
	return 0;
}

}	// Checkers
