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
	static int deep = 8;

	std::vector<Move_Ptr> moves;
	board.findMoves(_player, moves);
	if(moves.empty())
		return Move_Ptr();

	Move_Ptr max_move = moves.front();
	Chessboard board_after_move;
	board.move(max_move, board_after_move);
	int max_result = _min_max(!_player, board_after_move, deep, INT_MIN, INT_MAX);
	//std::cout<<max_move<<'\t'<<max_result<<std::endl;

	//std::cout<<"RESULTS\n";
	//std::cout<<max_move<<'\t'<<max_result<<std::endl;

	int i = 0;
	auto end_it = moves.end();
	for(auto it=++(moves.begin()); it!=end_it; ++it)
	{
		++i;
		//if(i == 4) continue;
		board.move(*it, board_after_move);
		int new_result = _min_max(!_player, board_after_move, deep, INT_MIN, INT_MAX);
		//std::cout<<*it<<'\t'<<new_result<<std::endl;
		if(max_result < new_result)
		{
			max_result = new_result;
			max_move = *it;
		}
	}
	//std::cout<<'\n';
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
	if(board.win(_player))
		return INT_MAX;
	else if(board.win(!_player))
		return INT_MIN;
	
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
	result += 5*player_pawns->size();
	result += 50*player_kings->size();
	result -= 5*opponent_pawns->size();
	result -= 50*opponent_kings->size();

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

	// points for pawn's level
	// for player
	auto end_it_pos = player_pawns->end();
	for(auto it=player_pawns->begin(); it!=end_it_pos; ++it)
	{
		switch(findLineNumber(_player, *it))
		{
		case 0:
		case 1:
			break;
		case 2:
		case 3:
			result += 1;
			break;
		case 4:
		case 5:
			result += 3;
			break;
		case 6:
		case 7:
			result += 20;
			break;
		default:
			break;
		}
	}
	// for opponent
	end_it_pos = opponent_pawns->end();
	for(auto it=opponent_pawns->begin(); it!=end_it_pos; ++it)
	{
		switch(findLineNumber(!_player, *it))
		{
		case 0:
		case 1:
			break;
		case 2:
		case 3:
			result -= 1;
			break;
		case 4:
		case 5:
			result -= 3;
			break;
		case 6:
		case 7:
			result -= 20;
			break;
		default:
			break;
		}
	}

	// points for king's distance from edges
	// for player
	auto end_it_king_pos = player_kings->end();
	for(auto it=player_kings->begin(); it!=end_it_king_pos; ++it)
	{
		switch((*it).getX())
		{
		case 0:
		case 7:
			result += 3;
			break;
		case 1:
		case 6:
			result += 1;
			break;
		default:
			break;
		}
	}
	// for opponent
	end_it_king_pos = opponent_kings->end();
	for(auto it=opponent_kings->begin(); it!=end_it_king_pos; ++it)
	{
		switch((*it).getX())
		{
		case 0:
		case 7:
			result -= 3;
			break;
		case 1:
		case 6:
			result -= 1;
			break;
		default:
			break;
		}
	}

	return result;
}

int AI::_min_max(Player curr_player, const Chessboard& board, int deep, int alpha, int beta)
{
	if(deep <= 0 || board.win() || board.draw())
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
