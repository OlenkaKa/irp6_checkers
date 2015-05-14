#ifndef _CHECKERS_CHESSBOARD_HPP_
#define _CHECKERS_CHESSBOARD_HPP_

#include "Basics.hpp"
#include "TakingMove.hpp"
#include "NotTakingMove.hpp"

#include <map>
#include <vector>
#include <iostream>
#include <memory>

namespace Checkers {

class Chessboard
{
public:

	static const int CHESSBOARD_SIZE;
	static const int PLAYER_LINES_AT_START;

	Chessboard();
	void addChecker(const Position& pos, FieldValue val);
	void initGame();
	void clear();
	static FieldColor getFieldColor(const Position& pos);
	FieldValue getFieldValue(const Position& pos) const;
	void findMoves(Player player, std::vector<Move_Ptr>& moves) const;
	void findMoves(Player player, std::vector<Move_Ptr>& taking_moves, std::vector<Move_Ptr>& not_taking_moves) const;
	bool move(const Move_Ptr& move);
	bool move(const Move_Ptr& move, Chessboard& output) const;
	bool checkEndLine(Player player, const Position& pos) const;
	bool win(Player player) const;
	bool win() const;
	bool draw() const;
	static bool legalMove(Player player, const Chessboard& prev, const Chessboard& next);
	bool operator==(const Chessboard& board) const;
	void getCheckers(std::vector<Position>& pawns_1, std::vector<Position>& pawns_2, 
		std::vector<Position>& kings_1, std::vector<Position>& kings_2) const;
	friend std::ostream& operator<<(std::ostream &os, const Chessboard& board);

private:

	std::map<Position, FieldValue> _player_1;
	std::map<Position, FieldValue> _player_2;

	void _findMove(Player player, const Position& curr_pos, Direction dir, const bool limit, 
		std::vector<Move_Ptr>& taking_moves, std::vector<Move_Ptr>& not_taking_moves) const;
	void _findTakingMove(Player player, const Position& curr_pos, const Direction dir, const bool limit, 
		std::vector<Move_Ptr>& moves) const;
	void _findTakingMoveContinue(Player player, TakingMove curr_move, const Direction from_dir, const bool limit, 
		std::vector<Move_Ptr>& moves) const;
};

}	// Checkers

#endif	// _CHECKERS_CHESSBOARD_HPP_
