#ifndef _CHECKERS_CHESSBOARD_HPP_
#define _CHECKERS_CHESSBOARD_HPP_

#include "Basics.hpp"
#include "TakingMove.hpp"
#include "NotTakingMove.hpp"

#include "irp6_checkers/Chessboard.h"

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
	Chessboard(const irp6_checkers::Chessboard& data);
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
	//bool draw(Player player) const;
	bool draw() const;
	void getCheckers(std::vector<Position>& pawns_1, std::vector<Position>& pawns_2, 
		std::vector<Position>& kings_1, std::vector<Position>& kings_2) const;
	friend std::ostream& operator<<(std::ostream &os, const Chessboard& board);

	// TODO: remove
	/*
	void prepareExample()
	{	
		_player_1.insert(std::make_pair(Position(0,0),PAWN_1));
		_player_1.insert(std::make_pair(Position(7,1),KING_1));
		_player_2.insert(std::make_pair(Position(1,1),PAWN_2));
		_player_2.insert(std::make_pair(Position(3,1),PAWN_2));
		_player_2.insert(std::make_pair(Position(3,3),PAWN_2));
		_player_2.insert(std::make_pair(Position(3,5),PAWN_2));
		_player_2.insert(std::make_pair(Position(5,3),PAWN_2));
	}
	void prepareExample2()
	{
		_player_1.insert(std::make_pair(Position(1,1),PAWN_1));
		_player_1.insert(std::make_pair(Position(2,0),PAWN_1));
		_player_1.insert(std::make_pair(Position(2,2),PAWN_1));

		_player_2.insert(std::make_pair(Position(0,2),PAWN_2));
		_player_2.insert(std::make_pair(Position(1,7),PAWN_2));
		_player_2.insert(std::make_pair(Position(2,6),PAWN_2));
		_player_2.insert(std::make_pair(Position(3,7),PAWN_2));
		_player_2.insert(std::make_pair(Position(5,7),PAWN_2));
		_player_2.insert(std::make_pair(Position(6,6),PAWN_2));
		_player_2.insert(std::make_pair(Position(7,7),PAWN_2));

		_player_2.insert(std::make_pair(Position(6,0),KING_2));
	}
	void prepareExample3()
	{
		_player_1.insert(std::make_pair(Position(0,0),PAWN_1));
		_player_1.insert(std::make_pair(Position(2,0),PAWN_1));
		_player_1.insert(std::make_pair(Position(4,0),PAWN_1));
		_player_1.insert(std::make_pair(Position(7,1),PAWN_1));
		_player_1.insert(std::make_pair(Position(0,2),PAWN_1));
		_player_1.insert(std::make_pair(Position(3,3),PAWN_1));
		_player_1.insert(std::make_pair(Position(5,3),PAWN_1));
		_player_1.insert(std::make_pair(Position(7,3),PAWN_1));
		_player_1.insert(std::make_pair(Position(0,4),PAWN_1));

		_player_2.insert(std::make_pair(Position(1,7),PAWN_2));
		_player_2.insert(std::make_pair(Position(3,7),PAWN_2));
		_player_2.insert(std::make_pair(Position(5,7),PAWN_2));
		_player_2.insert(std::make_pair(Position(7,7),PAWN_2));
		_player_2.insert(std::make_pair(Position(0,6),PAWN_2));	
		_player_2.insert(std::make_pair(Position(2,6),PAWN_2));
		_player_2.insert(std::make_pair(Position(6,6),PAWN_2));

	}
	void prepareExample4()
	{
		_player_1.insert(std::make_pair(Position(4,6),PAWN_1));

		_player_2.insert(std::make_pair(Position(3,5),PAWN_2));

	}
	*/


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
