#ifndef _CHECKERS_BASICS_HPP_
#define _CHECKERS_BASICS_HPP_

#include "Position.hpp"

namespace Checkers {

enum Player
{
	PLAYER_1,
	PLAYER_2
};

enum CheckerType
{
	PAWN,
	KING
};

enum FieldColor
{
	BLACK,
	WHITE
};

enum FieldValue
{
	EMPTY = 0,
	PAWN_1 = 1,
	PAWN_2 = 2,
	KING_1 = 3,
	KING_2 = 4
};

enum Direction
{
	UP_LEFT,
	UP_RIGHT,
	DOWN_LEFT,
	DOWN_RIGHT
};

bool checkPlayer(FieldValue field1, FieldValue field2);
bool checkPlayer(FieldValue field, Player player);
FieldValue createKing(FieldValue field);
Direction operator!(const Direction& dir);
Player operator!(const Player& player);
Position findPosition(Player player, Position pos, Direction dir, int steps);
int findLineNumber(Player player, const Position& pos);

}	// Checkers

#endif	// _CHECKERS_BASICS_H_
