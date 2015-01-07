#include "Basics.hpp"
#include "Chessboard.hpp"
#include <exception>

namespace Checkers {

bool checkPlayer(FieldValue field1, FieldValue field2)
{
	if(field1 == EMPTY || field2 == EMPTY || (field1+field2)%2 == 1)
		return false;
	return true;
}

bool checkPlayer(FieldValue field, Player player)
{
	if(field == EMPTY
		|| player == PLAYER_1 && (field == PAWN_2 || field == KING_2)
		|| player == PLAYER_2 && (field == PAWN_1 || field == KING_1))
		return false;
	
	return true;
}

FieldValue createKing(FieldValue field)
{
	if(field == PAWN_1)
		return KING_1;
	if(field == PAWN_2)
		return KING_2;
	return field;
}

Direction operator!(const Direction& dir)
{
	switch(dir)
	{
	case UP_LEFT:
		return DOWN_RIGHT;
	case UP_RIGHT:
		return DOWN_LEFT;
	case DOWN_LEFT:
		return UP_RIGHT;
	case DOWN_RIGHT:
		return UP_LEFT;
	default:
		;//throw std::exception("Unknown direction.");
	}
}

Position findPosition(Player player, Position pos, Direction dir, int steps)
{
	if(player == PLAYER_1)
	{
		switch(dir)
		{
		case UP_LEFT:
			return pos+Position(-steps,steps);
		case UP_RIGHT:
			return pos+Position(steps,steps);
		case DOWN_LEFT:
			return pos+Position(-steps,-steps);
		case DOWN_RIGHT:
			return pos+Position(steps,-steps);
		default:
			;//throw std::exception("Unknown direction.");
		};
	}
	// player == PLAYER_2
	switch(dir)
	{
	case UP_LEFT:
		return pos+Position(steps,-steps);
	case UP_RIGHT:
		return pos+Position(-steps,-steps);
	case DOWN_LEFT:
		return pos+Position(steps,steps);
	case DOWN_RIGHT:
		return pos+Position(-steps,steps);
	default:
		;//throw std::exception("Unknown direction.");
	};
}

Player operator!(const Player& p)
{
	if(p == PLAYER_1)
		return PLAYER_2;
	return PLAYER_1;
}

int findLineNumber(Player player, const Position& pos)
{
	int y = pos.getY();
	if(player == PLAYER_1)
		return y;
	return Chessboard::CHESSBOARD_SIZE - 1 - y;
}

}	// Checkers
