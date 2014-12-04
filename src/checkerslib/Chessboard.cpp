#include "Chessboard.hpp"

namespace Checkers {

Chessboard::Chessboard()
{
}

Chessboard::Chessboard(const irp6_checkers::Chessboard& data)
{
	std::vector<irp6_checkers::Checker>::const_iterator end_it = data.Chessboard.end();
		for(std::vector<irp6_checkers::Checker>::const_iterator it = data.Chessboard.begin(); it != end_it; ++it)
		{
			Checkers::FieldValue type;
			switch((*it).type)
			{
			case irp6_checkers::Checker::PAWN_1:
				type = Checkers::PAWN_1;
				break;
			case irp6_checkers::Checker::PAWN_2:
				type = Checkers::PAWN_2;
				break;
			case irp6_checkers::Checker::KING_1:
				type = Checkers::KING_1;
				break;
			case irp6_checkers::Checker::KING_2:
				type = Checkers::KING_2;
				break;
			}
			addChecker(Checkers::Position((*it).x,(*it).y), type);
		}
}

void Chessboard::addChecker(const Position& pos, FieldValue val)
{
	if(checkPlayer(val, PLAYER_1))
		_player_1.insert(std::make_pair(pos,val));
	else
		_player_2.insert(std::make_pair(pos,val));
}

void Chessboard::initGame()
{
	// set pawns for player 1
	for(int y=0; y < PLAYER_LINES_AT_START; ++y)
		for(int x=(y%2 ? 1 : 0); x < CHESSBOARD_SIZE; x+=2)
			_player_1.insert(std::make_pair(Position(x,y),PAWN_1));

	// set pawns for player 2
	for(int y=CHESSBOARD_SIZE-PLAYER_LINES_AT_START; y < CHESSBOARD_SIZE; ++y)
		for(int x=(y%2 ? 1 : 0); x < CHESSBOARD_SIZE; x+=2)
			_player_2.insert(std::make_pair(Position(x,y),PAWN_2));

	//std::cout<<"INIT GAME:\t"<<_player_1.size()<<' '<<_player_2.size()<<"!!!\n";
}

void Chessboard::clear()
{
	_player_1.clear();
	_player_2.clear();
}

FieldColor Chessboard::getFieldColor(const Position& c)
{
	if( ((c.getX()+c.getY())%2) == 0 )
		return BLACK;
	return WHITE;
}

FieldValue Chessboard::getFieldValue(const Position& c) const
{
	if(getFieldColor(c) == BLACK)
	{
		std::map<Position, FieldValue>::const_iterator it = _player_1.find(c);
		if(it != _player_1.end())
			return it->second;

		it = _player_2.find(c);
		if(it != _player_2.end())
			return it->second;
	}
	return EMPTY;
}

void Chessboard::findMoves(Player player, std::vector<Move_Ptr>& moves) const
{
	std::vector<Move_Ptr> taking_moves, not_taking_moves;
	findMoves(player, taking_moves, not_taking_moves);
	if(taking_moves.empty())
		moves = not_taking_moves;
	else
		moves = taking_moves;
}

void Chessboard::findMoves(Player player, std::vector<Move_Ptr>& taking_moves, std::vector<Move_Ptr>& not_taking_moves) const
{
	const std::map<Position, FieldValue>* player_fields = (player == PLAYER_1 ? &_player_1 : &_player_2);
	auto end_it = player_fields->end();
	
	for(auto it=player_fields->begin(); it!=end_it; ++it)
	{
		if(it->second < 3)	// it->second == PAWN_n
		{
			_findMove(player, it->first, Direction::UP_LEFT, true, taking_moves, not_taking_moves);
			_findMove(player, it->first, Direction::UP_RIGHT, true, taking_moves, not_taking_moves);
			_findTakingMove(player, it->first, Direction::DOWN_LEFT, true, taking_moves);
			_findTakingMove(player, it->first, Direction::DOWN_RIGHT, true, taking_moves);
		}
		else	// it->second == KING_n
		{
			_findMove(player, it->first, Direction::UP_LEFT, false, taking_moves, not_taking_moves);
			_findMove(player, it->first, Direction::UP_RIGHT, false, taking_moves, not_taking_moves);
			_findMove(player, it->first, Direction::DOWN_LEFT, false, taking_moves, not_taking_moves);
			_findMove(player, it->first, Direction::DOWN_RIGHT, false, taking_moves, not_taking_moves);
		}
	}
}

/*
void Chessboard::getCheckers(Player player, std::list<Position>& pawns, std::list<Position>& kings) const
{
	const std::map<Position, FieldValue>* player_fields = (player == PLAYER_1 ? &_player_1 : &_player_2);
	auto end_it = player_fields->end();
	for(auto it=player_fields->begin(); it!=end_it; ++it)
	{
		if(it->second < 3)
			pawns.push_back(it->first);
		else
			kings.push_back(it->first);
	}
}

bool Chessboard::move(Move* move)
{
	Position start_pos = move->getStartPos();
	Position final_pos = move->getFinalPos();
	FieldValue start_val = getFieldValue(start_pos);
	std::map<Position, FieldValue>* player_fields = ((start_val == PAWN_1 || start_val == KING_1) ? &_player_1 : &_player_2);
	std::map<Position, FieldValue>* opponent_fields = ((start_val == PAWN_1 || start_val == KING_1) ? &_player_2 : &_player_1);
	std::list<Position> removed_pos = move->getTakingPos();

	player_fields->erase(start_pos);
	if((start_val == PLAYER_1 && final_pos.getY() == CHESSBOARD_SIZE - 1) || (start_val == PLAYER_2 && final_pos.getY() == 0))
	{
		player_fields->insert(std::make_pair(final_pos, ((start_val == PLAYER_1) ? KING_1 : KING_2)) );
	}
	else
	{
		player_fields->insert(std::make_pair(final_pos,start_val));
	}

	auto end_it = removed_pos.end();
	for(auto it = removed_pos.begin(); it!=end_it; ++it)
		opponent_fields->erase(opponent_fields->find(*it));

	return true;
}

bool Chessboard::move(Move* move, Chessboard& output) const
{
	output = *this;
	Position start_pos = move->getStartPos();
	Position final_pos = move->getFinalPos();
	FieldValue start_val = getFieldValue(start_pos);
	std::map<Position, FieldValue>* player_fields = ((start_val == PAWN_1 || start_val == KING_1) ? &output._player_1 : &output._player_2);
	std::map<Position, FieldValue>* opponent_fields = ((start_val == PAWN_1 || start_val == KING_1) ? &output._player_2 : &output._player_1);
	std::list<Position> removed_pos = move->getTakingPos();

	player_fields->erase(start_pos);
	if((start_val == PLAYER_1 && final_pos.getY() == CHESSBOARD_SIZE - 1) || (start_val == PLAYER_2 && final_pos.getY() == 0))
	{
		player_fields->insert(std::make_pair(final_pos, ((start_val == PLAYER_1) ? KING_1 : KING_2)) );
	}
	else
	{
		player_fields->insert(std::make_pair(final_pos,start_val));
	}

	auto end_it = removed_pos.end();
	for(auto it = removed_pos.begin(); it!=end_it; ++it)
		opponent_fields->erase(opponent_fields->find(*it));

	return true;
	return false;
}
*/

bool Chessboard::checkEndLine(Player player, const Position& pos) const
{
	if(player == PLAYER_1)
	{
		if(pos.getY() == CHESSBOARD_SIZE-1)
			return true;
		return false;
	}
	// player == PLAYER_2
	if(pos.getY() == 0)
		return true;
	return false;
}

bool Chessboard::move(const Move_Ptr& move)
{
	std::cout<<"BEFORE MOVE:\t"<<_player_1.size()<<' '<<_player_2.size()<<"!!!\n";
	//return this->move(move, *this);
	Player player;
	std::map<Position, FieldValue> *player_fields, *opponent_fields;
	Position start_pos = move->getStartPos(), final_pos = move->getFinalPos();
	FieldValue start_val = getFieldValue(start_pos), final_val =  getFieldValue(final_pos);

	if(start_val == EMPTY)
		return false;
	
	if(checkPlayer(start_val, PLAYER_1))
	{
		player = PLAYER_1;
		player_fields = &_player_1;
		opponent_fields = &_player_2;
	}
	else
	{
		player = PLAYER_2;
		player_fields = &_player_2;
		opponent_fields = &_player_1;
	}

	auto next_pos = move->getNextPos();
	auto taking_pos = move->getTakingPos();

	// TODO: sprawdzenie poprawno�ci
	auto end_it = taking_pos.end();
	for(auto it=taking_pos.begin(); it!=end_it;)
	{
		auto it2 = it;
		++it;
		opponent_fields->erase(opponent_fields->find(*it2));
	}

	player_fields->erase(player_fields->find(start_pos));
	if(checkEndLine(player, final_pos))
		player_fields->insert(std::make_pair(final_pos, createKing(start_val)));
	else
		player_fields->insert(std::make_pair(final_pos, start_val));

	std::cout<<"AFTER MOVE:\t"<<_player_1.size()<<' '<<_player_2.size()<<"!!!\n";

	return true;
}

bool Chessboard::move(const Move_Ptr& move, Chessboard& output) const
{
	Player player;
	std::map<Position, FieldValue> *player_fields, *opponent_fields;
	Position start_pos = move->getStartPos(), final_pos = move->getFinalPos();
	FieldValue start_val = getFieldValue(start_pos), final_val =  getFieldValue(final_pos);

	output.clear();
	output = *this;

	if(start_val == EMPTY)
		return false;
	
	if(checkPlayer(start_val, PLAYER_1))
	{
		player = PLAYER_1;
		player_fields = &output._player_1;
		opponent_fields = &output._player_2;
	}
	else
	{
		player = PLAYER_2;
		player_fields = &output._player_2;
		opponent_fields = &output._player_1;
	}

	auto next_pos = move->getNextPos();
	auto taking_pos = move->getTakingPos();

	// TODO: sprawdzenie poprawno�ci
	auto end_it = taking_pos.end();
	for(auto it=taking_pos.begin(); it!=end_it;)
	{
		auto it2 = it;
		++it;
		opponent_fields->erase(opponent_fields->find(*it2));
	}

	player_fields->erase(player_fields->find(start_pos));
	if(checkEndLine(player, final_pos))
		player_fields->insert(std::make_pair(final_pos, createKing(start_val)));
	else
		player_fields->insert(std::make_pair(final_pos, start_val));

	return true;
}

bool Chessboard::win(Player player) const
{
	if(player == PLAYER_1)
	{
		if(_player_2.empty())
			return true;
		return false;
	}
	else
	{
		if(_player_1.empty())
			return true;
		return false;
	}
}

bool Chessboard::win() const
{
	if(_player_1.empty() || _player_2.empty())
			return true;
		return false;
}

bool Chessboard::draw() const
{
	if(!_player_1.empty() && !_player_2.empty())
	{
		std::vector<Move_Ptr> moves;
		findMoves(PLAYER_1, moves);
		if(moves.empty())
			return true;
		moves.clear();
		findMoves(PLAYER_2, moves);
		if(moves.empty())
			return true;
	}
	return false;
}

void Chessboard::getCheckers(std::vector<Position>& pawns_1, std::vector<Position>& pawns_2, 
		std::vector<Position>& kings_1, std::vector<Position>& kings_2) const
{
	//std::cout<<"I WANT TO GET! :\t"<<_player_1.size()<<' '<<_player_2.size()<<"!!!\n";
	//std::cout<<_player_1.size()<<' '<<_player_2.size()<<"!!!\n";
	auto end_it = _player_1.end();
	for(auto it=_player_1.begin(); it!=end_it; ++it)
	{
		if(it->second == PAWN_1)
			pawns_1.push_back(it->first);
		else
			kings_1.push_back(it->first);
	}

	end_it = _player_2.end();
	for(auto it=_player_2.begin(); it!=end_it; ++it)
	{
		if(it->second == PAWN_2)
			pawns_2.push_back(it->first);
		else
			kings_2.push_back(it->first);
	}
	//std::cout<<"1";
}

std::ostream& operator<< (std::ostream &os, const Chessboard& board)
{
	Position pos;
	os<<"    __________"<<std::endl;
	for(int y=board.CHESSBOARD_SIZE-1; y >= 0; --y)
	{
		os<<" "<<y<<" | ";
		for(int x=0; x < board.CHESSBOARD_SIZE; ++x)
		{
			Position pos(x,y);
			if(Chessboard::getFieldColor(pos) == WHITE)
				os<<' ';
			else
			{
				switch(board.getFieldValue(pos))
				{
				case EMPTY:
					os<<'X';
					break;
				case PAWN_1:
					os<<'1';
					break;
				case PAWN_2:
					os<<'2';
					break;
				case KING_1:
					os<<'!';
					break;
				case KING_2:
					os<<'@';
					break;
				default:
					os<<'?';
					break;
				}
			}
		}
		os<<" |"<<std::endl;
	}
	os<<"    __________"<<std::endl;
	os<<"     01234567 "<<std::endl;
	return os;
	/*
	Position pos;
	for(int y=board.CHESSBOARD_SIZE-1; y >= 0; --y)
	{
		for(int x=0; x < board.CHESSBOARD_SIZE; ++x)
		{
			Position pos(x,y);
			if(Chessboard::getFieldColor(pos) == WHITE)
				os<<'w';
			else
			{
				switch(board.getFieldValue(pos))
				{
				case EMPTY:
					os<<'b';
					break;
				case PAWN_1:
					os<<'1';
					break;
				case PAWN_2:
					os<<'2';
					break;
				case KING_1:
					os<<'!';
					break;
				case KING_2:
					os<<'@';
					break;
				default:
					os<<'?';
					break;
				}
			}
		}
		os<<std::endl;
	}
	return os;
	*/
}

void Chessboard::_findMove(Player player, const Position& curr_pos, Direction dir, const bool limit, 
	std::vector<Move_Ptr>& taking_moves, std::vector<Move_Ptr>& not_taking_moves) const
{
	Position next_pos;
	FieldValue next_val, curr_val = getFieldValue(curr_pos);

	for(int steps=1; ; ++steps)
	{
		next_pos = findPosition(player, curr_pos, dir, steps);
		if(!next_pos.valid())
			break;
		next_val = getFieldValue(next_pos);
		
		if(next_val == EMPTY)
			not_taking_moves.push_back(Move_Ptr(new NotTakingMove(curr_pos, next_pos)));
		else if(!checkPlayer(curr_val, next_val))	// taking move
		{
			//std::cout<<"Taking";
			for(int steps_taking = steps+1; ; ++steps_taking)
			{
				Position after_next_pos = findPosition(player, curr_pos, dir, steps_taking);
				if(after_next_pos.valid() && getFieldValue(after_next_pos) == EMPTY)
				{
					TakingMove move(curr_pos);
					move.addMove(after_next_pos, next_pos);
					_findTakingMoveContinue(player, move, !dir, limit, taking_moves);
				}
				else
					break;
				if(limit)	// PAWN move
					break;
			}
			break;
		}
		if(limit)	// PAWN move
			break;
	}
}

void Chessboard::_findTakingMove(Player player, const Position& curr_pos, const Direction dir, const bool limit, 
	std::vector<Move_Ptr>& moves) const
{
	// TODO
	//std::cout<<"*** CURRENT MOVE:\t"<<curr_move<<std::endl;
	bool move_continue = false;
	
	Position taking_pos, next_pos;
	FieldValue taking_val, next_val;
	TakingMove curr_move(curr_pos);

	for(int i=1; ; ++i)
	{
		// find piece to take
		taking_pos = findPosition(player, curr_pos, (Direction)dir, i);
		if(!taking_pos.valid())
			break;
		taking_val = getFieldValue(taking_pos);
		if(taking_val == EMPTY)
		{
			if(limit)
				break;
			continue;
		}
		else if(checkPlayer(taking_val, player))
			break;
		//std::cout<<"PIECE TO TAKE:\t"<<taking_pos<<std::endl;
		for(int j=1; ; ++j)	// opponent on taking_pos
		{
			next_pos = findPosition(player, taking_pos, (Direction)dir, j);
			if(!next_pos.valid())
				break;	// break twice (to change direction)
			next_val = getFieldValue(next_pos);
			if(getFieldValue(next_pos) == EMPTY)
			{
				//std::cout<<"PLACE NEXT:\t"<<next_pos<<std::endl;
				curr_move.addMove(next_pos, taking_pos);
				move_continue = true;
				_findTakingMoveContinue(player, curr_move, !(Direction)dir, limit, moves);
				curr_move.removeLastMove();
				if(limit)
					break;
			}
			else
				break;
		}
		break;
	}

	if(!move_continue && curr_move.countTakingPieces() > 0)
		moves.push_back(Move_Ptr(curr_move.clone()));
}

void Chessboard::_findTakingMoveContinue(Player player, TakingMove curr_move, const Direction from_dir, 
	const bool limit, std::vector<Move_Ptr>& moves) const
{
	//std::cout<<"*** CURRENT MOVE:\t"<<curr_move<<std::endl;
	bool move_continue = false;
	
	Position taking_pos, next_pos, curr_pos = curr_move.getFinalPos();
	FieldValue taking_val, next_val;

	for(int dir=Direction::UP_LEFT; dir<=Direction::DOWN_RIGHT; ++dir)
	{
		//std::cout<<dir<<std::endl;
		if(dir != from_dir)
		{
			for(int i=1; ; ++i)
			{
				// find piece to take
				taking_pos = findPosition(player, curr_pos, (Direction)dir, i);
				if(!taking_pos.valid())
					break;
				taking_val = getFieldValue(taking_pos);
				if(taking_val == EMPTY)
				{
					if(limit)
						break;
					continue;
				}
				else if(checkPlayer(taking_val, player))
					break;
				//std::cout<<"PIECE TO TAKE:\t"<<taking_pos<<std::endl;
				for(int j=1; ; ++j)	// opponent on taking_pos
				{
					next_pos = findPosition(player, taking_pos, (Direction)dir, j);
					if(!next_pos.valid())
						break;	// break twice (to change direction)
					next_val = getFieldValue(next_pos);
					if(getFieldValue(next_pos) == EMPTY)
					{
						//std::cout<<"PLACE NEXT:\t"<<next_pos<<std::endl;
						curr_move.addMove(next_pos, taking_pos);
						move_continue = true;
						_findTakingMoveContinue(player, curr_move, !(Direction)dir, limit, moves);
						curr_move.removeLastMove();
						if(limit)
							break;
					}
					else
						break;
				}
				break;
			}
		}
	}
	if(!move_continue)
		moves.push_back(Move_Ptr(curr_move.clone()));
	//std::cout<<"Wychodze";
}

}	// Checkers
