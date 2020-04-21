#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "game.h"


void Init_Board(char **board)
{
    char x = "x";
    char init[BOARD_WIDTH+1] = "xxxxxxxxxxxxxxxxxxxx";
    for (int i = 0; i <BOARD_HEIGHT;i++)
    {
        strcpy(&board[i],init);
    }
    
}
/**
 * Updates the positions of the previous positions based on the current position
 */
void update_tail_positions(PositionsS *pos, int calced_x, int calced_y)
{
	pos->s_pos[0] = pos->m_pos[0];
	pos->s_pos[1] = pos->m_pos[1];
	pos->m_pos[0] = pos->b_pos[0];
	pos->m_pos[1] = pos->b_pos[1];
	pos->b_pos[0] = pos->curr_pos[0];
	pos->b_pos[1] = pos->curr_pos[1];
	pos->curr_pos[0] = calced_x;
	pos->curr_pos[1] = calced_y;

}

/**
 * Takes a random number and limiter,  modulos the random number by the limiter and adds 1
 */
int random_limit (unsigned int rand,unsigned int lim)
{
	return (rand % lim) + 1;
}
