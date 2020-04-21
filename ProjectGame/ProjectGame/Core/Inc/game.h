#ifndef GAME_H
#define GAME_H


#define LEVEL_ONE_TIME (10)
#define LEVEL_TWO_TIME (6.66)
#define LEVEL_THREE_TIME (4.44)
#define LEVEL_FOUR_TIME (2.96)
#define LEVEL_FIVE_TIME (1.97)
#define LEVEL_SIX_TIME (1.31)

#define X_SCALING_FACTOR (1)
#define Y_SCALING_FACTOR (1)

#define X_MIN_CLIP   (-650)
#define X_MAX_CLIP   (650)
#define BOARD_WIDTH  (20)

#define BOARD_HEIGHT (40)

#define DEFAULT_X_SUB_VAL (-7)
#define DEFAULT_Y_SUB_VAL (-303)

#define X_MIN_RANGE (-650)
#define X_MAX_RANGE (650)
#define Y_MIN_RANGE (-900)
#define Y_MAX_RANGE (800)

#define X_TOTAL_MIN_RANGE (-24000000)
#define X_TOTAL_MAX_RANGE (22000000)
#define X_DIVIDING_FACTOR (2300000)

#define Y_TOTAL_MIN_RANGE (-52000000)
#define Y_TOTAL_MAX_RANGE (48000000)
#define Y_DIVIDING_FACTOR (2500000)


#define ROUNDS_PER_LEVEL (5)
#define AMOUNT_OF_LEVELS (6)
typedef struct Total_Movement_S
{
     long long int total_dx;
     long long int total_dy;
     int x_pos;
     int y_pos;

}Total_MovementS;

typedef struct Positions_S
{
    int rand_pos[2];
    int curr_pos[2];
    int b_pos[2];
    int m_pos[2];
    int s_pos[2];
} PositionsS;

void Init_Board(char **board);
void update_tail_positions(PositionsS *pos, int calced_x, int calced_y);
int random_limit (unsigned int rand,unsigned int lim);
#endif
