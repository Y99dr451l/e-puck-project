#ifndef BOT_H
#define BOT_H

#define MM_PER_STEP 0.12566f // actual value
#define INV_ROBOT_WIDTH 0.01852f // 1/54, same thing here

float* get_position(void);
float* get_destination(void);
void bot_start(void);

typedef enum {X_, Y_, T_, Tm, D_} index_t;

#endif /* BOT_H */
