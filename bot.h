#ifndef BOT_H
#define BOT_H

float* get_position(void);
float* get_destination(void);
void bot_start(void);

typedef enum {X_, Y_, T_} index_t;

#endif /* BOT_H */
