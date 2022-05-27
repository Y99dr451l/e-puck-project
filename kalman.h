#ifndef KALMAN_H
#define KALMAN_H

void kalman_start(void);
void audio_processing(int16_t *data, uint16_t num_samples);
float* get_kalman_pos(void);

typedef enum {B_, L_, F_, R_} mic_index;

#endif
