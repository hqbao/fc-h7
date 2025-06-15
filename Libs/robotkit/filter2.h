#ifndef FILTER2_H
#define FILTER2_H

typedef struct {
    float pred_encoder;
    float gyro_freq;
    float k;
    char no_correction;
} filter2_t;

void filter2_init(filter2_t *f, float k, float freq);
void filter2_update_gyro1(filter2_t *f, float gyro);
void filter2_update_gyro2(filter2_t *f, float gyro);
void filter2_update_encoder(filter2_t *f, float encoder);

#endif