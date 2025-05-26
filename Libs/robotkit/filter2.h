#ifndef FILTER2_H
#define FILTER2_H

typedef struct {
    float pred_encoder;
    float gyro_freq;
    float k;
    char no_correction;
} filter2_t;

void filter2_init(filter2_t *f, float k, float freq);
void filter2_predict(filter2_t *f, float gyro1, float gyro2);
void filter2_update(filter2_t *f, float encoder);

#endif