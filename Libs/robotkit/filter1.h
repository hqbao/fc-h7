#ifndef FILTER1_H
#define FILTER1_H

#include "vector3d.h"
#include "quat.h"

typedef struct {
    quaternion_t q;
    vector3d_t a;
    vector3d_t v_pred;
    vector3d_t v_true;
    vector3d_t pred_euler_angle;
    vector3d_t true_euler_angle;
    float k;
    char no_correction;
} filter1_t;

void filter1_init(filter1_t *f, float k);
void filter1_predict(filter1_t *f, float gx, float gy, float gz);
void filter1_update(filter1_t *f, float ax, float ay, float az);

#endif