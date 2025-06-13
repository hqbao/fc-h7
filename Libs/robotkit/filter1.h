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
    vector3d_t linear_acceleration;
    double k;
    double accel_scale;
    char no_correction;
} filter1_t;

void filter1_init(filter1_t *f, double k);
void filter1_use_linear_acceleration(filter1_t *f, int accel_scale);
void filter1_predict(filter1_t *f, double gx, double gy, double gz);
void filter1_update(filter1_t *f, double ax, double ay, double az);

#endif