#ifndef QUAT_H
#define QUAT_H

#include "vector3d.h"
#include "matrix.h"

typedef struct {
    double w, x, y, z;
} quaternion_t;

typedef struct {
    double roll, pitch, yaw;
} euler_angle_t;

// Initialize quaternion
void quat_init(quaternion_t* q, double w, double x, double y, double z);

// Set quaternion
void quat_set(quaternion_t* q, const quaternion_t* q_new);

// Add quaternion
void quat_add(quaternion_t* result, const quaternion_t* a, const quaternion_t* b);

// Create identity quaternion
void quat_identity(quaternion_t* q);

// Normalize quaternion
void quat_normalize(quaternion_t* result, const quaternion_t* q);

// Quaternion multiplication (result = a * b)
void quat_mult(quaternion_t* result, const quaternion_t* a, const quaternion_t* b);

// Quaternion conjugate
void quat_conjugate(quaternion_t* result, const quaternion_t* q);

// Create quaternion from axis-angle
void quat_from_axis_angle(quaternion_t* q, const vector3d_t* axis, double angle);

// Quaternion from rotations
void quat_from_euler(quaternion_t *result, double roll, double pitch, double yaw);

// Convert quaternion to Euler angles (ZYX order) and store result in output parameter
void quat_to_euler(euler_angle_t* euler, const quaternion_t* q);

// Convert quaternion to rotation matrix (3x3)
void quat_to_rot_matrix(matrix_t* rot, const quaternion_t* q);

// Rotate vector by quaternion (v' = q * v * q^-1)
void quat_rotate_vector(vector3d_t* result, const quaternion_t* q, const vector3d_t* v);

// Quaternion exponential map (for small angles)
void quat_exp_map(quaternion_t* result, const vector3d_t* w, double dt);

// Measurement function h(q): Predicts accelerometer reading from quaternion state
void quat_to_accel(vector3d_t *a, const quaternion_t *q, double g);

#endif
