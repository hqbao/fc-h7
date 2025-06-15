#ifndef PID_CONTROL_H
#define PID_CONTROL_H

typedef struct {
  float pid_freq;

  float value;
  float prev_value;
  float p_value;
  float error;

  float smooth1;
  float smooth2;
  float smooth3;

  float p_gain;
  float i_gain;
  float i_gain_accum;
  float d_gain;

  float p_limit;
  float i_limit;
  float d_limit;
  float o_limit;

  char halt_i;

  float p_term;
  float i_term;
  float d_term;

  float output;
} pid_control_t;

void pid_control_init(pid_control_t *pid_control, float pid_freq);
void pid_control_reset(pid_control_t *pid_control, float init_value);
void pid_control_set_smooth(pid_control_t *pid_control, float smooth1, float smooth2, float smooth3);
void pid_control_set_p_gain(pid_control_t *pid_control, float p_gain);
void pid_control_set_i_gain(pid_control_t *pid_control, float i_gain, float i_gain_accum);
void pid_control_set_d_gain(pid_control_t *pid_control, float d_gain);
void pid_control_set_p_limit(pid_control_t *pid_control, float abs_value);
void pid_control_set_i_limit(pid_control_t *pid_control, float abs_value);
void pid_control_set_d_limit(pid_control_t *pid_control, float abs_value);
void pid_control_set_o_limit(pid_control_t *pid_control, float abs_value);
void pid_control_halt_i(pid_control_t *pid_control, char halt_i);
float pid_control_update(pid_control_t *pid_control, float value, float target);

#endif
