#ifndef _PID_H
#define _PID_H

typedef enum PID_Mode_t
{
	PID_INCREMENT_MODE = 0x00,
	PID_REGULAR_MODE = 0x01,
} PID_Mode_t;

typedef struct PID_t
{
	PID_Mode_t mode;

	float kp;
	float ki;
	float kd;

	float in;
	float fbk;
	float ffd;

	/* Queue, where the new value is inserted at index 0.*/
	float out[2];
	float err[3];

	float max;
	float min;
	float err_sum;

} PID_t;

void PID_init(PID_t *pid, PID_Mode_t mode, float kp, float ki, float kd);

void PID_calc(PID_t *pid);

void PID_set_in(PID_t *pid, float in);

void PID_set_fbk(PID_t *pid, float fbk);

void PID_set_ffd(PID_t *pid, float ffd);

void PID_set_gain(PID_t *pid, float kp, float ki, float kd);

void PID_set_limit(PID_t *pid, float max, float min);

float PID_get_out(PID_t *pid);

void PID_clr_buf(PID_t *pid);

#endif
