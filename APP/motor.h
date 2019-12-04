#ifndef _MOTOR_H
#define _MOTOR_H

#include "pid.h"

typedef enum MOTOR_Mode_t
{
	MOTOR_UNKNOWN_MODE = 0x00,

	MOTOR_CURRENT_MODE = 0x01,
	MOTOR_VELOCITY_MODE = 0x02,
	MOTOR_POSITION_MODE = 0x03,

	MOTOR_POSITION_CURRENT_MODE = 0x04,
	MOTOR_VELOCITY_CURRENT_MODE = 0x05,
	MOTOR_POSITION_VELOCITY_MODE = 0x06,
	MOTOR_POSITION_VELOCITY_CURRENT_MODE = 0x07,

}MOTOR_Mode_t;

typedef struct MOTOR_t
{
	MOTOR_Mode_t mode;

	PID_t pid_cur;
	PID_t pid_vel;
	PID_t pid_pos;

} MOTOR_t;

void MOTOR_init(MOTOR_t* motor, MOTOR_Mode_t mode);

void MOTOR_calc(MOTOR_t* motor);

void MOTOR_set_mode(MOTOR_t* motor, MOTOR_Mode_t mode);

void MOTOR_set_cur_loop_gain(MOTOR_t* motor, float kp, float kd);

void MOTOR_set_vel_loop_gain(MOTOR_t* motor, float kp, float ki);

void MOTOR_set_pos_loop_gain(MOTOR_t* motor, float kp, float ki, float kd);

void MOTOR_set_cur_loop_limit(MOTOR_t* motor, float max, float min);

void MOTOR_set_vel_loop_limit(MOTOR_t* motor, float max, float min);

void MOTOR_set_pos_loop_limit(MOTOR_t* motor, float max, float min);

void MOTOR_set_fbk(MOTOR_t* motor, float cur, float vel, float pos);

int MOTOR_cur_mode(MOTOR_t* motor, float torque);

int MOTOR_vel_mode(MOTOR_t* motor, float velocity);

int MOTOR_pos_mode(MOTOR_t* motor, float position);

int MOTOR_pos_cur_mode(MOTOR_t* motor, float position, float torque);

int MOTOR_vel_cur_mode(MOTOR_t* motor, float velocity, float torque);

int MOTOR_pos_vel_mode(MOTOR_t* motor, float position, float velocity);

int MOTOR_pos_vel_cur_mode(MOTOR_t* motor, float position, float velocity, float torque);

float MOTOR_get_cmd(MOTOR_t* motor);

#endif
