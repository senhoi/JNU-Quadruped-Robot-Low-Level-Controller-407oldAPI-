/**
	* @File:	motor.c
	* @Author:	Chunyu Zhang
	* @Version:	V0.0.1
	* @Date:	2019.11.25
	* @Description:	
	*		
	*/

#include "motor.h"

/**
	* @Function:	Initializing the structure member of motor
	* @Parameter:	- *motor:	pointer of motor structure
					- mode:		mode of motor
	* @Return:		none
	* @Attention:	Because the close loop of current of motor is usually implemented by the 
					hardware or driver/amplifer, gain of the current loop are set to kp = 1,
					ki = 0, kd = 0, which means the PID controller in this program will not work.
					The current loop here is just for the exception.
*/
void MOTOR_init(MOTOR_t* motor, MOTOR_Mode_t mode)
{
	motor->mode = mode;

	PID_init(&motor->pid_cur, PID_INCREMENT_MODE, 1.0f, 0.0f, 0.0f);
	PID_init(&motor->pid_vel, PID_REGULAR_MODE, 1.0f, 0.0f, 0.0f);
	PID_init(&motor->pid_pos, PID_REGULAR_MODE, 1.0f, 0.0f, 0.0f);
}

/**
	* @Function:	Calculating the control command of motor
	* @Parameter:	- *motor:	pointer of motor structure
	* @Return:		none
	* @Attention:	none
*/
void MOTOR_calc(MOTOR_t* motor)
{
	switch (motor->mode)
	{
	case MOTOR_POSITION_MODE:
	case MOTOR_POSITION_CURRENT_MODE:
	case MOTOR_POSITION_VELOCITY_MODE:
	case MOTOR_POSITION_VELOCITY_CURRENT_MODE:
		PID_calc(&motor->pid_pos);
		PID_set_in(&motor->pid_vel, PID_get_out(&motor->pid_pos));
	case MOTOR_VELOCITY_MODE:
	case MOTOR_VELOCITY_CURRENT_MODE:
		PID_calc(&motor->pid_vel);
		PID_set_in(&motor->pid_cur, PID_get_out(&motor->pid_vel));
	case MOTOR_CURRENT_MODE:
		PID_calc(&motor->pid_cur);
		break;
	default:
		break;
	}
}

/**
	* @Function:	Setting the mode of motor
	* @Parameter:	- *motor:	pointer of motor structure
					- mode:		mode of motor
	* @Return:		none
	* @Attention:	none
*/
void MOTOR_set_mode(MOTOR_t* motor, MOTOR_Mode_t mode)
{
	motor->mode = mode;

	PID_clr_buf(&motor->pid_cur);
	PID_clr_buf(&motor->pid_vel);
	PID_clr_buf(&motor->pid_pos);
}

/**
	* @Function:	Setting the gain for current loop of motor
	* @Parameter:	- *motor:	pointer of motor structure
					- kp:		gain value of P controller
					- kd:		gain value of D controller
	* @Return:		none
	* @Attention:	The controller for controlling the current is a PD controller.
					Usually the current loop is implemented by the driver/amplifer, in this case, 
					don't call this function. Keep kp equal to 1 and the other constants equal to 0.
*/
void MOTOR_set_cur_loop_gain(MOTOR_t* motor, float kp, float kd)
{
	PID_set_gain(&motor->pid_cur, kp, 0.0f, kd);
}

/**
	* @Function:	Setting the gain for velocity loop of motor
	* @Parameter:	- *motor:	pointer of motor structure
					- kp:		gain value of P controller
					- ki:		gain value of I controller
	* @Return:		none
	* @Attention:	The controller for controlling the velocity is a PI controller.
*/
void MOTOR_set_vel_loop_gain(MOTOR_t* motor, float kp, float ki)
{
	PID_set_gain(&motor->pid_vel, kp, ki, 0.0f);
}

/**
	* @Function:	Setting the gain for position loop of motor
	* @Parameter:	- *motor:	pointer of motor structure
					- kp:		gain value of P controller
					- ki:		gain value of I controller
					- kd:		gain value of D controller
	* @Return:		none
	* @Attention:	The controller for controlling the velocity is a PID controller.
*/
void MOTOR_set_pos_loop_gain(MOTOR_t* motor, float kp, float ki, float kd)
{
	PID_set_gain(&motor->pid_pos, kp, ki, kd);
}

/**
	* @Function:	Setting the limit for current loop of motor
	* @Parameter:	- *motor:	pointer of motor structure
					- max:		maxinum output of current
					- min:		mininum output of current
	* @Return:		none
	* @Attention:	If abs(max - min) < 0.01, it will be consider that there is no limit for output.
					If max < min, the output is going to stay at 0.
*/
void MOTOR_set_cur_loop_limit(MOTOR_t* motor, float max, float min)
{
	PID_set_limit(&motor->pid_cur, max, min);
}

/**
	* @Function:	Setting the limit for velocity loop of motor
	* @Parameter:	- *motor:	pointer of motor structure
					- max:		maxinum output of velocity
					- min:		mininum output of velocity
	* @Return:		none
	* @Attention:	If abs(max - min) < 0.01, it will be consider that there is no limit for output.
					If max < min, the output is going to stay at 0.
*/
void MOTOR_set_vel_loop_limit(MOTOR_t* motor, float max, float min)
{
	PID_set_limit(&motor->pid_vel, max, min);
}

/**
	* @Function:	Setting the limit for position loop of motor
	* @Parameter:	- *motor:	pointer of motor structure
					- max:		maxinum output of position
					- min:		mininum output of position
	* @Return:		none
	* @Attention:	If abs(max - min) < 0.01, it will be consider that there is no limit for output.
					If max < min, the output is going to stay at 0.
*/
void MOTOR_set_pos_loop_limit(MOTOR_t* motor, float max, float min)
{
	PID_set_limit(&motor->pid_pos, max, min);
}

/**
	* @Function:	Setting the feedback value for motor controller
	* @Parameter:	- *motor:	pointer of motor structure
					- cur:		current feedback value
					- vel:		velocity feedback value
					- pos:		position feedback value
	* @Return:		none
	* @Attention:	none
*/
void MOTOR_set_fbk(MOTOR_t* motor, float cur, float vel, float pos)
{
	PID_set_fbk(&motor->pid_cur, cur);
	PID_set_fbk(&motor->pid_vel, vel);
	PID_set_fbk(&motor->pid_pos, pos);
}

/**
	* @Function:	Current mode command
	* @Parameter:	- *motor:	pointer of motor structure
					- current:	setvalue of current
	* @Return:		operation status 
					- 0:		operating successfully
					- 1:		setting mode of motor does not match
	* @Attention:	none
*/
int MOTOR_cur_mode(MOTOR_t* motor, float current)
{
	if (motor->mode == MOTOR_CURRENT_MODE)
	{
		PID_set_in(&motor->pid_cur, current);
		return 0;
	}
	else
		return -1;
}

/**
	* @Function:	Velocity mode command
	* @Parameter:	- *motor:	pointer of motor structure
					- velocity:	setvalue of velocity
	* @Return:		operation status
					- 0:		operating successfully
					- 1:		setting mode of motor does not match
	* @Attention:	none
*/
int MOTOR_vel_mode(MOTOR_t* motor, float velocity)
{
	if (motor->mode == MOTOR_VELOCITY_MODE)
	{
		PID_set_in(&motor->pid_vel, velocity);
		return 0;
	}
	else
		return -1;
}

/**
	* @Function:	Position mode command
	* @Parameter:	- *motor:	pointer of motor structure
					- velocity:	setvalue of position
	* @Return:		operation status
					- 0:		operating successfully
					- 1:		setting mode of motor does not match
	* @Attention:	none
*/
int MOTOR_pos_mode(MOTOR_t* motor, float position)
{
	if (motor->mode == MOTOR_POSITION_MODE)
	{
		PID_set_in(&motor->pid_pos, position);
		return 0;
	}
	else
		return -1;
}

/**
	* @Function:	Position current mode command
	* @Parameter:	- *motor:	pointer of motor structure
					- position:	setvalue of position
					- current:	setvalue of current
	* @Return:		operation status
					- 0:		operating successfully
					- 1:		setting mode of motor does not match
	* @Attention:	none
*/
int MOTOR_pos_cur_mode(MOTOR_t* motor, float position, float current)
{
	if (motor->mode == MOTOR_POSITION_CURRENT_MODE)
	{
		PID_set_in(&motor->pid_pos, position);
		PID_set_ffd(&motor->pid_vel, current);
		return 0;
	}
	else
		return -1;
}

/**
	* @Function:	Velocity current mode command
	* @Parameter:	- *motor:	pointer of motor structure
					- velocity:	setvalue of velocity
					- current:	setvalue of current
	* @Return:		operation status
					- 0:		operating successfully
					- 1:		setting mode of motor does not match
	* @Attention:	none
*/
int MOTOR_vel_cur_mode(MOTOR_t* motor, float velocity, float current)
{
	if (motor->mode == MOTOR_VELOCITY_CURRENT_MODE)
	{
		PID_set_in(&motor->pid_vel, velocity);
		PID_set_ffd(&motor->pid_vel, current);
		return 0;
	}
	else
		return -1;
}

/**
	* @Function:	Position velocity mode command
	* @Parameter:	- *motor:	pointer of motor structure
					- position:	setvalue of position
					- velocity:	setvalue of velocity
	* @Return:		operation status
					- 0:		operating successfully
					- 1:		setting mode of motor does not match
	* @Attention:	none
*/
int MOTOR_pos_vel_mode(MOTOR_t* motor, float position, float velocity)
{
	if (motor->mode == MOTOR_POSITION_VELOCITY_MODE)
	{
		PID_set_in(&motor->pid_pos, position);
		PID_set_ffd(&motor->pid_pos, velocity);
		return 0;
	}
	else
		return -1;
}

/**
	* @Function:	Position velocity current mode command
	* @Parameter:	- *motor:	pointer of motor structure
					- position:	setvalue of position
					- velocity:	setvalue of velocity
					- current:	setvalue of current
	* @Return:		operation status
					- 0:		operating successfully
					- 1:		setting mode of motor does not match
	* @Attention:	none
*/
int MOTOR_pos_vel_cur_mode(MOTOR_t* motor, float position, float velocity, float current)
{
	if (motor->mode == MOTOR_POSITION_VELOCITY_MODE)
	{
		PID_set_in(&motor->pid_pos, position);
		PID_set_ffd(&motor->pid_pos, velocity);
		PID_set_ffd(&motor->pid_vel, current);
		return 0;
	}
	else
		return -1;
}

/**
	* @Function:	Getting output command of motor controller
	* @Parameter:	- *motor:	pointer of motor structure
	* @Return:		operation status
	* @Attention:	motor current command
*/
float MOTOR_get_cmd(MOTOR_t* motor)
{
	return PID_get_out(&motor->pid_cur);
}
