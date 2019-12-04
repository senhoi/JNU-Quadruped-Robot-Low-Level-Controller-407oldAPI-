/**
	* @File:	motor.c
	* @Author:	Chunyu Zhang
	* @Version:	V0.0.1
	* @Date:	2019.11.25
	* @Description:
	*
*/

#include "pid.h"

/**
	* @Function:	Initializing the structure member for pid
	* @Parameter:	- *pid:	pointer of pid structure
					- mode: mode of pid controller
					- kp:	gain value of P controller
					- ki:	gain value of I controller
					- kd:	gain value of D controller
	* @Return:		none
	* @Attention:	If ki/kd is set to 0, the corrosponding controller will be disabled.
					kp cannot be set to 0!!!
*/
void PID_init(PID_t* pid, PID_Mode_t mode, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->in = 0;
	pid->fbk = 0;
	pid->ffd = 0;
	pid->out[0] = 0;
	pid->out[1] = 0;
	pid->err[0] = 0;
	pid->err[1] = 0;
	pid->err[2] = 0;

	pid->max = 0.0f;
	pid->min = 0.0f;
	pid->err_sum = 0;
}

/**
	* @Function:	Calculate the output value of the PID controller
	* @Parameter:	- *pid:	pointer of pid structure
	* @Return:		none
	* @Attention:	Either mode will output the actual control rather than the control increment.
					For regular mode, the property of anti integral windup is added.
*/
void PID_calc(PID_t* pid)
{
	pid->err[2] = pid->err[1];
	pid->err[1] = pid->err[0];

	pid->err[0] = pid->in - pid->fbk;

	switch (pid->mode)
	{
	case PID_REGULAR_MODE:
		/* Solving integral windup */
		// FIXME:	The existing of feedfoward may decrease the effect of anti integral windup
		//			because the feedfoward isn't included in the pid->out[1].
		if (pid->out[1] > pid->max)
		{
			if (pid->err[0] < 0)
				pid->err_sum += pid->err[0];
		}	
		else if (pid->out[1] < pid->min)
		{
			if (pid->err[0] > 0)
				pid->err_sum += pid->err[0];
		}
		pid->out[0] = pid->kp * pid->err[0] + pid->ki * pid->err_sum + pid->kd * (pid->err[1] - pid->err[0]);
		break;

	case PID_INCREMENT_MODE:
		pid->out[0] = pid->out[1] + pid->kp * (pid->err[0] - pid->err[1]) + pid->ki * pid->err[0] + pid->kd * (pid->err[0] - 2 * pid->err[1] + pid->err[2]);
		break;

	default:
		break;
	}

	pid->out[1] = pid->out[0];
	/* pid->out[0] is the output calculated by error before this point*/
	/* After that pid->out[0] will puls the feedfoward and be limitted */
	pid->out[0] += pid->ffd;

	/* Limitting the output */
	pid->out[0] = (pid->out[0] > pid->max) ? pid->max : pid->out[0];
	pid->out[0] = (pid->out[0] < pid->min) ? pid->min : pid->out[0];
}

/**
	* @Function:	Setting input value (reference value) for pid controller
	* @Parameter:	- *pid:	pointer of pid structure
					- in:	setpoint
	* @Return:		none
	* @Attention:	none
*/
void PID_set_in(PID_t* pid, float in)
{
	pid->in = in;
}

/**
	* @Function:	Setting feedback value for pid controller
	* @Parameter:	- *pid:	pointer of pid structure
					- fbk:	feedback
	* @Return:		none
	* @Attention:	This feedforward value will minus the input. Which means the feedback 
					should be the same type of value with the input.
*/
void PID_set_fbk(PID_t* pid, float fbk)
{
	pid->fbk = fbk;
}

/**
	* @Function:	Setting feedforward value for pid controller
	* @Parameter:	- *pid:	pointer of pid structure
					- ffd:	feedforward
	* @Return:		none
	* @Attention:	This feedforward value will be add to the final output directly.
					Therefore, the feedforward should be the same type of value with
					the output.
					e.g. If the input of a pid controller of DC motor is position,
					the output should be velocity. And the feedforward is velocity as well.
*/
void PID_set_ffd(PID_t* pid, float ffd)
{
	pid->ffd = ffd;
}

/**
	* @Function:	Setting constant gain value for pid controller
	* @Parameter:	- *pid:	pointer of pid structure
					- kp:	gain value of P controller
					- ki:	gain value of I controller
					- kd:	gain value of D controller
	* @Return:		none
	* @Attention:	If ki/kd is set to 0, the corrosponding controller will be disabled.
					kp cannot be set to 0!!!
*/
void PID_set_gain(PID_t* pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/**
	* @Function:	Setting limit of the output for pid controller
	* @Parameter:	- *pid:	pointer of pid structure
					- max: maxinum output of pid controller
					- min: mininum output of pid controller
	* @Return:		none
	* @Attention:	If abs(max - min) < 0.01, it will be consider that there is no limit
					for output.
					If max < min, the output is going to stay at 0.
*/
void PID_set_limit(PID_t* pid, float max, float min)
{
	pid->max = max;
	pid->min = min;
}

/**
	* @Function:	Getting output value of the pid controller
	* @Parameter:	- *pid:	pointer of pid structure
	* @Return:		output value of the pid controller
	* @Attention:	none
*/
float PID_get_out(PID_t* pid)
{
	return pid->out[0];
}

/**
	* @Function:	Clear the data buffer but not change the parameters of gain and limits
	* @Parameter:	- *pid:	pointer of pid structure
	* @Return:		none
	* @Attention:	none
*/
void PID_clr_buf(PID_t* pid)
{
	pid->in = 0;
	pid->fbk = 0;
	pid->ffd = 0;
	pid->out[0] = 0;
	pid->out[1] = 0;
	pid->err[0] = 0;
	pid->err[1] = 0;
	pid->err[2] = 0;
	pid->err_sum = 0;
}
