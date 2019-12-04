#include "tasks.h"

MOTOR_t SCA[3];

void init_task(void)
{
    init_task_hardware();
    init_task_controller();
    init_task_innfos();
}

/**
	* @Function:	Initialize the onboard peripherals
	* @Parameter:	none
	* @Return:		none
	* @Attention:	none
*/
void init_task_hardware(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init(168);
    uart_init(256000);
    LED_Init();
    LCD_Init();
    CAN1_Init();
    ActrDevInit();
}

/**
	* @Function:	Initialize low-level controller structure and configure parameters
	* @Parameter:	none
	* @Return:		none
	* @Attention:	none
*/
void init_task_controller(void)
{
    MOTOR_init(&SCA[0], MOTOR_VELOCITY_MODE);

    MOTOR_set_vel_loop_gain(&SCA[0], 0.1f, 0.0f);
    MOTOR_set_vel_loop_limit(&SCA[0], 3.30f, -3.30f);
    MOTOR_set_pos_loop_gain(&SCA[0], 1.0f, 0.0f, 0.0f);
    MOTOR_set_pos_loop_limit(&SCA[0], 0.0f, 0.0f);
}

/**
	* @Function:	Initialize structures of innfos actuator, then enable the actuator
	* @Parameter:	none
	* @Return:		none
	* @Attention:	none
*/
void init_task_innfos(void)
{
    static ActrParaTypedef *pActrParaDev = NULL;

    for (int i = 0; i < ACTR_DEV_NUM; i++)
        pActrParaDev = FindActrDevByID(devIDList[i]);

    printf("Opening SCA.\r\n");

    for (int i = 0; i < ACTR_DEV_NUM; i++)
    {
        GetActrPara(ACTR_CMD_GET_ON_OFF, devIDList[i]);
        if (pActrParaDev->actrPwrState != PWR_ON)
            SetActrPwrState(PWR_ON, devIDList[i]);
    }

    printf("Setting mode of SCA.\r\n");

    for (int i = 0; i < ACTR_DEV_NUM; i++)
    {
        GetActrPara(ACTR_CMD_GET_CUR_MODE, devIDList[i]);
        if (pActrParaDev->actrMode != ACTR_MODE_CUR)
            SetActrMode(ACTR_MODE_CUR, devIDList[i]);
    }

    printf("SCA have been initialized!\r\n");
}

/**
	* @Function:	Get motor data, calculate PID output, send control data
	* @Parameter:	none
	* @Return:		none
	* @Attention:	Because the communication with the motor usually takes a lot of time,
                    so it is better to add the task scheduling function or real-time system 
                    for the MCU.
*/
void ctrl_task(void)
{
    static ActrParaTypedef *pActrParaDev = NULL;

    for (int i = 0; i < ACTR_DEV_NUM; i++)
        pActrParaDev = FindActrDevByID(devIDList[i]);

    for (int i = 0; i < ACTR_DEV_NUM; i++)
    {
        MOTOR_vel_mode(&SCA[i], 100.0f);

        GetActrPara(ACTR_CMD_GET_POSTION, devIDList[i]);
        GetActrPara(ACTR_CMD_GET_SPEED, devIDList[i]);

        MOTOR_set_fbk(&SCA[i], 0.0f, pActrParaDev->actrSpeed * 68.0f * 64.0f, pActrParaDev->actrPostion);

        MOTOR_calc(&SCA[i]);

        SetActrCurrent(MOTOR_get_cmd(&SCA[i]) / 33.0f, devIDList[i]);
        //printf("DEBUG3:CUR:%.3fA\r\n", MOTOR_get_cmd(&SCA[i]));
        //delay_ms(1);

        //printf("CUR-PID:\r\n");
        //printf("K:%.2f %.2f %.2f\r\n", SCA[0].pid_cur.kp, SCA[0].pid_cur.ki, SCA[0].pid_cur.kd);
        //printf("in:%.2f fbk:%.2f ffd:%.2f\r\n", SCA[0].pid_cur.in, SCA[0].pid_cur.fbk, SCA[0].pid_cur.ffd);
        //printf("out:%.2f %.2f\r\n", SCA[0].pid_cur.out[0], SCA[0].pid_cur.out[1]);
        //printf("err:%.2f %.2f %.2f\r\n", SCA[0].pid_cur.err[0], SCA[0].pid_cur.err[1], SCA[0].pid_cur.err[2]);
        //printf("max:%.2f min:%.2f err_sum:%.2f\r\n", SCA[0].pid_cur.max, SCA[0].pid_cur.min, SCA[0].pid_cur.err_sum);
        printf("VEL-PID:\r\n");
        printf("K:%.2f %.2f %.2f\r\n", SCA[0].pid_vel.kp, SCA[0].pid_vel.ki, SCA[0].pid_vel.kd);
        printf("in:%.2f fbk:%.2f ffd:%.2f\r\n", SCA[0].pid_vel.in, SCA[0].pid_vel.fbk, SCA[0].pid_vel.ffd);
        printf("out:%.2f %.2f\r\n", SCA[0].pid_vel.out[0], SCA[0].pid_vel.out[1]);
        //printf("err:%.2f %.2f %.2f\r\n", SCA[0].pid_vel.err[0], SCA[0].pid_vel.err[1], SCA[0].pid_vel.err[2]);
        //printf("max:%.2f min:%.2f err_sum:%.2f\r\n", SCA[0].pid_vel.max, SCA[0].pid_vel.min, SCA[0].pid_vel.err_sum);
        //printf("POS-PID:\r\n");
        //printf("K:%.2f %.2f %.2f\r\n", SCA[0].pid_pos.kp, SCA[0].pid_pos.ki, SCA[0].pid_pos.kd);
        //printf("in:%.2f fbk:%.2f ffd:%.2f\r\n", SCA[0].pid_pos.in, SCA[0].pid_pos.fbk, SCA[0].pid_pos.ffd);
        //printf("out:%.2f %.2f\r\n", SCA[0].pid_pos.out[0], SCA[0].pid_pos.out[1]);
        //printf("err:%.2f %.2f %.2f\r\n", SCA[0].pid_pos.err[0], SCA[0].pid_pos.err[1], SCA[0].pid_pos.err[2]);
        //printf("max:%.2f min:%.2f err_sum:%.2f\r\n", SCA[0].pid_pos.max, SCA[0].pid_pos.min, SCA[0].pid_pos.err_sum);
    }
}

/**
	* @Function:	Background tasks
	* @Parameter:	none
	* @Return:		none
	* @Attention:	Background task is the while loop in main.c
*/
void loop_task(void)
{
    while (1)
    {
        ctrl_task();
    }
}
