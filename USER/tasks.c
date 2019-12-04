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
    uart_init(115200);
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
    MOTOR_init(&SCA[0], MOTOR_POSITION_MODE);

    MOTOR_set_vel_loop_gain(&SCA[0], 1.0f, 0.0f);
    MOTOR_set_vel_loop_limit(&SCA[0], 0.0f, 0.0f);
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
        printf("DEBUG1:CODE:%d\r\n", GetActrPara(ACTR_CMD_GET_POSTION, devIDList[i]));
        printf("DEBUG2:CODE:%d\r\n", GetActrPara(ACTR_CMD_GET_SPEED, devIDList[i]));

        MOTOR_set_fbk(&SCA[i], 0.0f, pActrParaDev->actrSpeed, pActrParaDev->actrPostion);

        MOTOR_calc(&SCA[i]);

        SetActrCurrent(MOTOR_get_cmd(&SCA[i]) / 33.0f, devIDList[i]);
        printf("DEBUG3:CUR:%.3fA\r\n", MOTOR_get_cmd(&SCA[i]));
        delay_ms(1);
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
