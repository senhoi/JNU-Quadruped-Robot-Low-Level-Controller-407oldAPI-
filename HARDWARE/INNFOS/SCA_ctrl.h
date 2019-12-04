/*
******************************************************************************************************
*                         INNFOS SCA Controller Ref Design
*
*   Copyright 2018 - 2022 INNFOS (Beijing) Technology Co., Ltd.
*   www.innfos.com
*   All rights reserved.
*	ģ������ : SCA����ģ��ͷ�ļ���
*	�ļ����� : SCA_ctrl.h
*	��    �� : V1.0
*	˵    �� : 
*	�޸ļ�¼ :
******************************************************************************************************
*/

#ifndef _SCA_CTRL_H_
#define _SCA_CTRL_H_

#include "stdint.h"
#include "sys.h"
#include "delay.h"

#define CAN_BUSY_TIMEOUT 100000
#define CAN_WAIT_RECV_TIMEOUT 20
#define ACTR_DEV_NUM 1

#define CAN_BUS_STATE_FREE 0
#define CAN_BUS_STATE_RESET -1

#define ACTR_SET_MODE_SUCCESS 0
#define ACTR_SET_MODE_FIND_DEV_FAIL -1
#define ACTR_SET_MODE_SEND_FAIL -2
#define ACTR_SET_MODE_ACK_FAIL -3

#define CAN_RECV_UPDATE_SET 1
#define CAN_RECV_UPDATE_RESET 0

#define ACTR_CMD_SHAKE_HAND 0x00
#define ACTR_CMD_GET_CURRENT 0x04
#define ACTR_CMD_GET_SPEED 0x05
#define ACTR_CMD_GET_POSTION 0x06
#define ACTR_CMD_SET_MODE 0x07
#define ACTR_CMD_SET_CURRENT 0x08
#define ACTR_CMD_SET_SPEED 0x09
#define ACTR_CMD_SET_POSTION 0x0A
#define ACTR_CMD_SET_ON_OFF 0x2A
#define ACTR_CMD_GET_ON_OFF 0x2B
#define ACTR_CMD_SET_CURRENT_OUTPUT_LOWER_LIMIT 0x2E
#define ACTR_CMD_SET_CURRENT_OUTPUT_UPPER_LIMIT 0x2F
#define ACTR_CMD_SET_SPEED_OUTPUT_LOWER_LIMIT 0x30
#define ACTR_CMD_SET_SPEED_OUTPUT_UPPER_LIMIT 0x31
#define ACTR_CMD_SET_POSTION_OUTPUT_LOWER_LIMIT 0x32
#define ACTR_CMD_SET_POSTION_OUTPUT_UPPER_LIMIT 0x33
#define ACTR_CMD_GET_MOTOR_TEMP 0x5F
#define ACTR_CMD_GET_INVTR_TEMP 0x60
#define ACTR_CMD_GET_CUR_MODE 0x55
#define ACTR_CMD_GET_EXECPTION 0xFF

#define ACTR_CMD_GET_TSHAP_POS_MAX_SPEED 0x1C
#define ACTR_CMD_GET_TSHAP_POS_ACCELERATE 0x1D
#define ACTR_CMD_GET_TSHAP_POS_DECELERATE 0x1E
#define ACTR_CMD_GET_CURRENT_OUTPUT_LOWER_LIMIT 0x34
#define ACTR_CMD_GET_CURRENT_OUTPUT_UPPER_LIMIT 0x35
#define ACTR_CMD_GET_SPEED_OUTPUT_LOWER_LIMIT 0x36
#define ACTR_CMD_GET_SPEED_OUTPUT_UPPER_LIMIT 0x37
#define ACTR_CMD_GET_POSTION_OUTPUT_LOWER_LIMIT 0x38
#define ACTR_CMD_GET_POSTION_OUTPUT_UPPER_LIMIT 0x39
#define ACTR_CMD_GET_POSTION_LOWER_LIMIT 0x85
#define ACTR_CMD_GET_POSTION_UPPER_LIMIT 0x86
#define ACTR_CMD_GET_SHUTDOWN_STATE 0xB0

#define CAN_FRAME_BIT_CMD 0
#define CAN_FRAME_BIT_DAT_HH 1
#define CAN_FRAME_BIT_DAT_HL 2
#define CAN_FRAME_BIT_DAT_LH 3
#define CAN_FRAME_BIT_DAT_LL 4

#define CAN_FRAME_ANALYS_SUCCESS 0
#define CAN_FRAME_SET_MOD_FAIL -1
#define CAN_FRAME_GET_MOD_FAIL -2
#define CAN_FRAME_DAT_LEN_ERR -3

#define CAN_FRAME_ACK_SUCCESS 1
#define CAN_FRAME_ACK_FAIL 0
#define CAN_FRAME_ACK_CLEAR 0

#define SET_PARA_SUCCESS 0
#define SET_PARA_ERR_FIND_DEV -1
#define SET_PARA_ERR_OUT_RANGE -2
#define SET_PARA_ERR_CAN_T_ERR -3
#define SET_PARA_ERR_ACK_ERR -4

#define GET_PARA_SUCCESS 0
#define GET_PARA_ERR_FIND_DEV -1
#define GET_PARA_ERR_OUT_RANGE -2
#define GET_PARA_ERR_CAN_T_ERR -3
#define GET_PARA_ERR_ACK_ERR -4

#define ACTR_STATE_ON_LINE 1
#define ACTR_STATE_OFF_LINE 0
#define ACTR_OFF_LINE_LIMIT 10

typedef enum ActrRunModeTypedef
{
    ACTR_MODE_CUR = 1,   //����ģʽ
    ACTR_MODE_SPD,       //�ٶ�ģʽ
    ACTR_MODE_POS,       //λ��ģʽ
    ACTR_MODE_TECH,      //ʾ��ģʽ
    ACTR_MODE_PLAY,      //����ģʽ
    ACTR_MODE_TSHAP_POS, //profileλ��ģʽ
    ACTR_MODE_TSHAP_SPD, //profile�ٶ�ģʽ
    ACTR_MODE_HOMING     //homingģʽ
} ActrRunModeTypedef;

typedef enum ActrPwrStateTypedef
{
    PWR_OFF,
    PWR_ON
} ActrPwrStateTypedef;

typedef enum ActrShurdownStateTypedef
{
    SHUTDOWN_FALSE,
    SHUTDOWN_TURE
} ActrShutdownStateTypedef;

typedef struct ActrWarnStateBitTypedef
{
    uint16_t WARN_BIT_OVER_VOLT : 1;   //��ѹ�쳣
    uint16_t WARN_BIT_UNDER_VOLT : 1;  //Ƿѹ�쳣
    uint16_t WARN_BIT_LOCK_ROTOR : 1;  //��ת�쳣
    uint16_t WARN_BIT_OVER_TEMP : 1;   //�����쳣
    uint16_t WARN_BIT_RW_PARA : 1;     //��д�����쳣
    uint16_t WARN_BIT_MUL_CIRCLE : 1;  //��Ȧ�����쳣
    uint16_t WARN_BIT_TEMP_SENSOR : 1; //�¶ȴ������쳣
    uint16_t WARN_BIT_CAN_BUS : 1;     //CANͨѶ�쳣
    uint16_t WARN_BIT_RESERVE_B : 2;   //����λ
    uint16_t WARN_BIT_DRV_PROTEC : 1;  //DRV����
    uint16_t WARN_BIT_RESERVE_C : 5;   //����λ
} ActrWarnStateBitTypedef;

typedef union ActrWarnStateTypedef {
    uint16_t ActrWarnStateAll;
    ActrWarnStateBitTypedef ActrWarnStateBit;
} ActrWarnStateTypedef;

typedef struct ActrParaTypedef
{
    uint8_t actrID;                             //ִ����ID
    uint8_t actrParaUpdFlag;                    //�������±�־
    uint8_t actrOnlineState;                    //ִ��������״̬
    uint8_t actrRecvACKState;                   //���յ�ִ����Ӧ��״̬
    uint8_t actrOfflineCounter;                 //ִ��������״̬������
    ActrRunModeTypedef actrMode;                //ִ������ǰ����ģʽ
    ActrPwrStateTypedef actrPwrState;           //ִ�������ػ�״̬
    ActrWarnStateTypedef actrWarnState;         //ִ��������״̬������ִ���������쳣����
    ActrShutdownStateTypedef actrShutdownState; //ִ�����ϴιػ�״̬
    float actrCurrent;                          //ִ������ǰ�ĵ���
    float actrSpeed;                            //ִ������ǰ���ٶ�
    float actrPostion;                          //ִ������ǰ��λ��
    float actrDestCurrent;                      //ִ����Ŀ�����������ģʽ��Ч
    float actrDestSpeed;                        //ִ����Ŀ���ٶȣ��ٶ�ģʽ��Ч
    float actrDestPostion;                      //ִ����Ŀ��λ�ã�λ��ģʽ��Ч
    float actrMotorTemp;                        //ִ��������¶�
    float actrInvterTemp;                       //ִ����������¶�
    float actrTshapPosMaxSpeed;                 //ִ����λ���������ߵ�����ٶ�
    float actrTshapPosAccelerate;               //ִ����λ���������ߵļ��ٶ�
    float actrTshapPosDecelerate;               //ִ����λ���������ߵļ��ٶ�
    float actrPosOutputUpperLimit;              //ִ����λ�û��������
    float actrPosOutputLowerLimit;              //ִ����λ�û��������
    float actrSpeedOutputUpperLimit;            //ִ�����ٶȻ�������ޣ��޸Ĵ���ɵ���λ�û�ģʽ�µ���նȣ�
    float actrSpeedOutputLowerLimit;            //ִ�����ٶȻ�������ޣ��޸Ĵ���ɵ���λ�û�ģʽ�µ���նȣ�
    float actrCurrentOutputUpperLimit;          //ִ�����������������
    float actrCurrentOutputLowerLimit;          //ִ�����������������
    float actrPosUpperLimit;                    //ִ����λ�û�����
    float actrPosLowerLimit;                    //ִ����λ�û�����
} ActrParaTypedef;

extern uint8_t devIDList[ACTR_DEV_NUM];

int SetActrMode(ActrRunModeTypedef actrMode, uint8_t actrID);
int SetActrPosition(float posSet, uint32_t actrID);
int SetActrSpeed(float speedSet, uint32_t actrID);
int SetActrCurrent(float currentSet, uint32_t actrID);
int SetActrSpeedOutputLowerLimit(float speedSetLowerLimit, uint32_t actrID);
int SetActrSpeedOutputUpperLimit(float speedSetUpperLimit, uint32_t actrID);
int SetActrPwrState(ActrPwrStateTypedef PwrState, uint32_t actrID);
int GetActrPara(uint8_t actrGetParaCmd, uint32_t actrID);
int ActrHandShake(uint32_t actrID);

ActrParaTypedef *FindActrDevByID(uint8_t actrID);
void CanRecvFramAnalyse(CanRxMsg *pCanRxMsg, ActrParaTypedef *pActrParaDev);
int Can1BusyCheck(void);
void ActrDevInit(void);

#endif
