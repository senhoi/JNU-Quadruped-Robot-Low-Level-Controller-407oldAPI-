/*
******************************************************************************************************
*                         INNFOS SCA Controller Ref Design
*
*   Copyright 2018 - 2022 INNFOS (Beijing) Technology Co., Ltd.
*   www.innfos.com
*   All rights reserved.
*	ģ������ : SCA����ģ�顣
*	�ļ����� : SCA_ctrl.c
*	��    �� : V1.0
*	˵    �� : 
*	�޸ļ�¼ :
******************************************************************************************************
*/
#include "stdlib.h"
#include "math.h"
#include "can.h"
#include "SCA_ctrl.h"

static CanTxMsg g_tCanTxMsg;
static CanRxMsg g_tCanRxMsg;
static ActrParaTypedef ActrDevList[ACTR_DEV_NUM]; //ִ�����豸�ṹ�����飬���ڱ���ִ�����Ĳ�����״̬
static const uint32_t IQ24Factor = 16777216;
static const float TempFactor = 256.0f;

uint8_t devIDList[ACTR_DEV_NUM] = {2};

//*********************************************************************************
//��������: int SetActrMode(ActrRunModeTypedef actrMode,uint8_t actrID)
//��    ��������ִ�����Ĺ���ģʽ
//��ڲ���: actrMode ��Ҫ���õ�ģʽ �� actrID��ִ������ID
//���ڲ���: ����ִ�н��
//��    ע������ִ�����Ĺ���ģʽ
//Editor��  liuqiuhu
//*********************************************************************************
int SetActrMode(ActrRunModeTypedef actrMode, uint8_t actrID)
{
    uint32_t waitCanAckCounter = 0;
    ActrParaTypedef *pActrPara = NULL;
    CanTxMsg txMsg;
    pActrPara = FindActrDevByID(actrID);
    if (pActrPara == NULL)
    {
        return ACTR_SET_MODE_FIND_DEV_FAIL;
    }
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_Data;
    txMsg.DLC = 0x02;
    txMsg.Data[CAN_FRAME_BIT_CMD] = ACTR_CMD_SET_MODE;
    txMsg.Data[CAN_FRAME_BIT_DAT_HH] = actrMode;
    txMsg.StdId = pActrPara->actrID;
    txMsg.ExtId = 0x00;
    txMsg.RTR = CAN_RTR_Data;
    txMsg.IDE = CAN_ID_STD;

    pActrPara->actrParaUpdFlag = CAN_RECV_UPDATE_RESET;
    pActrPara->actrRecvACKState = CAN_FRAME_ACK_CLEAR;
    if (Can1BusyCheck() == CAN_BUS_STATE_FREE)
    {
        CAN_Transmit(CAN1, &txMsg);
    }
    else
    {
        return ACTR_SET_MODE_SEND_FAIL;
    }

    while (pActrPara->actrParaUpdFlag != CAN_RECV_UPDATE_SET)
    {
        waitCanAckCounter++;
        delay_us(10);
        if (waitCanAckCounter > CAN_WAIT_RECV_TIMEOUT)
        {
            return ACTR_SET_MODE_ACK_FAIL;
        }
    }
    CanRecvFramAnalyse(&g_tCanRxMsg, ActrDevList);
    if (pActrPara->actrRecvACKState == CAN_FRAME_ACK_SUCCESS)
    {
        if (pActrPara->actrMode == actrMode)
        {
            return ACTR_SET_MODE_SUCCESS;
        }
    }
    return ACTR_SET_MODE_ACK_FAIL;
}

//*********************************************************************************
//��������: SetActrPosition
//��    ��������ִ������λ��
//��ڲ���: posSet  actrID
//���ڲ���: ����ִ�н��
//��    ע��λ�õķ�Χ��-127.0f~128.0f����λ��Ȧ
//Editor��  liuqiuhu
//*********************************************************************************
int SetActrPosition(float posSet, uint32_t actrID)
{
    uint32_t i;
    int32_t tmpPos;
    ActrParaTypedef *pActrPara = NULL;
    pActrPara = FindActrDevByID(actrID);
    if (pActrPara == NULL)
    {
        return SET_PARA_ERR_FIND_DEV;
    }
    if ((posSet < -127.0f) || (posSet >= 128.0f))
    {
        return SET_PARA_ERR_OUT_RANGE;
    }
    pActrPara->actrDestPostion = posSet;

    g_tCanTxMsg.DLC = 0x05;

    g_tCanTxMsg.Data[0] = ACTR_CMD_SET_POSTION;

    g_tCanTxMsg.StdId = actrID;

    tmpPos = (int32_t)(posSet * IQ24Factor);

    for (i = 0; i < 4; i++)
    {
        g_tCanTxMsg.Data[i + CAN_FRAME_BIT_DAT_HH] = (tmpPos >> (8 * (3 - i)));
    }

    if (Can1BusyCheck() == CAN_BUS_STATE_FREE)
    {

        CAN_Transmit(CAN1, &g_tCanTxMsg);
    }
    else
    {
        return SET_PARA_ERR_CAN_T_ERR;
    }

    return SET_PARA_SUCCESS;
}

//*********************************************************************************
//��������: SetActrSpeed
//��    ��������ִ�������ٶ�
//��ڲ���: speedSet  actrID
//���ڲ���: ����ִ�н��
//��    ע���ٶȵķ�Χ��-1.0f~1.0f������ֵ����Ӧ�ٶȵĸ���������������ֵ��ʵ��ת�� = ���ת�� X ����ֵ
//Editor��  liuqiuhu
//*********************************************************************************
int SetActrSpeed(float speedSet, uint32_t actrID)
{
    uint32_t i;
    int32_t tmpSpd;
    ActrParaTypedef *pActrPara = NULL;
    pActrPara = FindActrDevByID(actrID);
    if (pActrPara == NULL)
    {
        return SET_PARA_ERR_FIND_DEV;
    }
    if ((speedSet < -1.0f) || (speedSet > 1.0f))
    {
        return SET_PARA_ERR_OUT_RANGE;
    }
    pActrPara->actrDestSpeed = speedSet;

    g_tCanTxMsg.DLC = 0x05;

    g_tCanTxMsg.Data[CAN_FRAME_BIT_CMD] = ACTR_CMD_SET_SPEED;

    g_tCanTxMsg.StdId = actrID;

    tmpSpd = (int32_t)(speedSet * IQ24Factor);

    for (i = 0; i < 4; i++)
    {
        g_tCanTxMsg.Data[i + CAN_FRAME_BIT_DAT_HH] = (tmpSpd >> (8 * (3 - i)));
    }

    if (Can1BusyCheck() == CAN_BUS_STATE_FREE)
    {

        CAN_Transmit(CAN1, &g_tCanTxMsg);
    }
    else
    {
        return SET_PARA_ERR_CAN_T_ERR;
    }

    return SET_PARA_SUCCESS;
}

//*********************************************************************************
//��������: SetActrCurrent
//��    ��������ִ�����ĵ���
//��ڲ���: currentSet  actrID
//���ڲ���: ����ִ�н��
//��    ע���ٶȵķ�Χ��-1.0f~1.0f������ֵ��ʵ�ʵ��� = ������ X ����ֵ
//Editor��  liuqiuhu
//*********************************************************************************
int SetActrCurrent(float currentSet, uint32_t actrID)
{
    uint32_t i;
    int32_t tmpCur;
    ActrParaTypedef *pActrPara = NULL;
    pActrPara = FindActrDevByID(actrID);
    if (pActrPara == NULL)
    {
        return SET_PARA_ERR_FIND_DEV;
    }
    if ((currentSet < -1.0f) || (currentSet > 1.0f))
    {
        return SET_PARA_ERR_OUT_RANGE;
    }
    pActrPara->actrDestCurrent = currentSet;

    g_tCanTxMsg.DLC = 0x05;

    g_tCanTxMsg.Data[CAN_FRAME_BIT_CMD] = ACTR_CMD_SET_CURRENT;

    g_tCanTxMsg.StdId = actrID;

    tmpCur = (int32_t)(currentSet * IQ24Factor);

    for (i = 0; i < 4; i++)
    {
        g_tCanTxMsg.Data[i + CAN_FRAME_BIT_DAT_HH] = (tmpCur >> (8 * (3 - i)));
    }

    if (Can1BusyCheck() == CAN_BUS_STATE_FREE)
    {

        CAN_Transmit(CAN1, &g_tCanTxMsg);
    }
    else
    {
        return SET_PARA_ERR_CAN_T_ERR;
    }

    return SET_PARA_SUCCESS;
}

//*********************************************************************************
//��������: SetActrSpeedOutputLowerLimit
//��    ��������ִ�������ٶ��������
//��ڲ���: speedSet  actrID
//���ڲ���: ����ִ�н��
//��    ע��
//Editor��
//*********************************************************************************
int SetActrSpeedOutputLowerLimit(float speedSetLowerLimit, uint32_t actrID)
{
    uint32_t i;
    int32_t tmpSpd;
    ActrParaTypedef *pActrPara = NULL;
    pActrPara = FindActrDevByID(actrID);
    if (pActrPara == NULL)
    {
        return SET_PARA_ERR_FIND_DEV;
    }
    if ((speedSetLowerLimit < -1.0f) || (speedSetLowerLimit > 1.0f))
    {
        return SET_PARA_ERR_OUT_RANGE;
    }

    g_tCanTxMsg.DLC = 0x05;

    g_tCanTxMsg.Data[CAN_FRAME_BIT_CMD] = ACTR_CMD_SET_SPEED_OUTPUT_LOWER_LIMIT;

    g_tCanTxMsg.StdId = actrID;

    tmpSpd = (int32_t)(speedSetLowerLimit * IQ24Factor);

    for (i = 0; i < 4; i++)
    {
        g_tCanTxMsg.Data[i + CAN_FRAME_BIT_DAT_HH] = (tmpSpd >> (8 * (3 - i)));
    }

    if (Can1BusyCheck() == CAN_BUS_STATE_FREE)
    {

        CAN_Transmit(CAN1, &g_tCanTxMsg);
    }
    else
    {
        return SET_PARA_ERR_CAN_T_ERR;
    }

    return SET_PARA_SUCCESS;
}

//*********************************************************************************
//��������: SetActrSpeedOutputUpperLimit
//��    ��������ִ�������ٶ��������
//��ڲ���: speedSet  actrID
//���ڲ���: ����ִ�н��
//��    ע��
//Editor��
//*********************************************************************************
int SetActrSpeedOutputUpperLimit(float speedSetUpperLimit, uint32_t actrID)
{
    uint32_t i;
    int32_t tmpSpd;
    ActrParaTypedef *pActrPara = NULL;
    pActrPara = FindActrDevByID(actrID);
    if (pActrPara == NULL)
    {
        return SET_PARA_ERR_FIND_DEV;
    }
    if ((speedSetUpperLimit < -1.0f) || (speedSetUpperLimit > 1.0f))
    {
        return SET_PARA_ERR_OUT_RANGE;
    }

    g_tCanTxMsg.DLC = 0x05;

    g_tCanTxMsg.Data[CAN_FRAME_BIT_CMD] = ACTR_CMD_SET_SPEED_OUTPUT_UPPER_LIMIT;

    g_tCanTxMsg.StdId = actrID;

    tmpSpd = (int32_t)(speedSetUpperLimit * IQ24Factor);

    for (i = 0; i < 4; i++)
    {
        g_tCanTxMsg.Data[i + CAN_FRAME_BIT_DAT_HH] = (tmpSpd >> (8 * (3 - i)));
    }

    if (Can1BusyCheck() == CAN_BUS_STATE_FREE)
    {

        CAN_Transmit(CAN1, &g_tCanTxMsg);
    }
    else
    {
        return SET_PARA_ERR_CAN_T_ERR;
    }

    return SET_PARA_SUCCESS;
}

//*********************************************************************************
//��������: ActrHandShake
//��    ����ִ��������
//��ڲ���: actrID ִ����ID
//���ڲ���: ����ִ�н��
//��    ע��
//Editor��  liuqiuhu
//*********************************************************************************
int ActrHandShake(uint32_t actrID)
{
    int32_t waitCanAckCounter = 0;
    ActrParaTypedef *pActrPara = NULL;
    pActrPara = FindActrDevByID(actrID);
    if (pActrPara == NULL)
    {
        return SET_PARA_ERR_FIND_DEV;
    }
    g_tCanTxMsg.DLC = 0x01;
    g_tCanTxMsg.Data[CAN_FRAME_BIT_CMD] = ACTR_CMD_SHAKE_HAND;
    g_tCanTxMsg.StdId = actrID;
    pActrPara->actrParaUpdFlag = CAN_RECV_UPDATE_RESET;
    pActrPara->actrRecvACKState = CAN_FRAME_ACK_CLEAR;
    if (Can1BusyCheck() == CAN_BUS_STATE_FREE)
    {

        CAN_Transmit(CAN1, &g_tCanTxMsg);
    }
    else
    {
        return SET_PARA_ERR_CAN_T_ERR;
    }

    while (pActrPara->actrParaUpdFlag != CAN_RECV_UPDATE_SET)
    {
        waitCanAckCounter++;
        delay_us(10);
        if (waitCanAckCounter > CAN_WAIT_RECV_TIMEOUT)
        {
            pActrPara->actrOfflineCounter++;
            if (pActrPara->actrOfflineCounter > ACTR_OFF_LINE_LIMIT)
            {
                pActrPara->actrOnlineState = ACTR_STATE_OFF_LINE;
            }
            return SET_PARA_ERR_ACK_ERR;
        }
    }
    CanRecvFramAnalyse(&g_tCanRxMsg, ActrDevList);
    if (pActrPara->actrRecvACKState == CAN_FRAME_ACK_SUCCESS)
    {
        pActrPara->actrOnlineState = ACTR_STATE_ON_LINE;
        pActrPara->actrOfflineCounter = 0;
        return SET_PARA_SUCCESS;
    }
    else
    {
        pActrPara->actrOfflineCounter++;
        if (pActrPara->actrOfflineCounter > ACTR_OFF_LINE_LIMIT)
        {
            pActrPara->actrOnlineState = ACTR_STATE_OFF_LINE;
        }
    }

    return SET_PARA_ERR_ACK_ERR;
}

//*********************************************************************************
//��������: SetActrPwrState
//��    ��������ִ�����Ŀ���/�ػ�״̬
//��ڲ���: actrID ִ����ID
//���ڲ���: ����ִ�н��
//��    ע��
//Editor��  liuqiuhu
//*********************************************************************************
int SetActrPwrState(ActrPwrStateTypedef PwrState, uint32_t actrID)
{
    int32_t waitCanAckCounter = 0;
    ActrParaTypedef *pActrPara = NULL;
    pActrPara = FindActrDevByID(actrID);
    if (pActrPara == NULL)
    {
        return SET_PARA_ERR_FIND_DEV;
    }
    g_tCanTxMsg.StdId = actrID;
    g_tCanTxMsg.DLC = 0x02;
    g_tCanTxMsg.Data[CAN_FRAME_BIT_CMD] = ACTR_CMD_SET_ON_OFF;
    g_tCanTxMsg.Data[CAN_FRAME_BIT_DAT_HH] = PwrState;
    pActrPara->actrParaUpdFlag = CAN_RECV_UPDATE_RESET;
    pActrPara->actrRecvACKState = CAN_FRAME_ACK_CLEAR;
    if (Can1BusyCheck() == CAN_BUS_STATE_FREE)
    {

        CAN_Transmit(CAN1, &g_tCanTxMsg);
    }
    else
    {
        return SET_PARA_ERR_CAN_T_ERR;
    }

    while (pActrPara->actrParaUpdFlag != CAN_RECV_UPDATE_SET)
    {
        waitCanAckCounter++;
        delay_us(10);
        if (waitCanAckCounter > CAN_WAIT_RECV_TIMEOUT)
        {
            return SET_PARA_ERR_ACK_ERR;
        }
    }
    CanRecvFramAnalyse(&g_tCanRxMsg, ActrDevList);
    if (pActrPara->actrRecvACKState == CAN_FRAME_ACK_SUCCESS)
    {
        pActrPara->actrPwrState = PwrState;
        return SET_PARA_SUCCESS;
    }
    return SET_PARA_ERR_ACK_ERR;
}

//*********************************************************************************
//��������: GetActrPara
//��    ������ȡִ�����Ĳ�����Ϣ�������µĲ���ֵ������ִ���������ṹ����
//��ڲ���: actrID ִ����ID
//���ڲ���: ����ִ�н��
//��    ע��
//Editor��  liuqiuhu
//*********************************************************************************
int GetActrPara(uint8_t actrGetParaCmd, uint32_t actrID)
{
    int32_t waitCanAckCounter = 0;
    ActrParaTypedef *pActrPara = NULL;
    pActrPara = FindActrDevByID(actrID);
    if (pActrPara == NULL)
    {
        return GET_PARA_ERR_FIND_DEV;
    }
    g_tCanTxMsg.StdId = actrID;
    g_tCanTxMsg.DLC = 0x01; //TODO:�ٷ������д˴�Ϊ0x02��Ȼ����������ֻװ����һ�ֽ�����֡
                            //Ϊ0x02ʱ���ڻ�ȡλ���ٶȵ���ģʽ����״̬ʱ����������������ΪSET_PARA_ERR_ACK_ERR
                            //����ȡ�¶ȵȲ���ʱ���ᱨ��
    g_tCanTxMsg.Data[CAN_FRAME_BIT_CMD] = actrGetParaCmd;
    pActrPara->actrParaUpdFlag = CAN_RECV_UPDATE_RESET;
    pActrPara->actrRecvACKState = CAN_FRAME_ACK_CLEAR;
    if (Can1BusyCheck() == CAN_BUS_STATE_FREE)
    {
        CAN_Transmit(CAN1, &g_tCanTxMsg);
    }
    else
    {
        return GET_PARA_ERR_CAN_T_ERR;
    }

    while (pActrPara->actrParaUpdFlag != CAN_RECV_UPDATE_SET)
    {
        waitCanAckCounter++;
        delay_us(10);
        if (waitCanAckCounter > CAN_WAIT_RECV_TIMEOUT)
        {
            return SET_PARA_ERR_ACK_ERR;
        }
    }
    CanRecvFramAnalyse(&g_tCanRxMsg, ActrDevList);
    if (pActrPara->actrRecvACKState == CAN_FRAME_ACK_SUCCESS)
    {
        return GET_PARA_SUCCESS;
    }
    return GET_PARA_ERR_ACK_ERR;
}

//*********************************************************************************
//��������: Can1InterruptHandler
//��    �����ú�������stm32f4xx_it.c�У�CAN1_RX0_IRQHandler������
//��ڲ���: ��
//���ڲ���: ��
//��    ע��
//Editor��  liuqiuhu
//*********************************************************************************
void Can1InterruptHandler(void)
{
    ActrParaTypedef *pActrParaDev = NULL;
    CAN_Receive(CAN1, CAN_FIFO0, &g_tCanRxMsg);
    pActrParaDev = FindActrDevByID(g_tCanRxMsg.StdId);
    if (pActrParaDev != NULL)
    {
        pActrParaDev->actrParaUpdFlag = CAN_RECV_UPDATE_SET;
    }
}

//*********************************************************************************
//��������: FindActrDevByID
//��    ����ͨ��ID����ִ�����豸�ṹ��
//��ڲ���: actrID ��Ҫ��ѯ��ִ����ID
//���ڲ���: ActrParaTypedef* ��Ӧִ����ID�Ľṹ��ָ��
//��    ע��
//Editor��  liuqiuhu
//*********************************************************************************

ActrParaTypedef *FindActrDevByID(uint8_t actrID)
{
    uint32_t i;
    for (i = 0; i < ACTR_DEV_NUM; i++)
    {
        if (ActrDevList[i].actrID == actrID)
        {
            return &ActrDevList[i];
        }
    }
    return NULL;
}

//*********************************************************************************
//��������: Can1BusyCheck()
//��    ����CAN����æµ���
//��ڲ�������
//���ڲ�����CAN����״̬
//��    ע��Editor��Liuqh 2018-01-29    Company: INNFOS
//*********************************************************************************
int Can1BusyCheck(void)
{
    uint32_t counter = 0;

    while (!(((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) || ((CAN1->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) || ((CAN1->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)))
    {
        counter++;
        if (counter >= CAN_BUSY_TIMEOUT)
        {
            counter = 0;
            //CAN_DeInit(CAN1);
            //CAN1_Init();
            return CAN_BUS_STATE_RESET;
        }
    }
    return CAN_BUS_STATE_FREE;
}

//*********************************************************************************
//��������: ActrDevInit()
//��    ����ִ�����豸��ʼ��
//��ڲ�������
//���ڲ�����CAN����״̬
//��    ע��Editor��Liuqh 2018-01-29    Company: INNFOS
//*********************************************************************************
void ActrDevInit(void)
{
    uint32_t i;
    for (i = 0; i < ACTR_DEV_NUM; i++)
    {
        ActrDevList[i].actrID = devIDList[i];
    }
}

//*********************************************************************************
//��������: CanRecvFramAnalyse()
//��    ����CAN���߽�������Э�����
//��ڲ�����CanRxMsg* CAN�������ݽṹ��ָ�� ��ActrParaTypedef* ִ�����豸�ṹ��ָ��
//���ڲ�������
//��    ע��Editor��Liuqh 2018-01-29    Company: INNFOS
//*********************************************************************************
void CanRecvFramAnalyse(CanRxMsg *pCanRxMsg, ActrParaTypedef *pActrParaDev)
{
    ActrParaTypedef *pTempActrDev = NULL;
    pTempActrDev = FindActrDevByID(pCanRxMsg->StdId);
    if (pTempActrDev != NULL)
    {
        switch (pCanRxMsg->Data[CAN_FRAME_BIT_CMD])
        {
        case ACTR_CMD_SHAKE_HAND:
        {
            if (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] == CAN_FRAME_ACK_SUCCESS)
            {
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
            else
            {
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_FAIL;
            }
        }
        break;
        case ACTR_CMD_GET_CURRENT:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrCurrent = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                    (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                            IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_SPEED:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrSpeed = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                  (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                          IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_POSTION:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrPostion = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                    (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                            IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;

        case ACTR_CMD_SET_MODE:
        {
            if (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] == CAN_FRAME_ACK_SUCCESS)
            {
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
            else
            {
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_FAIL;
            }
        }
        break;

        case ACTR_CMD_SET_ON_OFF:
        {
            if (pCanRxMsg->DLC == 0x02)
            {
                if (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] == CAN_FRAME_ACK_SUCCESS)
                {
                    pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
                }
                else
                {
                    pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_FAIL;
                }
            }
            pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS; //TODO:�ٷ��������ޱ�����䣬������GetActrPara(ACTR_CMD_GET_ON_OFF, DemoActrID)ʱ������������ΪGET_PARA_ERR_ACK_ERR
        }
        break;
        case ACTR_CMD_GET_ON_OFF:
        {
            if (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] == PWR_ON)
            {
                pTempActrDev->actrPwrState = PWR_ON;
            }
            else
            {
                pTempActrDev->actrPwrState = PWR_OFF;
            }
        }
        break;
        case ACTR_CMD_GET_MOTOR_TEMP:
        {
            pTempActrDev->actrMotorTemp = ((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 8) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH])) / TempFactor;
            pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
        }
        break;
        case ACTR_CMD_GET_INVTR_TEMP:
        {
            pTempActrDev->actrInvterTemp = ((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 8) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH])) / TempFactor;
            pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
        }
        break;
        case ACTR_CMD_GET_CUR_MODE:
        {
            pTempActrDev->actrMode = (ActrRunModeTypedef)pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH];
            pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
        }
        break;
        case ACTR_CMD_GET_EXECPTION:
        {
            pTempActrDev->actrWarnState.ActrWarnStateAll = (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 8) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH]);
            pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
        }
        break;
        case ACTR_CMD_GET_TSHAP_POS_MAX_SPEED:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrTshapPosMaxSpeed = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                             (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                                     IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_TSHAP_POS_ACCELERATE:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrTshapPosAccelerate = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                               (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                                       IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_TSHAP_POS_DECELERATE:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrTshapPosDecelerate = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                               (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                                       IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_POSTION_OUTPUT_LOWER_LIMIT:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrPosOutputLowerLimit = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                                (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                                        IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_POSTION_OUTPUT_UPPER_LIMIT:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrPosOutputUpperLimit = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                                (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                                        IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_POSTION_LOWER_LIMIT:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrPosLowerLimit = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                          (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                                  IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_POSTION_UPPER_LIMIT:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrPosUpperLimit = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                          (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                                  IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_SPEED_OUTPUT_LOWER_LIMIT:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrSpeedOutputLowerLimit = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                                  (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                                          IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_SPEED_OUTPUT_UPPER_LIMIT:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrSpeedOutputUpperLimit = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                                  (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                                          IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_CURRENT_OUTPUT_LOWER_LIMIT:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrCurrentOutputLowerLimit = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                                    (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                                            IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_CURRENT_OUTPUT_UPPER_LIMIT:
        {
            if (pCanRxMsg->DLC == 0x05)
            {
                pTempActrDev->actrCurrentOutputUpperLimit = (float)((pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH] << 24) | (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HL] << 16) |
                                                                    (pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LH] << 8) | pCanRxMsg->Data[CAN_FRAME_BIT_DAT_LL]) /
                                                            IQ24Factor;
                pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
            }
        }
        break;
        case ACTR_CMD_GET_SHUTDOWN_STATE:
        {
            pTempActrDev->actrShutdownState = (ActrShutdownStateTypedef)pCanRxMsg->Data[CAN_FRAME_BIT_DAT_HH];
            pTempActrDev->actrRecvACKState = CAN_FRAME_ACK_SUCCESS;
        }
        break;
        }
    }
}
