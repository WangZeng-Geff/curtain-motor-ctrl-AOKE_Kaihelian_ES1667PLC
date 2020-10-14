/*************************************************************************************************/
/*
Copyright:      QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:      uart_reg.h
Description:    the uart control
Author:         ch
Version:        v1.0
Date:           2017/08/01
History:
*/
/************************************************************************************************/
#ifndef __UART_REG_H
#define __UART_REG_H

#include "ssc1667.h"
#include "types.h"

/* ֹͣλ */
typedef enum
{
    UART_StopBits_1 = 0x0,        //1λֹͣλ
    UART_StopBits_2 = 0x1,        //2λֹͣλ
} UART_TYPE_TXFS;

/* �˿ڼ��� */
typedef enum
{
    UART_Polar_Normal    = 0,    //������
    UART_Polar_Opposite  = 1,    //������
} UART_TYPE_RTXP;

/* ʱ��ѡ�� */
typedef enum
{
    UART_Clock_0 = 0x0,         //close
    UART_Clock_1 = 0x1,         //PCLK
    UART_Clock_2 = 0x2,         //PCLK / 2
    UART_Clock_3 = 0x3,         //PCLK / 4
    UART_Clock_4 = 0x4,         //PCLK / 8
} UART_TYPE_BCS;

/* ��ʼ���ṹ�� */
typedef struct
{
    UART_TYPE_TXFS     UART_StopBits;    //ֹͣλѡ��
    UART_TYPE_DATAMOD  UART_TxMode;      //�������ݸ�ʽ
    UART_TYPE_RTXP     UART_TxPolar;     //���Ͷ˿ڼ���
    UART_TYPE_DATAMOD  UART_RxMode;      //�������ݸ�ʽ
    UART_TYPE_RTXP     UART_RxPolar;     //���ն˿ڼ���
    uint32_t           UART_BaudRate;    //������
    UART_TYPE_BCS      UART_ClockSet;    //UARTʱ��ѡ��
} UART_InitStruType;

/* ����֡���ģʽ */
typedef enum
{
    UART_IDM_Idle_1 = 0x0,          //����10��bit����
    UART_IDM_Idle_2 = 0x1,          //����11��bit����
    UART_IDM_Idle_3 = 0x2,          //����12��bit����
    UART_IDM_Idle_4 = 0x3,          //����13��bit����
} UART_TYPE_IDM;

/* �Զ�������ģʽ */
typedef enum
{
    UART_BDM_Mode_1 = 0x0,          //�����������λΪ1B�������ʼλ������
    UART_BDM_Mode_2 = 0x1,          //�������������λΪ01B�������ʼλ�����ݵ�һλ������
    UART_BDM_Mode_3 = 0x2,          //��������7FH�������ʼλ��ǰ7������λ
    UART_BDM_Mode_4 = 0x3,          //��������55H�������ʼλ��ǰ7������λ
} UART_TYPE_BDM;

/* Interrupt */
typedef enum
{
    UART_IT_TB    = (1 << 0),        //���ͻ��������ж�
    UART_IT_TC    = (1 << 1),        //��������ж�
    UART_IT_TBWE  = (1 << 8),        //���ͻ�������ж�
    UART_IT_TBWO  = (1 << 9),        //���ͻ�������ж�
    UART_IT_RB    = (1 << 16),       //���ջ��������ж�
    UART_IT_ID    = (1 << 17),       //���տ���֡�ж�
    UART_IT_RO    = (1 << 24),       //������������ж�
    UART_IT_FE    = (1 << 25),       //����֡�����ж�
    UART_IT_PE    = (1 << 26),       //����У������ж�
    UART_IT_BDE   = (1 << 27),       //�����ʼ������ж�
    UART_IT_RBRE  = (1 << 28),       //�����ջ�������ж�
    UART_IT_RBRO  = (1 << 29),       //�����ջ�������ж�
} UART_TYPE_IT;

/* Interrupt Flag */
typedef enum
{
    UART_FLAG_TB   = (1 << 0),      //���ͻ��������ж�
    UART_FLAG_TC   = (1 << 1),      //��������ж�
    UART_FLAG_TBWE = (1 << 8),      //���ͻ�������ж�
    UART_FLAG_TBWO = (1 << 9),      //���ͻ�������ж�
    UART_FLAG_RB   = (1 << 16),     //���ջ��������ж�
    UART_FLAG_ID   = (1 << 17),     //���տ���֡�ж�
    UART_FLAG_RO   = (1 << 24),     //������������ж�
    UART_FLAG_FE   = (1 << 25),     //����֡�����ж�
    UART_FLAG_PE   = (1 << 26),     //����У������ж�
    UART_FLAG_BDE  = (1 << 27),     //�����ʼ������ж�
    UART_FLAG_RBRE = (1 << 28),     //�����ջ�������ж�
    UART_FLAG_RBRO = (1 << 29),     //�����ջ�������ж�
} UART_TYPE_FLAG;

/* Interrupt Clear Flag */
typedef enum
{
    UART_CLR_TC    = (1 << 1),       //��������ж�
    UART_CLR_TBWE  = (1 << 8),       //���ͻ�������ж�
    UART_CLR_TBWO  = (1 << 9),       //���ͻ�������ж�
    UART_CLR_ID    = (1 << 17),      //���տ���֡�ж�
    UART_CLR_RO    = (1 << 24),      //������������ж�
    UART_CLR_FE    = (1 << 25),      //����֡�����ж�
    UART_CLR_PE    = (1 << 26),      //����У������ж�
    UART_CLR_BDE   = (1 << 27),      //�����ʼ������ж�
    UART_CLR_RBRE  = (1 << 28),      //�����ջ�������ж�
    UART_CLR_RBRO  = (1 << 29),      //�����ջ�������ж�
} UART_CLR_IF;

#define UART_CLR_ALL  (0xFFFFFFFF)     /* ���ȫ���жϱ�־λ */
/* Interrupt Mode */
typedef enum
{
    UART_TRBIM_Byte     = 0x0,       //�ֽ��ж�
    UART_TRBIM_HalfWord = 0x1,       //�����ж�
    UART_TRBIM_Word     = 0x2,       //���ж�
    UART_RBIM_Full      = 0x3,       //ȫ�ж�
    UART_TBIM_EMPTY     = 0x3,       //ȫ�ж�
} UART_TYPE_TRBIM;

/* Status */
typedef enum
{
    UART_STA_TBPTR     = (0x0F << 0),  /* ���ͻ������ֽ��� */
    UART_STA_TBOV      = (1 << 4),     //���ͻ��������״̬λ
    UART_STA_TXBUSY    = (1 << 5),     //����״̬λ
    UART_STA_RBPTR     = (0x0F << 8),  /* ���ջ������ֽ��� */
    UART_STA_RBOV      = (1 << 12),    //���ջ��������״̬λ
    UART_STA_RXBUSY    = (1 << 13),    //����״̬λ
    UART_STA_FER0      = (1 << 16),    //��ǰ��ȡBYTE0֡��ʽ����λ
    UART_STA_PER0      = (1 << 17),    //��ǰ��ȡBYTE0У�����λ
    UART_STA_FER1      = (1 << 18),    //��ǰ��ȡBYTE1֡��ʽ����λ
    UART_STA_PER1      = (1 << 19),    //��ǰ��ȡBYTE1У�����λ
    UART_STA_FER2      = (1 << 20),    //��ǰ��ȡBYTE2֡��ʽ����λ
    UART_STA_PER2      = (1 << 21),    //��ǰ��ȡBYTE2У�����λ
    UART_STA_FER3      = (1 << 22),    //��ǰ��ȡBYTE3֡��ʽ����λ
    UART_STA_PER3      = (1 << 23),    //��ǰ��ȡBYTE3У�����λ
} UART_TYPE_STA;

#define UART_RXCNT_OFFSET  (8)
#define UART_TXCNT_OFFSET  (0)

#define _UARTx(chn)   (UART0_BASE + ((chn) * (0x400)))

#define UART_MAX_INX      (3)
#define UART_DMode_Mask   (0x0F)

#define INVAL_DMODE_CHK(mode)   ((mode == 0x01) || (mode == 0x03)\
                                 ||(mode == 0x05) || (mode == 0x07))

#define UART_GetITStatus(UARTx, flag) (((UARTx)->STA.Word) & flag)
#define UAER_GetRXCnt(UARTx)          ((((UARTx)->STA.Word) & UART_STA_RBPTR) >> UART_RXCNT_OFFSET)
#define UAER_GetTXCnt(UARTx)          ((((UARTx)->STA.Word) & UART_STA_TBPTR) >> UART_TXCNT_OFFSET)
#define UART_ReceiveData(UARTx)       ((UARTx)->RBR.Byte[0])
#define UART_SendData(UARTx, c)       ((UARTx)->TBW.Byte[0] = c)
#define UART_ClearIF(UARTx, flag)     ((UARTx)->IF.Word = (flag))
#define UART_GetIF(UARTx, flag)       ((UARTx)->IF.Word & (flag))

#endif
