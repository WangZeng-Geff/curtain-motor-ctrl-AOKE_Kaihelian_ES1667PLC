/*************************************************************************************************/
/*
Copyright:      QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:      gpio_reg.h
Description:    the gpio control
Author:         ch
Version:        v1.0
Date:           2017/08/03
History:
*/
/************************************************************************************************/

#ifndef __GPIO_REG_H
#define __GPIO_REG_H


#include "ssc1667.h"
#include "types.h"


/* ���������©ʹ��λ */
typedef enum
{
    GPIO_ODE_OUT_DIS = 0,   // ��©��ֹ
    GPIO_ODE_OUT_EN  = 1,   // ��©ʹ��
} GPIO_ODE_Output;

/* �������������������ѡ�� */
typedef enum
{
    GPIO_DS_OUT_NOMAL  = 0,    // ��ͨ�������
    GPIO_DS_OUT_STRONG = 1,    // ǿ�������
} GPIO_DS_Output;

/* ��������������ʹ�� */
typedef enum
{
    GPIO_PUE_IN_DIS = 0,   // ��������ֹ
    GPIO_PUE_IN_EN = 1,    // ������ʹ��
} GPIO_PUE_Input;

/* ��������������ʹ�� */
typedef enum
{
    GPIO_PDE_IN_DIS = 0,    // ��������ֹ
    GPIO_PDE_IN_EN  = 1,    // ������ʹ��
} GPIO_PDE_Input;

/* �����ź����� */
typedef enum
{
    GPIO_SIG_DIG  = 0,  // �����ź�����
    GPIO_SIG_ANA  = 1,   // ģ���ź�����
} GPIO_Pin_Signal;

/* GPIO��ʼ�����ýṹ���� */
typedef struct
{
    GPIO_Pin_Signal Signal;       // �����ϵ��ź����ͣ�ֻ��ģ�����������

    GPIO_DIR        Dir;          // ��������ʱ���������롢�������ѡ��
    GPIO_Reuse_Func Func;         // ��������ʱ�����Ÿ��ù���ѡ��

    GPIO_ODE_Output ODE_Output;   // �������š��������ʱ����©���ѡ��
    GPIO_DS_Output DS_Output;     // �������š��������ʱ�����������Сѡ��

    GPIO_PUE_Input PUE_Input;     // �������š����뷽��ʱ��������ѡ��
    GPIO_PDE_Input PDE_Input;     // �������š����뷽��ʱ��������ѡ��
} GPIO_InitSettingType;

/* �ⲿ�˿��ж�ʹ�� */
typedef enum
{
    PINT_IE_DIS = 0,              // �жϽ�ֹ
    PINT_IE_EN  = 1,              // �ж�ʹ��
} PINT_IE_Set;

/* KINT�ж�ʹ�� */
typedef enum
{
    KINT_IE_Set_Disable = 0,          // �жϽ�ֹ
    KINT_IE_Set_Enable = 1,           // �ж�ʹ��
} KINT_IE_Set;

/* KINT�жϷ�ʽ */
typedef enum
{
    KINT_Trigger_Rising_Edge = 0,    // �����ش����ж�
    KINT_Trigger_Trailing_Edge = 1,  // �½��ش����ж�
    KINT_Trigger_High_Level = 2,     // �ߵ�ƽ�����ж�
    KINT_Trigger_Low_Level = 3,      // �͵�ƽ�����ж�
    KINT_Trigger_Both_Edge = 4,      // �����غ��½��ض������ж�
} KINT_Trigger_Style;

/* KINT��ʼ�����ýṹ���� */
typedef struct
{
    KINT_IE_Set IE_Set;               // �ж��Ƿ�ʹ��
    KINT_Trigger_Style Trigger_Style; // �жϴ�����ʽ
} KINT_InitSettingType;

typedef enum
{
    GPIO_PINT0          = 0x00,
    GPIO_PINT1          = 0x01,
    GPIO_PINT2          = 0x02,
    GPIO_PINT3          = 0x03,
    GPIO_PINT4          = 0x04,
    GPIO_PINT5          = 0x05,
    GPIO_PINT6          = 0x06,
    GPIO_PINT7          = 0x07,
} GPIO_PINT;

#define GPIO_IRQn_ENUM   GPIO0_IRQn, GPIO1_IRQn, GPIO2_IRQn, GPIO3_IRQn, GPIO4_IRQn, GPIO5_IRQn, GPIO6_IRQn, GPIO7_IRQn

#define GPIO_PINT_MAX                   (8)

#define GPIO_PINTx(pin)                 ((pin) & 0x07)
#define GPIOx_FUNCx(pin, func)          (func << (GPIO_PINTx(pin) * 4))

#define GPIO_PINT_CLRMASK(pint)         (GPIO->PINTIE.PMASK_V &= ~(0x01 << (pint)))
#define GPIO_PINTEN(pint)               (GPIO->PINTIE.Word |= (0x01 << (pint)))
#define GPIO_PINTDIS(pint)              (GPIO->PINTIE.Word &= (~(0x01 << (pint))))

#define GPIO_GETIF(pint)                (GPIO->PINTIF.Word &= (0x01 << (pint)))
#define GPIO_CLRIF(pint)                (GPIO->PINTIF.Word = (0x01 << (pint)))

#define GPIO_SELx(pin)                  (((pin) & 0x1F) >> 3)
#define GPIO_SETSEL(pin)                (GPIO->PINTSEL.Word |= GPIO_SELx(pin) << (GPIO_PINTx(pin) * 4))

#define GPIO_SETTRI(pint, tri)           (GPIO->PINTCFG.Word |= (tri) << ((pint) * 4))


#endif /* __GPIO_reg  */

