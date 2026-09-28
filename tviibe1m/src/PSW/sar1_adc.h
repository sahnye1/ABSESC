/**
 * @file    sar1_adc.h
 * @brief   压力传感器 + VPOWER 电压采集 (复用 SAR0 的 CH6~CH8)
 *
 * @details 硬件链路: SAR0 (ADC0) 九通道合成一个 Group 自动序列, 软件触发/轮询
 *          CH6 (AN13, P7.5): 前桥压力传感器, 0.5V~4.5V → 0~1000kPa
 *          CH7 (AN1,  P6.1): 后桥压力传感器, 0.5V~4.5V → 0~1000kPa
 *          CH8 (AN10, P7.2): VPOWER 电压分压 (继电器后), 0~5V → 0~55V (分压比=11.0)
 *          CH8 为组尾; CH0~CH5 由 sensor_diag 占用
 */

#ifndef SAR1_ADC_H
#define SAR1_ADC_H

#include <stdbool.h>
#include <stdint.h>
#include "peri_div.h"

/* ========================================================================== */
/*  ADC 硬件配置                                                               */
/* ========================================================================== */
#define SAR1_ADC_MACRO          PASS0_SAR0
#define SAR1_ADC_PCLK           PCLK_PASS0_CLOCK_SAR0
#define SAR1_ADC_CLK_DIV        DIV16_NO_ADC   /* 与 sensor_diag 共享同一 SAR0 分频器 */

/* 通道定义: CH6=AN13(前桥P7.5), CH7=AN1(后桥P6.1), CH8=AN10(VPOWER P7.2, 组尾)
 * 9 个信号(轮速6+压力2+VPOWER1)全在 SAR0(ADC0), 合成一个 group CH0~CH8 */
#define SAR1_CH_FRONT           6u
#define SAR1_CH_REAR            7u
#define SAR1_CH_VPWR            8u
#define SAR1_NUM_CHANNELS       3u

/* 引脚: SARMUX 直连 → GPIO (HSIOM=0), Analog drive mode */
#define SAR1_CH_FRONT_PORT      GPIO_PRT7
#define SAR1_CH_FRONT_PIN       5u
#define SAR1_CH_FRONT_MUX       P7_5_GPIO

#define SAR1_CH_REAR_PORT       GPIO_PRT6
#define SAR1_CH_REAR_PIN        1u
#define SAR1_CH_REAR_MUX        P6_1_GPIO

#define SAR1_CH_VPWR_PORT       GPIO_PRT7
#define SAR1_CH_VPWR_PIN        2u
#define SAR1_CH_VPWR_MUX        P7_2_GPIO

/* ADC 参考电压 */
#define SAR1_ADC_VREF_MV        5000u
#define SAR1_ADC_12BIT_MAX      4095u

/* ========================================================================== */
/*  VPOWER 电压检测 (分压电路 R1=47k, R2=4.7k, 继电器后)                         */
/* ========================================================================== */
/* 分压比 = (R1 + R2) / R2 = (47 + 4.7) / 4.7 = 11.0
 * 36V → ADC 3.27V, 18V → ADC 1.64V (均在 0-5V 范围内) */
#define VPWR_DIVIDER_RATIO_NUM  110u    /* 分压比 × 10 (分子), 11.0 */
#define VPWR_DIVIDER_RATIO_DEN   10u    /* 分压比 × 10 (分母)     */
/* VPOWER 电压 = ADC_V(mV) × (NUM/DEN) / 100 = ADC_V × 11.0 / 100 = V×10 */

/* 报警阈值 (fact:0.1V) */
#define VPWR_ALARM_HIGH_MV      360u    /* 36.0V, 高压报警 */
#define VPWR_ALARM_LOW_MV       180u    /* 18.0V, 低压报警 */

/* 连续判定次数 (对称消抖) */
#define VPWR_ALARM_STREAK        10u

/* ========================================================================== */
/*  压力传感器故障诊断 (双层窗口 + 对称消抖)                                      */
/*                                                                            */
/*  0V ──── 0.23V ──── 0.5V ──────────── 4.7V ──── 4.77V ──── 5V            */
/*        ├─ 故障区 ─┤├─ 容忍 ┤├─ 正常计算区 ─┤├ 容忍 ┤├─ 故障区 ┤            */
/*                  ERR_MIN  NORMAL_MIN    NORMAL_MAX  ERR_MAX                 */
/*                                                                            */
/*  raw < ERR_MIN 或 raw > ERR_MAX → errCnt++   (报故障)                       */
/*  容忍区内可计算压力, 但 clamp 到 0 或 MAX_KPA                                */
/* ========================================================================== */

/* 故障阈值 (12bit ADC raw): <0.23V 或 >4.77V 为故障 */
#define PRESS_ERR_MIN_RAW       189u    /* 0.23V → 189 */
#define PRESS_ERR_MAX_RAW       3907u   /* 4.77V → 3907 */

/* 正常区间 (12bit ADC raw): 0.5V~4.7V 内做线性压力换算 */
#define PRESS_NORMAL_MIN_RAW    410u    /* 0.5V → 410 */
#define PRESS_NORMAL_MAX_RAW    3849u   /* 4.7V → 3849 */

/* 对称消抖次数 (50 × 10ms = 0.5s 连报, 0.5s 连恢复) */
#define PRESS_FAULT_STREAK      50u

/* ========================================================================== */
/*  压力换算 (0.5V~4.7V → 0~PRESS_MAX_KPA kPa)                                   */
/* ========================================================================== */
#define PRESS_MAX_KPA           1050u   /* 最大量程 kPa */

/* ========================================================================== */
/*  对外 API                                                                   */
/* ========================================================================== */

void Sar1Adc_Init(void);

/**
 * @brief   触发一轮 ADC Group 转换 (CH0→CH1→CH2), 阻塞等待完成,
 *          读取结果 → 压力/电压换算 → 故障诊断/报警
 * @note    主循环每 10ms 调用, 阻塞 ~几十 μs
 */
void Sar1Adc_Read(void);

/* ---- 压力查询接口 ---- */

uint16_t Pressure_GetFrontRaw(void);
uint16_t Pressure_GetRearRaw(void);
uint16_t Pressure_GetFrontKpa(void);
uint16_t Pressure_GetRearKpa(void);
bool     Pressure_IsFrontFault(void);
bool     Pressure_IsRearFault(void);

/* ---- VPOWER 查询接口 (P7.2 AN10, 继电器后电压) ---- */

uint16_t Vpower_GetRaw(void);             /* ADC 原始值 */
uint16_t Vpower_GetVoltage(void);         /* fact:0.1V */
bool     Vpower_IsHighAlarm(void);        /* > 36.0V */
bool     Vpower_IsLowAlarm(void);         /* < 18.0V */

#endif /* SAR1_ADC_H */
