/**
 * @file    sensor_diag.h
 * @brief   ADC 传感器故障诊断 — 开路/短路/间隙过大检测
 *
 * @details 硬件链路: WSS 传感器 → LM2902M 差分放大 → MCU SARMUX ADC (SAR0)
 *          采样策略: 每 10ms 一轮 6 通道, 100 点环形缓冲 (1s 窗口)
 *          故障诊断基于 100 点平均值/峰峰值 + 滞回 + 速度门控
 */

#ifndef SENSOR_DIAG_H
#define SENSOR_DIAG_H

#include <stdbool.h>
#include <stdint.h>
#include "peri_div.h"

/* ========================================================================== */
/*  ADC 硬件配置                                                               */
/* ========================================================================== */
#define ADC_DIAG_MACRO          PASS0_SAR0
#define ADC_DIAG_PCLK           PCLK_PASS0_CLOCK_SAR0

#define ADC_DIAG_FIRST_CH       0u
#define ADC_DIAG_CH0            0u
#define ADC_DIAG_CH1            1u
#define ADC_DIAG_CH2            2u
#define ADC_DIAG_CH3            3u
#define ADC_DIAG_CH4            4u
#define ADC_DIAG_CH5            5u
#define ADC_DIAG_LAST_CH        5u
#define ADC_DIAG_NUM_CHANNELS   6u

/* 通道使能开关 */
#define ADC_DIAG_CH0_ENABLE     1u
#define ADC_DIAG_CH1_ENABLE     1u
#define ADC_DIAG_CH2_ENABLE     1u
#define ADC_DIAG_CH3_ENABLE     1u
#define ADC_DIAG_CH4_ENABLE     1u
#define ADC_DIAG_CH5_ENABLE     1u

#define IS_ADC_DIAG_CH_ENABLED(ch)  \
    ((ch) == 0u ? ADC_DIAG_CH0_ENABLE : \
     (ch) == 1u ? ADC_DIAG_CH1_ENABLE : \
     (ch) == 2u ? ADC_DIAG_CH2_ENABLE : \
     (ch) == 3u ? ADC_DIAG_CH3_ENABLE : \
     (ch) == 4u ? ADC_DIAG_CH4_ENABLE : \
     (ch) == 5u ? ADC_DIAG_CH5_ENABLE : 0u)

#define ADC_DIAG_CLK_DIV        DIV16_NO_ADC   /* 与 sar1_adc 共享同一 SAR0 分频器 */
#define ADC_DIAG_BUF_SIZE       100u

/* ADC 诊断引脚: SARMUX 直连 (HSIOM=0, ANALOG) */
#define ADC_DIAG_CH0_PORT       GPIO_PRT6
#define ADC_DIAG_CH0_PIN        4u
#define ADC_DIAG_CH0_MUX        P6_4_GPIO

#define ADC_DIAG_CH1_PORT       GPIO_PRT6
#define ADC_DIAG_CH1_PIN        5u
#define ADC_DIAG_CH1_MUX        P6_5_GPIO

#define ADC_DIAG_CH2_PORT       GPIO_PRT7
#define ADC_DIAG_CH2_PIN        0u
#define ADC_DIAG_CH2_MUX        P7_0_GPIO

#define ADC_DIAG_CH3_PORT       GPIO_PRT7
#define ADC_DIAG_CH3_PIN        1u
#define ADC_DIAG_CH3_MUX        P7_1_GPIO

#define ADC_DIAG_CH4_PORT       GPIO_PRT7
#define ADC_DIAG_CH4_PIN        3u
#define ADC_DIAG_CH4_MUX        P7_3_GPIO

#define ADC_DIAG_CH5_PORT       GPIO_PRT7
#define ADC_DIAG_CH5_PIN        4u
#define ADC_DIAG_CH5_MUX        P7_4_GPIO

/* 电压换算常量 */
#define ADC_VREF_MV             5000u
#define ADC_12BIT_MAX           4095u
#define ADC_MV_TO_RAW(mv)       ((mv) * ADC_12BIT_MAX / ADC_VREF_MV)

/* 故障诊断阈值 (12bit ADC 原始值) */
#define DIAG_OPEN_MIN           0x1C0
#define DIAG_OPEN_MAX           0x22D
#define DIAG_OPEN_HYST          0x50

#define DIAG_SHORT_MIN          0x9B0
#define DIAG_SHORT_MAX          0xAEF
#define DIAG_SHORT_HYST         0x50

#define DIAG_GAP_AMPL_MIN       0x30
#define DIAG_GAP_AMPL_HYST      0x10
#define DIAG_GAP_RPM_THRESH     417u   /* 15(0.1km/h) × 250/9 ≈ 417 mm/s, 与 wheel_speed 单位对齐 */

/* ========================================================================== */
/*  数据结构                                                                   */
/* ========================================================================== */

/* 单通道 ADC 诊断结果 */
typedef struct
{
    uint16_t adc_raw;
    uint16_t voltage_mv;
    bool     fault_short;
    bool     fault_open;
    bool     fault_gap_too_large;
} sensor_ch_diag_t;

/* 六通道诊断结果 (volatile, 主循环写入/读取) */
extern volatile sensor_ch_diag_t g_sensor_diag[ADC_DIAG_NUM_CHANNELS];

/* 传感器故障状态汇总 (供外部模块查询) */
typedef struct
{
    bool ch0_fault_open;
    bool ch0_fault_short;
    bool ch0_fault_gap_too_large;
    bool ch1_fault_open;
    bool ch1_fault_short;
    bool ch1_fault_gap_too_large;
    bool ch2_fault_open;
    bool ch2_fault_short;
    bool ch2_fault_gap_too_large;
    bool ch3_fault_open;
    bool ch3_fault_short;
    bool ch3_fault_gap_too_large;
    bool ch4_fault_open;
    bool ch4_fault_short;
    bool ch4_fault_gap_too_large;
    bool ch5_fault_open;
    bool ch5_fault_short;
    bool ch5_fault_gap_too_large;
} sensor_fault_status_t;

extern volatile sensor_fault_status_t g_sensor_fault_status;

/* ========================================================================== */
/*  对外 API                                                                   */
/* ========================================================================== */

/**
 * @brief   初始化 SAR ADC0 (时钟 + 引脚 + 通道配置)
 */
void SensorDiag_Init(void);

/**
 * @brief   ADC 采样 + 故障诊断 — 主循环每 10ms 调用
 * @note    单函数完成: 触发转换 → 结果入 100 点缓冲 → 统计 → 诊断,
 *          阻塞 ~几十 μs; 缓冲满 100 点 (1s) 后开始输出诊断,
 *          结果写入 g_sensor_fault_status。
 */
void SensorDiag_CalcAndProcess(void);

/**
 * @brief   ADC 中断处理 (保留空函数体, 当前使用软件轮询方式)
 */
void SensorDiag_IntHandler(void);

#endif /* SENSOR_DIAG_H */
