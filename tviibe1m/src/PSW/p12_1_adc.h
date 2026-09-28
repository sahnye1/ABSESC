/**
 * @file    p12_1_adc.h
 * @brief   P12.1 独立 SAR1 ADC 采集 (0~5V 模拟输入)
 *
 * @details 硬件链路: P12.1 = PADS37 = ADC[1](PASS0_SAR1) 的 AN5
 *          用独立 PASS0_SAR1 实例, 单通道软件触发 + 轮询,
 *          不占用 SAR0 (诊断6CH + 压力/VPOWER) 的 group。
 *
 * @note    参考 examples/adc/OneChannelConversion_SwTrigger 单通道例子,
 *          但改为轮询方式 (与项目 10ms 主循环节奏一致, 不引入新中断)。
 *          开关 HW_REV_P12_1_ADC (hw_rev.h), 默认 0=关闭, 改版后置 1。
 */

#ifndef P12_1_ADC_H
#define P12_1_ADC_H

#include <stdint.h>
#include "peri_div.h"
#include "hw_rev.h"

#if (HW_REV_P12_1_ADC == 1u)

/* ========================================================================== */
/*  硬件配置                                                                   */
/* ========================================================================== */
#define P12_1_ADC_MACRO         PASS0_SAR1
#define P12_1_ADC_PCLK          PCLK_PASS0_CLOCK_SAR1
#define P12_1_ADC_CLK_DIV       DIV16_NO_ADC2   /* 独立分频器, 与 SAR0 分开 */
#define P12_1_ADC_CH            0u              /* 逻辑通道 CH0 (单通道, 组头=组尾) */

/* 引脚: P12.1 模拟直连 (GPIO + ANALOG), SAR1 AN5 */
#define P12_1_ADC_PORT          GPIO_PRT12
#define P12_1_ADC_PIN           1u
#define P12_1_ADC_MUX           P12_1_GPIO

/* ADC 参考电压 */
#define P12_1_ADC_VREF_MV       5000u
#define P12_1_ADC_12BIT_MAX     4095u

#endif /* HW_REV_P12_1_ADC == 1u */

/* ========================================================================== */
/*  对外 API (开关关闭时由 .c 提供空实现, 调用点无需条件编译)                   */
/* ========================================================================== */

/** @brief P12.1 独立 SAR1 ADC 初始化 (时钟 + 引脚 + ADC + 单通道) */
void P12_1_Adc_Init(void);

/** @brief 触发一次 P12.1 单通道转换, 轮询完成, 读结果并换算 mV */
void P12_1_Adc_Read(void);

/** @brief P12.1 ADC 原始值 (12bit) */
uint16_t P12_1_GetRaw(void);

/** @brief P12.1 电压 (mV) */
uint16_t P12_1_GetVoltageMv(void);

#endif /* P12_1_ADC_H */
