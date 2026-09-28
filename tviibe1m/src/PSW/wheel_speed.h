/**
 * @file    wheel_speed.h
 * @brief   六通道轮速采集 — TCPWM 捕获 + 双模算法 (测周法/计齿法) + 中值滤波
 *
 * @details 硬件链路: WSS 传感器 → LM2901DR 比较整形 → MCU TCPWM 捕获引脚
 *          算法: 低速 (<10齿/10ms) 测周法, 高速 (≥10齿/10ms) 计齿法
 */

#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdbool.h>
#include <stdint.h>
#include "hw_rev.h"
#include "peri_div.h"

/* ========================================================================== */
/*  六路捕获通道硬件配置 (CH0 ~ CH5) — 全部 6 通道均有 TCPWM 功能               */
/* ========================================================================== */

/* CH0: P8.2,  TR_ONE_CNT_IN63,  TCPWM0_GRP0_CNT21 — LX (附加桥左) */
#define CH0_TCPWM              TCPWM0_GRP0_CNT21
#define CH0_PCLK               PCLK_TCPWM0_CLOCKS21
#define CH0_PORT               GPIO_PRT8
#define CH0_PIN                2ul
#define CH0_MUX                P8_2_TCPWM0_TR_ONE_CNT_IN63
#define CH0_IRQ_SRC            tcpwm_0_interrupts_21_IRQn
#define CH0_CLK_DIV            DIV16_NO_WHEEL   /* 6 通道共享同一 2MHz 分频器 */

/* CH1: P12.0, TR_ONE_CNT_IN108, TCPWM0_GRP0_CNT36 — FL (前桥左) */
#define CH1_TCPWM              TCPWM0_GRP0_CNT36
#define CH1_PCLK               PCLK_TCPWM0_CLOCKS36
#define CH1_PORT               GPIO_PRT12
#define CH1_PIN                0
#define CH1_MUX                P12_0_TCPWM0_TR_ONE_CNT_IN108
#define CH1_IRQ_SRC            tcpwm_0_interrupts_36_IRQn
#define CH1_CLK_DIV            DIV16_NO_WHEEL

/* CH2: RX (附加桥右) 轮速捕获 — P8.1, TR_ONE_CNT_IN60, CNT20 (飞线后固定, 见 hw_rev.h) */
#define CH2_TCPWM              TCPWM0_GRP0_CNT20
#define CH2_PCLK               PCLK_TCPWM0_CLOCKS20
#define CH2_PORT               GPIO_PRT8
#define CH2_PIN                1ul
#define CH2_MUX                P8_1_TCPWM0_TR_ONE_CNT_IN60
#define CH2_IRQ_SRC            tcpwm_0_interrupts_20_IRQn
#define CH2_CLK_DIV            DIV16_NO_WHEEL

/* CH3: P12.2, TR_ONE_CNT_IN114, TCPWM0_GRP0_CNT38 — RR (后桥右) */
#define CH3_TCPWM              TCPWM0_GRP0_CNT38
#define CH3_PCLK               PCLK_TCPWM0_CLOCKS38
#define CH3_PORT               GPIO_PRT12
#define CH3_PIN                2ul
#define CH3_MUX                P12_2_TCPWM0_TR_ONE_CNT_IN114
#define CH3_IRQ_SRC            tcpwm_0_interrupts_38_IRQn
#define CH3_CLK_DIV            DIV16_NO_WHEEL

/* CH4: P12.3, TR_ONE_CNT_IN117, TCPWM0_GRP0_CNT39 — LR (后桥左) */
#define CH4_TCPWM              TCPWM0_GRP0_CNT39
#define CH4_PCLK               PCLK_TCPWM0_CLOCKS39
#define CH4_PORT               GPIO_PRT12
#define CH4_PIN                3ul
#define CH4_MUX                P12_3_TCPWM0_TR_ONE_CNT_IN117
#define CH4_IRQ_SRC            tcpwm_0_interrupts_39_IRQn
#define CH4_CLK_DIV            DIV16_NO_WHEEL

/* CH5: P12.4, TR_ONE_CNT_IN120, TCPWM0_GRP0_CNT40 — RF (前桥右) */
#define CH5_TCPWM              TCPWM0_GRP0_CNT40
#define CH5_PCLK               PCLK_TCPWM0_CLOCKS40
#define CH5_PORT               GPIO_PRT12
#define CH5_PIN                4ul
#define CH5_MUX                P12_4_TCPWM0_TR_ONE_CNT_IN120
#define CH5_IRQ_SRC            tcpwm_0_interrupts_40_IRQn
#define CH5_CLK_DIV            DIV16_NO_WHEEL

/* ========================================================================== */
/*  轮速计算参数                                                               */
/* ========================================================================== */
#define GEAR_DIAMETER_MM       110u
#define GEAR_TEETH_NUM         100u
#define NUM_CAPTURE_CHANNELS   6u
#define NO_CAPTURE_TIMEOUT     5u
#define MIN_PERIOD_TICKS       500u

/* 捕获通道时钟目标频率 2MHz (分频值由 periph_divider 动态计算, 不硬编码源频) */
#define CAPTURE_CLK_TARGET_HZ   2000000ul

/* 轮速换算常量 — 输出单位 mm/s
 * 齿圈周长 C = π × D = 3.14159265 × 110 ≈ 345.575 mm
 *
 * 测周法 (低速, <10齿/10ms):  v = C × f / (N × period_ticks)
 *   = 345.575 × 2e6 / (100 × period_ticks)
 *   = 6911504 / period_ticks   [mm/s]
 *   (period_ticks = 相邻两上升沿的 2MHz 计数差)
 *
 * 计齿法 (高速, ≥10齿/10ms):  v = edges × C / (N × 0.01s)
 *   = edges × 345.575 / (100 × 0.01)
 *   = edges × 345.575          [mm/s]   (每齿 345.575 mm/s)
 *   定点: 345.575 × 10 ≈ 3456, 最后 /10
 */
#define PERIOD_TO_SPEED_MM      6911504ul  /* 测周法分子: mm/s */
#define TEETH_TO_SPEED_MM_x10   3456u      /* 每齿 mm/s ×10 (345.575 四舍五入) */

/* ========================================================================== */
/*  数据结构                                                                   */
/* ========================================================================== */

/* 单通道捕获数据 (ISR 写入, 主循环读取) */
typedef struct
{
    volatile uint16_t cc0;
    volatile uint16_t cc1;
    volatile uint16_t cc0_prev;
    volatile uint16_t rpm;
    volatile uint16_t edgeCount;
    volatile bool     newCapture;
    volatile uint8_t  noCaptureCount;

    uint16_t speed_buf[5];
    uint8_t  speed_buf_idx;
    bool     speed_buf_full;
} capture_ch_data_t;

/* 10ms ISR 快照: 原子性拷贝 volatile 数据, 供主循环计算 */
typedef struct {
    uint16_t cc0;
    uint16_t cc0_prev;
    uint16_t edgeCount;
    bool     newCapture;
} isr_snapshot_t;

/* ========================================================================== */
/*  全局变量 (供 10ms ISR 和主循环访问)                                         */
/* ========================================================================== */
extern capture_ch_data_t g_ch[NUM_CAPTURE_CHANNELS];
extern volatile isr_snapshot_t g_snapshot[NUM_CAPTURE_CHANNELS];

/* 滤波后轮速 (单位 mm/s), 由 WheelSpeed_CalcAndSend 填充, psw_data 读取 */
extern uint16_t g_wheel_speed_rpm[NUM_CAPTURE_CHANNELS];

/* ========================================================================== */
/*  对外 API                                                                   */
/* ========================================================================== */

/**
 * @brief   初始化六路 TCPWM 捕获通道 (时钟 + GPIO + 中断 + TCPWM)
 */
void WheelSpeed_Init(void);

/**
 * @brief   轮速计算 + 中值滤波 — 主循环每 10ms 调用
 *
 * @details 读取 g_snapshot 快照, 双模算法计算原始速度,
 *          5 点中值滤波后通过 CAN 发送六通道轮速报文。
 */
void WheelSpeed_CalcAndSend(void);

/**
 * @brief   整车参考车速: 六通道 rpm 中值
 * @return  车速 (mm/s)
 *
 * @note    用于传感器间隙诊断的速度门控, 避免单通道故障自我阻塞。
 */
uint16_t WheelSpeed_VehicleRefRpm(void);

#endif /* WHEEL_SPEED_H */
