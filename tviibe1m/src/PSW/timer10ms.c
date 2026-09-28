/**
 * @file    timer10ms.c
 * @brief   10ms 周期定时器实现
 *
 * @details 本模块处理作为系统心跳的 10ms 周期中断。
 *          ISR 快照六通道捕获数据、递增全局 systick 计数器，
 *          并置位 g_b10msTick 以通知主循环调度器。
 */

#include "cy_project.h"
#include "cy_device_headers.h"
#include "timer10ms.h"
#include "wheel_speed.h"
#include "bts724g.h"
#include "psw_data.h"
#include "peri_div.h"
#include "cmn.h"

/* ========================================================================== */
/*  10ms 周期定时器配置                                                         */
/*  实例: TCPWM0_GRP0_CNT1 (比较模式, 连续计数)                                */
/*  时钟: clk_peri → ÷N(动态) → 2MHz → DIVBY_2 → 1MHz                         */
/*  周期: 10000 - 1 = 9999,  9999+1 us = 10ms                                 */
/* ========================================================================== */
#define TIMER_10MS_CH          TCPWM0_GRP0_CNT1
#define TIMER_10MS_CLK_SRC     PCLK_TCPWM0_CLOCKS1
#define TIMER_10MS_IRQ_SRC     tcpwm_0_interrupts_1_IRQn
#define TIMER_10MS_CLK_DIV     DIV16_NO_TIMER10MS   /* 分频器 #1, 2MHz */
#define TIMER_10MS_TICKS       10000u          /* 10ms @ 1MHz */

volatile bool    g_b10msTick       = false;
volatile uint8_t g_timer10msCount  = 0u;

/* 任务计时: TCPWM 计数器快照 + 溢出计数 (0.1ms 单位, 对齐 PSW_EBS) */
static uint32_t s_task_start_counter = 0u;
static volatile uint32_t s_task_overflow_cnt = 0u;  /* 10ms TC 溢出次数 */
#define TIMER_10MS_AT_1MHZ    10000u          /* 10ms = 10000 ticks @ 1MHz */
#define TICKS_PER_01MS         100u           /* 0.1ms = 100 ticks @ 1MHz */

/* 10ms 定时器配置 */
static const cy_stc_tcpwm_counter_config_t timer10msCfg =
{
    .period            = 10000u - 1u,
    .clockPrescaler    = CY_TCPWM_PRESCALER_DIVBY_2,
    .runMode           = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection    = CY_TCPWM_COUNTER_COUNT_UP,
    .debug_pause       = 0u,
    .compareOrCapture  = CY_TCPWM_COUNTER_MODE_COMPARE,
    .compare0 = 0, .compare0_buff = 0, .compare1 = 0, .compare1_buff = 0,
    .enableCompare0Swap = false, .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_ON_TC,
    .capture0InputMode  = CY_TCPWM_INPUT_LEVEL,  .capture0Input = 0u,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,  .reloadInput = 0u,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,  .startInput  = 0u,
    .stopInputMode      = CY_TCPWM_INPUT_LEVEL,  .stopInput   = 0u,
    .capture1InputMode  = CY_TCPWM_INPUT_LEVEL,  .capture1Input = 0u,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,  .countInput   = 1u,
    .trigger0EventCfg   = CY_TCPWM_COUNTER_OVERFLOW,
    .trigger1EventCfg   = CY_TCPWM_COUNTER_OVERFLOW,
};

/* ========================================================================== */
/*  10ms 定时器 ISR — 优先级 1 (最高)                                          */
/*  职责: 快照六通道捕获数据 + 清零窗口计数器                                  */
/*        + systick 加 10 + 置位 g_b10msTick                                   */
/* ========================================================================== */
static void Timer10ms_Handler(void)
{
    Cy_Tcpwm_ClearInterrupt(TIMER_10MS_CH, CY_TCPWM_INT_ON_TC);

    /* 递增 1ms 节拍计数器 (10ms ISR 每次加 10, 供 bts724g 模块使用) */
    g_systick_ms += 10u;

    /* 快照六通道捕获数据并清零窗口计数器 */
    for (uint32_t i = 0u; i < NUM_CAPTURE_CHANNELS; i++)
    {
        g_snapshot[i].cc0        = g_ch[i].cc0;
        g_snapshot[i].cc0_prev   = g_ch[i].cc0_prev;
        g_snapshot[i].edgeCount  = g_ch[i].edgeCount;
        g_snapshot[i].newCapture = g_ch[i].newCapture;

        g_ch[i].edgeCount  = 0u;
        g_ch[i].newCapture = false;
    }

    g_timer10msCount++;
    s_task_overflow_cnt++;   /* 任务计时溢出计数 */
    g_b10msTick = true;
}

void Timer10ms_ISR(void)
{
    Timer10ms_Handler();
}

/* ========================================================================== */
/*  10ms 定时器初始化                                                           */
/* ========================================================================== */
void Timer10ms_Init(void)
{
    periph_divider(TIMER_10MS_CLK_SRC, CY_SYSCLK_DIV_16_BIT, TIMER_10MS_CLK_DIV, CAPTURE_CLK_TARGET_HZ);

    Cy_Tcpwm_Counter_Init(TIMER_10MS_CH, &timer10msCfg);
    Cy_Tcpwm_Counter_Enable(TIMER_10MS_CH);
    Cy_Tcpwm_TriggerStart(TIMER_10MS_CH);

    cy_stc_sysint_irq_t irqCfg = {.sysIntSrc = TIMER_10MS_IRQ_SRC, .intIdx = CPUIntIdx3_IRQn, .isEnabled = true};
    Cy_SysInt_InitIRQ(&irqCfg);
    Cy_SysInt_SetSystemIrqVector(irqCfg.sysIntSrc, Timer10ms_ISR);
    NVIC_SetPriority(irqCfg.intIdx, 1u);
    NVIC_ClearPendingIRQ(irqCfg.intIdx);
    NVIC_EnableIRQ(irqCfg.intIdx);
}

/* ========================================================================== */
/*  10ms 定时器计数 (对齐 PSW_EBS)                                             */
/* ========================================================================== */

uint8_t timer_get_count(void)
{
    return g_timer10msCount;
}

void timer_count_dev1(void)
{
    g_timer10msCount = 0u;
}

/* ========================================================================== */
/*  任务计时 (TCPWM 计数器快照, 对齐 PSW_EBS)                                   */
/*                                                                            */
/*  TCPWM 时钟 1MHz → 1μs/步, 0.1ms=100步                                     */
/* ========================================================================== */

void rcrd_task_start(void)
{
    s_task_start_counter = Cy_Tcpwm_Counter_GetCounter(TIMER_10MS_CH);
    s_task_overflow_cnt  = 0u;
}

void calc_task_time(void)
{
    uint32_t end_counter = Cy_Tcpwm_Counter_GetCounter(TIMER_10MS_CH);
    uint32_t overflow    = s_task_overflow_cnt;
    uint32_t elapsed;      /* 单位: μs (1MHz 计数器, 1 tick = 1μs) */
    uint32_t time_01ms;

    /* 先读溢出次数再读计数器 (与老代码 g_int_count 顺序一致)。
     * elapsed = 完整 10ms 周期数 × 10000 + 残余 tick。
     * 支持跨多个 10ms 周期, 不再退化为单次 wrap。 */
    if (end_counter >= s_task_start_counter)
    {
        elapsed = overflow * TIMER_10MS_TICKS + (end_counter - s_task_start_counter);
    }
    else
    {
        elapsed = (overflow - 1u) * TIMER_10MS_TICKS
                + (TIMER_10MS_TICKS - s_task_start_counter) + end_counter;
    }

    /* 换算 0.1ms: elapsed(μs) / 100; 饱和到 255 (PSWTaskTime 为 uint8_t,
     * 对外 RTETaskTime 亦为 uint8_t, 上限 25.5ms, 避免直接截断回绕) */
    time_01ms = elapsed / TICKS_PER_01MS;
    PSWTaskTime = (time_01ms > 255u) ? 255u : (uint8_t)time_01ms;
}
