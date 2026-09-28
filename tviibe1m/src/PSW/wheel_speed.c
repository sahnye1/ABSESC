/**
 * @file    wheel_speed.c
 * @brief   六通道轮速采集实现 — TCPWM 捕获 + 双模算法 + 中值滤波 + CAN 发送
 *
 * @details 六路独立捕获 ISR (优先级 2), 10ms ISR 快照数据,
 *          主循环调用 WheelSpeed_CalcAndSend() 完成计算和 CAN 发送。
 *
 *          CPUIntIdx 分配 (CM4 共 8 槽位, 0~7):
 *            CH0 → CPUIntIdx4, CH1 → CPUIntIdx5
 *            CH2 → CPUIntIdx6, CH3 → CPUIntIdx7
 *            CH4 → CPUIntIdx0, CH5 → CPUIntIdx1
 *            (CPUIntIdx2=10ms定时器, CPUIntIdx3=Crypto/IPC)
 */

#include "wheel_speed.h"
#include "can.h"
#include "cmn.h"
#include "RTE.h"
#include "cy_project.h"
#include "cy_device_headers.h"

/* ========================================================================== */
/*  全局变量                                                                   */
/* ========================================================================== */

capture_ch_data_t g_ch[NUM_CAPTURE_CHANNELS] = {0};
uint16_t g_wheel_speed_rpm[NUM_CAPTURE_CHANNELS] = {0u};
volatile isr_snapshot_t   g_snapshot[NUM_CAPTURE_CHANNELS] = {0};

/* ========================================================================== */
/*  CH0: P8.2,  TR_ONE_CNT_IN63,  TCPWM0_GRP0_CNT21 — XL (附加桥左) */
/* ========================================================================== */

static const cy_stc_tcpwm_counter_config_t captureCh0Cfg =
{
    .period             = 0xFFFFul,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1,
    .runMode            = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection     = CY_TCPWM_COUNTER_COUNT_UP,
    .debug_pause        = false,
    .compareOrCapture   = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .compare0           = 0ul,
    .compare0_buff      = 0ul,
    .compare1           = 0ul,
    .compare1_buff      = 0ul,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .capture0InputMode  = CY_TCPWM_INPUT_RISING_EDGE,
    .capture0Input      = 2ul,
    .capture1InputMode  = CY_TCPWM_INPUT_FALLING_EDGE,
    .capture1Input      = 2ul,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,
    .reloadInput        = 0ul,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,
    .startInput         = 0ul,
    .stopInputMode      = CY_TCPWM_INPUT_LEVEL,
    .stopInput          = 0ul,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1ul,
    .trigger0EventCfg   = CY_TCPWM_COUNTER_CC0_MATCH,
    .trigger1EventCfg   = CY_TCPWM_COUNTER_CC1_MATCH,
};

static const cy_stc_gpio_pin_config_t captureCh0_pinCfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom     = CH0_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

/* ========================================================================== */
/*  CH1: P12.0, TR_ONE_CNT_IN108, TCPWM0_GRP0_CNT36 — FL (前桥左) */
/* ========================================================================== */

static const cy_stc_tcpwm_counter_config_t captureCh1Cfg =
{
    .period             = 0xFFFFul,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1,
    .runMode            = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection     = CY_TCPWM_COUNTER_COUNT_UP,
    .debug_pause        = false,
    .compareOrCapture   = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .compare0           = 0ul,
    .compare0_buff      = 0ul,
    .compare1           = 0ul,
    .compare1_buff      = 0ul,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .capture0InputMode  = CY_TCPWM_INPUT_RISING_EDGE,
    .capture0Input      = 2ul,
    .capture1InputMode  = CY_TCPWM_INPUT_FALLING_EDGE,
    .capture1Input      = 2ul,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,
    .reloadInput        = 0ul,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,
    .startInput         = 0ul,
    .stopInputMode      = CY_TCPWM_INPUT_LEVEL,
    .stopInput          = 0ul,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1ul,
    .trigger0EventCfg   = CY_TCPWM_COUNTER_CC0_MATCH,
    .trigger1EventCfg   = CY_TCPWM_COUNTER_CC1_MATCH,
};

static const cy_stc_gpio_pin_config_t captureCh1_pinCfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom     = CH1_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

/* ========================================================================== */
/*  CH2: XR (附加桥右) 轮速捕获 — 引脚见 hw_rev.h (P12.1/CNT37 或 P8.1/CNT20) */
/* ========================================================================== */

static const cy_stc_tcpwm_counter_config_t captureCh2Cfg =
{
    .period             = 0xFFFFul,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1,
    .runMode            = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection     = CY_TCPWM_COUNTER_COUNT_UP,
    .debug_pause        = false,
    .compareOrCapture   = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .compare0           = 0ul,
    .compare0_buff      = 0ul,
    .compare1           = 0ul,
    .compare1_buff      = 0ul,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .capture0InputMode  = CY_TCPWM_INPUT_RISING_EDGE,
    .capture0Input      = 2ul,
    .capture1InputMode  = CY_TCPWM_INPUT_FALLING_EDGE,
    .capture1Input      = 2ul,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,
    .reloadInput        = 0ul,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,
    .startInput         = 0ul,
    .stopInputMode      = CY_TCPWM_INPUT_LEVEL,
    .stopInput          = 0ul,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1ul,
    .trigger0EventCfg   = CY_TCPWM_COUNTER_CC0_MATCH,
    .trigger1EventCfg   = CY_TCPWM_COUNTER_CC1_MATCH,
};

static const cy_stc_gpio_pin_config_t captureCh2_pinCfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom     = CH2_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

/* ========================================================================== */
/*  CH3: P12.2, TR_ONE_CNT_IN114, TCPWM0_GRP0_CNT38 — RR (后桥右) */
/* ========================================================================== */

static const cy_stc_tcpwm_counter_config_t captureCh3Cfg =
{
    .period             = 0xFFFFul,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1,
    .runMode            = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection     = CY_TCPWM_COUNTER_COUNT_UP,
    .debug_pause        = false,
    .compareOrCapture   = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .compare0           = 0ul,
    .compare0_buff      = 0ul,
    .compare1           = 0ul,
    .compare1_buff      = 0ul,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .capture0InputMode  = CY_TCPWM_INPUT_RISING_EDGE,
    .capture0Input      = 2ul,
    .capture1InputMode  = CY_TCPWM_INPUT_FALLING_EDGE,
    .capture1Input      = 2ul,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,
    .reloadInput        = 0ul,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,
    .startInput         = 0ul,
    .stopInputMode      = CY_TCPWM_INPUT_LEVEL,
    .stopInput          = 0ul,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1ul,
    .trigger0EventCfg   = CY_TCPWM_COUNTER_CC0_MATCH,
    .trigger1EventCfg   = CY_TCPWM_COUNTER_CC1_MATCH,
};

static const cy_stc_gpio_pin_config_t captureCh3_pinCfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom     = CH3_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

/* ========================================================================== */
/*  CH4: P12.3, TR_ONE_CNT_IN117, TCPWM0_GRP0_CNT39 — RL (后桥左) */
/* ========================================================================== */

static const cy_stc_tcpwm_counter_config_t captureCh4Cfg =
{
    .period             = 0xFFFFul,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1,
    .runMode            = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection     = CY_TCPWM_COUNTER_COUNT_UP,
    .debug_pause        = false,
    .compareOrCapture   = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .compare0           = 0ul,
    .compare0_buff      = 0ul,
    .compare1           = 0ul,
    .compare1_buff      = 0ul,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .capture0InputMode  = CY_TCPWM_INPUT_RISING_EDGE,
    .capture0Input      = 2ul,
    .capture1InputMode  = CY_TCPWM_INPUT_FALLING_EDGE,
    .capture1Input      = 2ul,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,
    .reloadInput        = 0ul,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,
    .startInput         = 0ul,
    .stopInputMode      = CY_TCPWM_INPUT_LEVEL,
    .stopInput          = 0ul,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1ul,
    .trigger0EventCfg   = CY_TCPWM_COUNTER_CC0_MATCH,
    .trigger1EventCfg   = CY_TCPWM_COUNTER_CC1_MATCH,
};

static const cy_stc_gpio_pin_config_t captureCh4_pinCfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom     = CH4_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

/* ========================================================================== */
/*  CH5: P12.4, TR_ONE_CNT_IN120, TCPWM0_GRP0_CNT40 — FR (前桥右) */
/* ========================================================================== */

static const cy_stc_tcpwm_counter_config_t captureCh5Cfg =
{
    .period             = 0xFFFFul,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1,
    .runMode            = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection     = CY_TCPWM_COUNTER_COUNT_UP,
    .debug_pause        = false,
    .compareOrCapture   = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .compare0           = 0ul,
    .compare0_buff      = 0ul,
    .compare1           = 0ul,
    .compare1_buff      = 0ul,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .capture0InputMode  = CY_TCPWM_INPUT_RISING_EDGE,
    .capture0Input      = 2ul,
    .capture1InputMode  = CY_TCPWM_INPUT_FALLING_EDGE,
    .capture1Input      = 2ul,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,
    .reloadInput        = 0ul,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,
    .startInput         = 0ul,
    .stopInputMode      = CY_TCPWM_INPUT_LEVEL,
    .stopInput          = 0ul,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1ul,
    .trigger0EventCfg   = CY_TCPWM_COUNTER_CC0_MATCH,
    .trigger1EventCfg   = CY_TCPWM_COUNTER_CC1_MATCH,
};

static const cy_stc_gpio_pin_config_t captureCh5_pinCfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom     = CH5_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

/* ========================================================================== */
/*  六路独立 TCPWM 捕获 ISR (优先级 2)                                         */
/*  职责: 仅存储原始时间戳, 不做计算, 不发 CAN                                 */
/* ========================================================================== */

void Capture_ISR_Ch0(void)
{
    if (Cy_Tcpwm_Counter_GetCC0_IntrMasked(CH0_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC0_Intr(CH0_TCPWM);
        g_ch[0].cc0_prev = g_ch[0].cc0;
        g_ch[0].cc0      = (uint16_t)Cy_Tcpwm_Counter_GetCompare0(CH0_TCPWM);
        g_ch[0].newCapture = true;
        g_ch[0].edgeCount++;
    }
    if (Cy_Tcpwm_Counter_GetCC1_IntrMasked(CH0_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC1_Intr(CH0_TCPWM);
        g_ch[0].cc1 = (uint16_t)Cy_Tcpwm_Counter_GetCompare1(CH0_TCPWM);
    }
}

void Capture_ISR_Ch1(void)
{
    if (Cy_Tcpwm_Counter_GetCC0_IntrMasked(CH1_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC0_Intr(CH1_TCPWM);
        g_ch[1].cc0_prev = g_ch[1].cc0;
        g_ch[1].cc0      = (uint16_t)Cy_Tcpwm_Counter_GetCompare0(CH1_TCPWM);
        g_ch[1].newCapture = true;
        g_ch[1].edgeCount++;
    }
    if (Cy_Tcpwm_Counter_GetCC1_IntrMasked(CH1_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC1_Intr(CH1_TCPWM);
        g_ch[1].cc1 = (uint16_t)Cy_Tcpwm_Counter_GetCompare1(CH1_TCPWM);
    }
}

void Capture_ISR_Ch2(void)
{
    if (Cy_Tcpwm_Counter_GetCC0_IntrMasked(CH2_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC0_Intr(CH2_TCPWM);
        g_ch[2].cc0_prev = g_ch[2].cc0;
        g_ch[2].cc0      = (uint16_t)Cy_Tcpwm_Counter_GetCompare0(CH2_TCPWM);
        g_ch[2].newCapture = true;
        g_ch[2].edgeCount++;
    }
    if (Cy_Tcpwm_Counter_GetCC1_IntrMasked(CH2_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC1_Intr(CH2_TCPWM);
        g_ch[2].cc1 = (uint16_t)Cy_Tcpwm_Counter_GetCompare1(CH2_TCPWM);
    }
}

void Capture_ISR_Ch3(void)
{
    if (Cy_Tcpwm_Counter_GetCC0_IntrMasked(CH3_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC0_Intr(CH3_TCPWM);
        g_ch[3].cc0_prev = g_ch[3].cc0;
        g_ch[3].cc0      = (uint16_t)Cy_Tcpwm_Counter_GetCompare0(CH3_TCPWM);
        g_ch[3].newCapture = true;
        g_ch[3].edgeCount++;
    }
    if (Cy_Tcpwm_Counter_GetCC1_IntrMasked(CH3_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC1_Intr(CH3_TCPWM);
        g_ch[3].cc1 = (uint16_t)Cy_Tcpwm_Counter_GetCompare1(CH3_TCPWM);
    }
}

void Capture_ISR_Ch4(void)
{
    if (Cy_Tcpwm_Counter_GetCC0_IntrMasked(CH4_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC0_Intr(CH4_TCPWM);
        g_ch[4].cc0_prev = g_ch[4].cc0;
        g_ch[4].cc0      = (uint16_t)Cy_Tcpwm_Counter_GetCompare0(CH4_TCPWM);
        g_ch[4].newCapture = true;
        g_ch[4].edgeCount++;
    }
    if (Cy_Tcpwm_Counter_GetCC1_IntrMasked(CH4_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC1_Intr(CH4_TCPWM);
        g_ch[4].cc1 = (uint16_t)Cy_Tcpwm_Counter_GetCompare1(CH4_TCPWM);
    }
}

void Capture_ISR_Ch5(void)
{
    if (Cy_Tcpwm_Counter_GetCC0_IntrMasked(CH5_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC0_Intr(CH5_TCPWM);
        g_ch[5].cc0_prev = g_ch[5].cc0;
        g_ch[5].cc0      = (uint16_t)Cy_Tcpwm_Counter_GetCompare0(CH5_TCPWM);
        g_ch[5].newCapture = true;
        g_ch[5].edgeCount++;
    }
    if (Cy_Tcpwm_Counter_GetCC1_IntrMasked(CH5_TCPWM))
    {
        Cy_Tcpwm_Counter_ClearCC1_Intr(CH5_TCPWM);
        g_ch[5].cc1 = (uint16_t)Cy_Tcpwm_Counter_GetCompare1(CH5_TCPWM);
    }
}

/* ========================================================================== */
/*  六路独立捕获通道初始化                                                     */
/* ========================================================================== */

static void Capture_Ch0_Init(void)
{
    periph_divider(CH0_PCLK, CY_SYSCLK_DIV_16_BIT, CH0_CLK_DIV, CAPTURE_CLK_TARGET_HZ);

    cy_stc_sysint_irq_t irqCfg = {.sysIntSrc = CH0_IRQ_SRC, .intIdx = CPUIntIdx6_IRQn, .isEnabled = true};
    Cy_SysInt_InitIRQ(&irqCfg);
    Cy_SysInt_SetSystemIrqVector(irqCfg.sysIntSrc, Capture_ISR_Ch0);
    NVIC_SetPriority(irqCfg.intIdx, 2ul);
    NVIC_ClearPendingIRQ(irqCfg.intIdx);
    NVIC_EnableIRQ(irqCfg.intIdx);

    Cy_GPIO_Pin_Init(CH0_PORT, CH0_PIN, &captureCh0_pinCfg);
    Cy_Tcpwm_Counter_Init(CH0_TCPWM, &captureCh0Cfg);
    Cy_Tcpwm_Counter_SetCC0_IntrMask(CH0_TCPWM);
    Cy_Tcpwm_Counter_SetCC1_IntrMask(CH0_TCPWM);
    Cy_Tcpwm_Counter_Enable(CH0_TCPWM);
    Cy_Tcpwm_TriggerStart(CH0_TCPWM);
}

static void Capture_Ch1_Init(void)
{
    periph_divider(CH1_PCLK, CY_SYSCLK_DIV_16_BIT, CH1_CLK_DIV, CAPTURE_CLK_TARGET_HZ);

    cy_stc_sysint_irq_t irqCfg = {.sysIntSrc = CH1_IRQ_SRC, .intIdx = CPUIntIdx6_IRQn, .isEnabled = true};
    Cy_SysInt_InitIRQ(&irqCfg);
    Cy_SysInt_SetSystemIrqVector(irqCfg.sysIntSrc, Capture_ISR_Ch1);
    NVIC_SetPriority(irqCfg.intIdx, 2ul);
    NVIC_ClearPendingIRQ(irqCfg.intIdx);
    NVIC_EnableIRQ(irqCfg.intIdx);

    Cy_GPIO_Pin_Init(CH1_PORT, CH1_PIN, &captureCh1_pinCfg);
    Cy_Tcpwm_Counter_Init(CH1_TCPWM, &captureCh1Cfg);
    Cy_Tcpwm_Counter_SetCC0_IntrMask(CH1_TCPWM);
    Cy_Tcpwm_Counter_SetCC1_IntrMask(CH1_TCPWM);
    Cy_Tcpwm_Counter_Enable(CH1_TCPWM);
    Cy_Tcpwm_TriggerStart(CH1_TCPWM);
}

static void Capture_Ch2_Init(void)
{
    periph_divider(CH2_PCLK, CY_SYSCLK_DIV_16_BIT, CH2_CLK_DIV, CAPTURE_CLK_TARGET_HZ);

    cy_stc_sysint_irq_t irqCfg = {.sysIntSrc = CH2_IRQ_SRC, .intIdx = CPUIntIdx6_IRQn, .isEnabled = true};
    Cy_SysInt_InitIRQ(&irqCfg);
    Cy_SysInt_SetSystemIrqVector(irqCfg.sysIntSrc, Capture_ISR_Ch2);
    NVIC_SetPriority(irqCfg.intIdx, 2ul);
    NVIC_ClearPendingIRQ(irqCfg.intIdx);
    NVIC_EnableIRQ(irqCfg.intIdx);

    Cy_GPIO_Pin_Init(CH2_PORT, CH2_PIN, &captureCh2_pinCfg);
    Cy_Tcpwm_Counter_Init(CH2_TCPWM, &captureCh2Cfg);
    Cy_Tcpwm_Counter_SetCC0_IntrMask(CH2_TCPWM);
    Cy_Tcpwm_Counter_SetCC1_IntrMask(CH2_TCPWM);
    Cy_Tcpwm_Counter_Enable(CH2_TCPWM);
    Cy_Tcpwm_TriggerStart(CH2_TCPWM);
}

static void Capture_Ch3_Init(void)
{
    periph_divider(CH3_PCLK, CY_SYSCLK_DIV_16_BIT, CH3_CLK_DIV, CAPTURE_CLK_TARGET_HZ);

    cy_stc_sysint_irq_t irqCfg = {.sysIntSrc = CH3_IRQ_SRC, .intIdx = CPUIntIdx6_IRQn, .isEnabled = true};
    Cy_SysInt_InitIRQ(&irqCfg);
    Cy_SysInt_SetSystemIrqVector(irqCfg.sysIntSrc, Capture_ISR_Ch3);
    NVIC_SetPriority(irqCfg.intIdx, 2ul);
    NVIC_ClearPendingIRQ(irqCfg.intIdx);
    NVIC_EnableIRQ(irqCfg.intIdx);

    Cy_GPIO_Pin_Init(CH3_PORT, CH3_PIN, &captureCh3_pinCfg);
    Cy_Tcpwm_Counter_Init(CH3_TCPWM, &captureCh3Cfg);
    Cy_Tcpwm_Counter_SetCC0_IntrMask(CH3_TCPWM);
    Cy_Tcpwm_Counter_SetCC1_IntrMask(CH3_TCPWM);
    Cy_Tcpwm_Counter_Enable(CH3_TCPWM);
    Cy_Tcpwm_TriggerStart(CH3_TCPWM);
}

static void Capture_Ch4_Init(void)
{
    periph_divider(CH4_PCLK, CY_SYSCLK_DIV_16_BIT, CH4_CLK_DIV, CAPTURE_CLK_TARGET_HZ);

    cy_stc_sysint_irq_t irqCfg = {.sysIntSrc = CH4_IRQ_SRC, .intIdx = CPUIntIdx6_IRQn, .isEnabled = true};
    Cy_SysInt_InitIRQ(&irqCfg);
    Cy_SysInt_SetSystemIrqVector(irqCfg.sysIntSrc, Capture_ISR_Ch4);
    NVIC_SetPriority(irqCfg.intIdx, 2ul);
    NVIC_ClearPendingIRQ(irqCfg.intIdx);
    NVIC_EnableIRQ(irqCfg.intIdx);

    Cy_GPIO_Pin_Init(CH4_PORT, CH4_PIN, &captureCh4_pinCfg);
    Cy_Tcpwm_Counter_Init(CH4_TCPWM, &captureCh4Cfg);
    Cy_Tcpwm_Counter_SetCC0_IntrMask(CH4_TCPWM);
    Cy_Tcpwm_Counter_SetCC1_IntrMask(CH4_TCPWM);
    Cy_Tcpwm_Counter_Enable(CH4_TCPWM);
    Cy_Tcpwm_TriggerStart(CH4_TCPWM);
}

static void Capture_Ch5_Init(void)
{
    periph_divider(CH5_PCLK, CY_SYSCLK_DIV_16_BIT, CH5_CLK_DIV, CAPTURE_CLK_TARGET_HZ);

    cy_stc_sysint_irq_t irqCfg = {.sysIntSrc = CH5_IRQ_SRC, .intIdx = CPUIntIdx6_IRQn, .isEnabled = true};
    Cy_SysInt_InitIRQ(&irqCfg);
    Cy_SysInt_SetSystemIrqVector(irqCfg.sysIntSrc, Capture_ISR_Ch5);
    NVIC_SetPriority(irqCfg.intIdx, 2ul);
    NVIC_ClearPendingIRQ(irqCfg.intIdx);
    NVIC_EnableIRQ(irqCfg.intIdx);

    Cy_GPIO_Pin_Init(CH5_PORT, CH5_PIN, &captureCh5_pinCfg);
    Cy_Tcpwm_Counter_Init(CH5_TCPWM, &captureCh5Cfg);
    Cy_Tcpwm_Counter_SetCC0_IntrMask(CH5_TCPWM);
    Cy_Tcpwm_Counter_SetCC1_IntrMask(CH5_TCPWM);
    Cy_Tcpwm_Counter_Enable(CH5_TCPWM);
    Cy_Tcpwm_TriggerStart(CH5_TCPWM);
}

/* ========================================================================== */
/*  轮速计算 + 中值滤波 + CAN 发送 — 从主循环调用                             */
/* ========================================================================== */
void WheelSpeed_CalcAndSend(void)
{
    uint16_t raw_speed[NUM_CAPTURE_CHANNELS];
    uint16_t final_speed[NUM_CAPTURE_CHANNELS];

    /* 第一步: 逐通道计算原始速度 */
    for (uint32_t i = 0u; i < NUM_CAPTURE_CHANNELS; i++)
    {
        uint16_t edges = g_snapshot[i].edgeCount;

        if (g_snapshot[i].newCapture && edges > 0 && edges < 10)
        {
            /* 测周法: 低速 (<10 齿/10ms) */
            g_ch[i].noCaptureCount = 0u;

            uint16_t period_ticks;
            if (g_snapshot[i].cc0 >= g_snapshot[i].cc0_prev)
                period_ticks = g_snapshot[i].cc0 - g_snapshot[i].cc0_prev;
            else
                period_ticks = (0xFFFFu - g_snapshot[i].cc0_prev) + g_snapshot[i].cc0 + 1u;

            if (period_ticks >= MIN_PERIOD_TICKS && period_ticks < 0xFFF0u)
                raw_speed[i] = (uint16_t)((PERIOD_TO_SPEED_MM + (uint32_t)period_ticks / 2u) / (uint32_t)period_ticks);
            else
                raw_speed[i] = 0u;
        }
        else if (edges >= 10)
        {
            /* 计齿法: 高速 (≥10 齿/10ms) */
            g_ch[i].noCaptureCount = 0u;
            raw_speed[i] = (uint16_t)(((uint32_t)edges * TEETH_TO_SPEED_MM_x10 + 5u) / 10u);
        }
        else
        {
            /* 无捕获: 超时判零 */
            g_ch[i].noCaptureCount++;
            if (g_ch[i].noCaptureCount > NO_CAPTURE_TIMEOUT)
                raw_speed[i] = 0u;
            else
                raw_speed[i] = g_ch[i].rpm;
        }
    }

    /* 第二步: 中值滤波 (5 点缓冲) */
    for (uint32_t i = 0u; i < NUM_CAPTURE_CHANNELS; i++)
    {
        g_ch[i].speed_buf[g_ch[i].speed_buf_idx] = raw_speed[i];
        g_ch[i].speed_buf_idx = (g_ch[i].speed_buf_idx + 1u) % 5u;

        if (!g_ch[i].speed_buf_full)
        {
            static uint8_t buf_fill[NUM_CAPTURE_CHANNELS] = {0};
            buf_fill[i]++;
            if (buf_fill[i] >= 5u)
                g_ch[i].speed_buf_full = true;
        }

        if (!g_ch[i].speed_buf_full)
        {
            final_speed[i] = raw_speed[i];
            g_ch[i].rpm    = raw_speed[i];
            continue;
        }

        uint16_t sorted[5] = {g_ch[i].speed_buf[0], g_ch[i].speed_buf[1],
                              g_ch[i].speed_buf[2], g_ch[i].speed_buf[3],
                              g_ch[i].speed_buf[4]};
        for (int j = 0; j < 4; j++)
            for (int k = j + 1; k < 5; k++)
                if (sorted[j] > sorted[k])
                    { uint16_t tmp = sorted[j]; sorted[j] = sorted[k]; sorted[k] = tmp; }

        final_speed[i] = sorted[2];
        g_ch[i].rpm    = sorted[2];
    }

    /* 第三步: 暴露滤波后速度给 psw_data (单位已是 mm/s) */
    for (uint8_t i = 0u; i < NUM_CAPTURE_CHANNELS; i++)
    {
        g_wheel_speed_rpm[i] = final_speed[i];
    }

#if 0
    /* 帧 0x80: FL+FR+RL+RR (小端, 8 字节) */
    {
        uint8_t can_data[8] = {0};
        can_data[0] = (uint8_t)(final_speed[1] & 0xFFu); can_data[1] = (uint8_t)(final_speed[1] >> 8); /* FL */
        can_data[2] = (uint8_t)(final_speed[5] & 0xFFu); can_data[3] = (uint8_t)(final_speed[5] >> 8); /* FR */
        can_data[4] = (uint8_t)(final_speed[4] & 0xFFu); can_data[5] = (uint8_t)(final_speed[4] >> 8); /* RL */
        can_data[6] = (uint8_t)(final_speed[3] & 0xFFu); can_data[7] = (uint8_t)(final_speed[3] >> 8); /* RR */
        can0_sendMsg(0x80u, can_data, 8u);
    }
    /* 帧 0x81: XL+XR (小端, 8 字节, 后 4 填 0) */
    {
        uint8_t can_data[8] = {0};
        can_data[0] = (uint8_t)(final_speed[0] & 0xFFu); can_data[1] = (uint8_t)(final_speed[0] >> 8); /* XL */
        can_data[2] = (uint8_t)(final_speed[2] & 0xFFu); can_data[3] = (uint8_t)(final_speed[2] >> 8); /* XR */
        can0_sendMsg(0x81u, can_data, 8u);
    }
    /* 帧 0x82: CH0~CH3 edgeCount 调试 */
    {
        uint8_t can_data[8] = {0};
        can_data[0] = (uint8_t)(g_ch[0].edgeCount & 0xFFu); can_data[1] = (uint8_t)(g_ch[0].edgeCount >> 8); /* XL */
        can_data[2] = (uint8_t)(g_ch[1].edgeCount & 0xFFu); can_data[3] = (uint8_t)(g_ch[1].edgeCount >> 8); /* FL */
        //can_data[4] = (uint8_t)(g_ch[2].edgeCount & 0xFFu); can_data[5] = (uint8_t)(g_ch[2].edgeCount >> 8); /* XR */
        can_data[6] = (uint8_t)(g_ch[3].edgeCount & 0xFFu); can_data[7] = (uint8_t)(g_ch[3].edgeCount >> 8); /* RR */
        /* 临时调试：读 CNT20 计数器，看它是否在递增 */
        can_data[4] = (uint8_t)(Cy_Tcpwm_Counter_GetCounter(TCPWM0_GRP0_CNT20) & 0xFFu);
        can_data[5] = (uint8_t)(Cy_Tcpwm_Counter_GetCounter(TCPWM0_GRP0_CNT20) >> 8);

        can0_sendMsg(0x82u, can_data, 8u);
    }
    /* 帧 0x83: CH4+CH5 edgeCount 调试 */
    {
        uint8_t can_data[8] = {0};
        can_data[0] = (uint8_t)(g_ch[4].edgeCount & 0xFFu); can_data[1] = (uint8_t)(g_ch[4].edgeCount >> 8); /* RL */
        can_data[2] = (uint8_t)(g_ch[5].edgeCount & 0xFFu); can_data[3] = (uint8_t)(g_ch[5].edgeCount >> 8); /* FR */
        can0_sendMsg(0x83u, can_data, 8u);
    }
#endif
    
}

/* ========================================================================== */
/*  VehicleRefRpm — 六通道中值参考轮速 (sen
sor_diag 间隙检测门控用)             */
/* ========================================================================== */
uint16_t WheelSpeed_VehicleRefRpm(void)
{
    /* 取六通道中值作为参考车速 */
    uint16_t sorted[6] = {
        g_wheel_speed_rpm[0], g_wheel_speed_rpm[1], g_wheel_speed_rpm[2],
        g_wheel_speed_rpm[3], g_wheel_speed_rpm[4], g_wheel_speed_rpm[5]
    };
    for (uint8_t j = 0u; j < 5u; j++)
        for (uint8_t k = j + 1u; k < 6u; k++)
            if (sorted[j] > sorted[k])
                { uint16_t tmp = sorted[j]; sorted[j] = sorted[k]; sorted[k] = tmp; }
    return (uint16_t)(((uint32_t)sorted[2] + sorted[3]) / 2u);
}

/* ========================================================================== */
/*  初始化入口: 六通道捕获统一初始化                                           */
/* ========================================================================== */
void WheelSpeed_Init(void)
{
    Capture_Ch0_Init();
    Capture_Ch1_Init();
    Capture_Ch2_Init();
    Capture_Ch3_Init();
    Capture_Ch4_Init();
    Capture_Ch5_Init();
}
