/**
 * @file    sar1_adc.c
 * @brief   SAR1 ADC 模拟信号采集驱动 — 压力传感器 + VPOWER 电压
 *
 * @details SAR1 (PASS0_SAR1) SARMUX0 三通道 Group 自动序列, 软件触发/轮询。
 *          CH0=AN13(P7.5, 前桥压力), CH1=AN1(P6.1, 后桥压力), CH2=AN10(P7.2, VPOWER, 组尾)。
 *          每轮转换后计算电压/压力/故障/报警。
 */

#include "sar1_adc.h"
#include "psw_data.h"
#include "cy_project.h"
#include "cy_device_headers.h"

/* ========================================================================== */
/*  内部状态                                                                   */
/* ========================================================================== */

/* 压力故障连续计数 (双层窗口 + 对称消抖) */
static uint8_t s_front_err_cnt = 0u;
static uint8_t s_front_ok_cnt  = 0u;
static uint8_t s_rear_err_cnt  = 0u;
static uint8_t s_rear_ok_cnt   = 0u;

/* 最新一轮的 ADC 原始值 & 换算结果 */
static uint16_t s_raw_front = 0u;
static uint16_t s_raw_rear  = 0u;
static uint16_t s_kpa_front = 0u;
static uint16_t s_kpa_rear  = 0u;

static bool s_front_fault = false;
static bool s_rear_fault  = false;

/* VPOWER 电压检测状态 */
static uint16_t s_raw_vpwr  = 0u;
static uint16_t s_mv_vpwr   = 0u;
static uint16_t s_vpwr_x10  = 0u;     /* fact:0.1V, 如 245=24.5V */
static uint8_t  s_vpwr_high_cnt    = 0u;
static uint8_t  s_vpwr_high_ok_cnt = 0u;
static uint8_t  s_vpwr_low_cnt     = 0u;
static uint8_t  s_vpwr_low_ok_cnt  = 0u;
static bool     s_vpwr_high_alarm  = false;
static bool     s_vpwr_low_alarm   = false;

/* ========================================================================== */
/*  初始化 SAR1 ADC                                                            */
/* ========================================================================== */
void Sar1Adc_Init(void)
{
    /* ---- 时钟: 动态读取 clk_peri, 分频到 ≤26.67MHz ---- */
    uint32_t periFreq = 0u;
    Cy_SysClk_GetClkPeriFrequency(&periFreq);
    uint32_t adcMaxFreq = 26670000ul;
    uint32_t divNum = (periFreq + adcMaxFreq / 2u) / adcMaxFreq;

    Cy_SysClk_PeriphAssignDivider(SAR1_ADC_PCLK, CY_SYSCLK_DIV_16_BIT, SAR1_ADC_CLK_DIV);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, SAR1_ADC_CLK_DIV, (divNum - 1ul));
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, SAR1_ADC_CLK_DIV);

    /* 采样时间: ceil(412ns × f_adc) × 5 */
    uint32_t actualAdcFreq = periFreq / divNum;
    uint32_t samplingCycle = (412ull * actualAdcFreq + 500000000ull) / 1000000000ull;
    if (samplingCycle < 2u) samplingCycle = 2u;
    samplingCycle *= 5u;

    /* ---- SAR0 已由 sensor_diag 初始化 (adcConfig 一致), 此处仅配通道和引脚, 不再重复 Cy_Adc_Init ---- */

    /* ---- 引脚配置: P7.5(前桥) + P6.1(后桥) + P7.2(VPOWER), SARMUX 直连模拟模式 ---- */
    {
        cy_stc_gpio_pin_config_t adcPinCfg =
        {
            .outVal = 0ul, .driveMode = CY_GPIO_DM_ANALOG,
            .intEdge = 0ul, .intMask = 0ul, .vtrip = 0ul, .slewRate = 0ul, .driveSel = 0ul,
        };
        adcPinCfg.hsiom = SAR1_CH_FRONT_MUX;
        Cy_GPIO_Pin_Init(SAR1_CH_FRONT_PORT, SAR1_CH_FRONT_PIN, &adcPinCfg);

        adcPinCfg.hsiom = SAR1_CH_REAR_MUX;
        Cy_GPIO_Pin_Init(SAR1_CH_REAR_PORT, SAR1_CH_REAR_PIN, &adcPinCfg);

        adcPinCfg.hsiom = SAR1_CH_VPWR_MUX;
        Cy_GPIO_Pin_Init(SAR1_CH_VPWR_PORT, SAR1_CH_VPWR_PIN, &adcPinCfg);
    }

    /* ---- 三通道配置: CH0=AN13(前桥), CH1=AN1(后桥), CH2=AN10(VPOWER, 组尾) ----
     * (SAR1 独立 group, 组头 CH0 触发, 组尾 CH2) */
    {
        const cy_stc_adc_channel_config_t adcChBase =
        {
            .triggerSelection       = CY_ADC_TRIGGER_OFF,
            .channelPriority        = 0u,
            .preenptionType         = CY_ADC_PREEMPTION_FINISH_RESUME,
            .isGroupEnd             = false,
            .doneLevel              = CY_ADC_DONE_LEVEL_PULSE,
            .extMuxSelect           = 0u, .extMuxEnable = true,   /* 对齐老代码 adc.c: extMuxEnable=true */
            .preconditionMode       = CY_ADC_PRECONDITION_MODE_OFF,
            .overlapDiagMode        = CY_ADC_OVERLAP_DIAG_MODE_OFF,
            .sampleTime             = samplingCycle,
            .calibrationValueSelect = CY_ADC_CALIBRATION_VALUE_REGULAR,
            .postProcessingMode     = CY_ADC_POST_PROCESSING_MODE_NONE,
            .resultAlignment        = CY_ADC_RESULT_ALIGNMENT_RIGHT,
            .signExtention          = CY_ADC_SIGN_EXTENTION_UNSIGNED,
            .averageCount           = 0u, .rightShift = 0u,
            .rangeDetectionMode     = CY_ADC_RANGE_DETECTION_MODE_INSIDE_RANGE,
            .rangeDetectionLoThreshold = 0x0000u, .rangeDetectionHiThreshold = 0x0FFFu,
            .mask.grpDone      = false, .mask.grpCancelled = false, .mask.grpOverflow = false,
            .mask.chRange      = false, .mask.chPulse     = false, .mask.chOverflow  = false,
        };

        /* CH0: AN13 (P7.5) — 前桥 */
        cy_stc_adc_channel_config_t chCfg = adcChBase;
        chCfg.pinAddress = CY_ADC_PIN_ADDRESS_AN13;
        chCfg.portAddress = CY_ADC_PORT_ADDRESS_SARMUX0;
        Cy_Adc_Channel_Init(&SAR1_ADC_MACRO->CH[SAR1_CH_FRONT], &chCfg);

        /* CH1: AN1 (P6.1) — 后桥 */
        chCfg.pinAddress  = CY_ADC_PIN_ADDRESS_AN1;
        chCfg.portAddress  = CY_ADC_PORT_ADDRESS_SARMUX0;
        Cy_Adc_Channel_Init(&SAR1_ADC_MACRO->CH[SAR1_CH_REAR], &chCfg);

        /* CH2: AN10 (P7.2) — VPOWER, 组尾, SARMUX 直连 (与轮速/压力一致, extMux 关闭) */
        chCfg.pinAddress  = CY_ADC_PIN_ADDRESS_AN10;
        chCfg.portAddress  = CY_ADC_PORT_ADDRESS_SARMUX0;
        chCfg.isGroupEnd   = true;
        chCfg.mask.grpDone = true;
        chCfg.extMuxEnable = true;
        Cy_Adc_Channel_Init(&SAR1_ADC_MACRO->CH[SAR1_CH_VPWR], &chCfg);
    }

    /* ---- 使能全部通道 ---- */
    Cy_Adc_Channel_Enable(&SAR1_ADC_MACRO->CH[SAR1_CH_FRONT]);
    Cy_Adc_Channel_Enable(&SAR1_ADC_MACRO->CH[SAR1_CH_REAR]);
    Cy_Adc_Channel_Enable(&SAR1_ADC_MACRO->CH[SAR1_CH_VPWR]);
}

/* ========================================================================== */
/*  三通道读取 + 压力故障诊断/换算 + VPOWER 报警                                 */
/* ========================================================================== */
void Sar1Adc_Read(void)
{
    /* ---- 软件触发组头 CH0(物理), 启动 Group 自动序列 CH0→CH8 ----
     * (含 sensor_diag CH0~CH5 + 本模块 CH6~CH8, 组尾 CH8) */
    Cy_Adc_Channel_SoftwareTrigger(&SAR1_ADC_MACRO->CH[0u]);

    /* ---- 轮询 CH8 (组尾) grpDone ---- */
    cy_stc_adc_interrupt_source_t intr;
    do {
        intr = (cy_stc_adc_interrupt_source_t){ false };
        Cy_Adc_Channel_GetInterruptMaskedStatus(&SAR1_ADC_MACRO->CH[SAR1_CH_VPWR], &intr);
    } while (!intr.grpDone);

    /* ---- 读取三通道结果 ---- */
    static const uint8_t s_ch_idx[SAR1_NUM_CHANNELS] = { SAR1_CH_FRONT, SAR1_CH_REAR, SAR1_CH_VPWR };
    uint16_t raw[SAR1_NUM_CHANNELS] = { 0u, 0u, 0u };
    bool valid[SAR1_NUM_CHANNELS]   = { false, false, false };

    for (uint8_t i = 0u; i < SAR1_NUM_CHANNELS; i++)
    {
        const uint8_t hwCh = s_ch_idx[i];
        cy_stc_adc_ch_status_t adc_status;
        uint16_t r = 0u;
        if (Cy_Adc_Channel_GetResult(&SAR1_ADC_MACRO->CH[hwCh], &r, &adc_status) == CY_ADC_SUCCESS
            && adc_status.valid)
        {
            raw[i]   = r;
            valid[i] = true;
        }
        Cy_Adc_Channel_ClearInterruptStatus(&SAR1_ADC_MACRO->CH[hwCh], &intr);
    }

    /* ---- 保存原始值 (raw[0]=前桥, raw[1]=后桥, raw[2]=VPOWER) ---- */
    if (valid[0]) s_raw_front = raw[0];
    if (valid[1]) s_raw_rear  = raw[1];
    if (valid[2]) s_raw_vpwr  = raw[2];

    /* ---- VPOWER 电压换算 (mV) ---- */
    s_mv_vpwr  = (uint16_t)((uint32_t)s_raw_vpwr * SAR1_ADC_VREF_MV / SAR1_ADC_12BIT_MAX);

    /* ---- 前桥压力故障诊断 (双层窗口 + 50次对称消抖, VPOWER 异常时跳过) ---- */
    const bool vpwr_ok = (PSWvIgn >= 180u && PSWvIgn <= 320u);
    if (vpwr_ok && (s_raw_front < PRESS_ERR_MIN_RAW || s_raw_front > PRESS_ERR_MAX_RAW))
    {
        s_front_ok_cnt = 0u;
        s_front_err_cnt++;
        if (s_front_err_cnt >= PRESS_FAULT_STREAK)
        {
            s_front_fault = true;
            s_front_err_cnt = PRESS_FAULT_STREAK;
        }
    }
    else
    {
        s_front_err_cnt = 0u;
        s_front_ok_cnt++;
        if (s_front_ok_cnt >= PRESS_FAULT_STREAK)
        {
            s_front_fault = false;
            s_front_ok_cnt = PRESS_FAULT_STREAK;
        }
    }

    /* ---- 后桥压力故障诊断 ---- */
    if (vpwr_ok && (s_raw_rear < PRESS_ERR_MIN_RAW || s_raw_rear > PRESS_ERR_MAX_RAW))
    {
        s_rear_ok_cnt = 0u;
        s_rear_err_cnt++;
        if (s_rear_err_cnt >= PRESS_FAULT_STREAK)
        {
            s_rear_fault = true;
            s_rear_err_cnt = PRESS_FAULT_STREAK;
        }
    }
    else
    {
        s_rear_err_cnt = 0u;
        s_rear_ok_cnt++;
        if (s_rear_ok_cnt >= PRESS_FAULT_STREAK)
        {
            s_rear_fault = false;
            s_rear_ok_cnt = PRESS_FAULT_STREAK;
        }
    }

    /* ---- 前桥压力换算 (0.5V~4.7V 线性, 容忍区 clamp) ---- */
    if (!s_front_fault)
    {
        if (s_raw_front >= PRESS_NORMAL_MIN_RAW && s_raw_front <= PRESS_NORMAL_MAX_RAW)
        {
            uint32_t kpa = (uint32_t)(s_raw_front - PRESS_NORMAL_MIN_RAW) * PRESS_MAX_KPA
                         / (PRESS_NORMAL_MAX_RAW - PRESS_NORMAL_MIN_RAW);
            s_kpa_front = (uint16_t)kpa;
        }
        else if (s_raw_front < PRESS_NORMAL_MIN_RAW)
        {
            s_kpa_front = 0u;
        }
        else
        {
            s_kpa_front = PRESS_MAX_KPA;
        }
    }
    else
    {
        s_kpa_front = 0u;
    }

    /* ---- 后桥压力换算 ---- */
    if (!s_rear_fault)
    {
        if (s_raw_rear >= PRESS_NORMAL_MIN_RAW && s_raw_rear <= PRESS_NORMAL_MAX_RAW)
        {
            uint32_t kpa = (uint32_t)(s_raw_rear - PRESS_NORMAL_MIN_RAW) * PRESS_MAX_KPA
                         / (PRESS_NORMAL_MAX_RAW - PRESS_NORMAL_MIN_RAW);
            s_kpa_rear = (uint16_t)kpa;
        }
        else if (s_raw_rear < PRESS_NORMAL_MIN_RAW)
        {
            s_kpa_rear = 0u;
        }
        else
        {
            s_kpa_rear = PRESS_MAX_KPA;
        }
    }
    else
    {
        s_kpa_rear = 0u;
    }

    /* ---- VPOWER 电压换算: ADC_V(mV) × ratio → 实际电压(×10, fact:0.1V) ---- */
    {
        uint32_t vpwr = (uint32_t)s_mv_vpwr * VPWR_DIVIDER_RATIO_NUM / VPWR_DIVIDER_RATIO_DEN;
        s_vpwr_x10 = (uint16_t)(vpwr / 100u);
    }

    /* ---- VPOWER 高压报警 (对称消抖) ---- */
    if (s_vpwr_x10 >= VPWR_ALARM_HIGH_MV)
    {
        s_vpwr_high_ok_cnt = 0u;
        s_vpwr_high_cnt++;
        if (s_vpwr_high_cnt >= VPWR_ALARM_STREAK)
        {
            s_vpwr_high_alarm = true;
            s_vpwr_high_cnt = VPWR_ALARM_STREAK;
        }
    }
    else
    {
        s_vpwr_high_cnt = 0u;
        s_vpwr_high_ok_cnt++;
        if (s_vpwr_high_ok_cnt >= VPWR_ALARM_STREAK)
        {
            s_vpwr_high_alarm = false;
            s_vpwr_high_ok_cnt = VPWR_ALARM_STREAK;
        }
    }

    /* ---- VPOWER 低压报警 (对称消抖) ---- */
    if (s_vpwr_x10 <= VPWR_ALARM_LOW_MV && s_vpwr_x10 > 0u)
    {
        s_vpwr_low_ok_cnt = 0u;
        s_vpwr_low_cnt++;
        if (s_vpwr_low_cnt >= VPWR_ALARM_STREAK)
        {
            s_vpwr_low_alarm = true;
            s_vpwr_low_cnt = VPWR_ALARM_STREAK;
        }
    }
    else
    {
        s_vpwr_low_cnt = 0u;
        s_vpwr_low_ok_cnt++;
        if (s_vpwr_low_ok_cnt >= VPWR_ALARM_STREAK)
        {
            s_vpwr_low_alarm = false;
            s_vpwr_low_ok_cnt = VPWR_ALARM_STREAK;
        }
    }

}

/* ========================================================================== */
/*  压力查询接口                                                               */
/* ========================================================================== */
uint16_t Pressure_GetFrontRaw(void)  { return s_raw_front; }
uint16_t Pressure_GetRearRaw(void)   { return s_raw_rear; }
uint16_t Pressure_GetFrontKpa(void)  { return s_kpa_front; }
uint16_t Pressure_GetRearKpa(void)   { return s_kpa_rear; }
bool     Pressure_IsFrontFault(void) { return s_front_fault; }
bool     Pressure_IsRearFault(void)  { return s_rear_fault; }

/* ---- VPOWER 查询接口 ---- */
uint16_t Vpower_GetRaw(void)         { return s_raw_vpwr; }
uint16_t Vpower_GetVoltage(void)     { return s_vpwr_x10; }
bool     Vpower_IsHighAlarm(void)    { return s_vpwr_high_alarm; }
bool     Vpower_IsLowAlarm(void)     { return s_vpwr_low_alarm; }
