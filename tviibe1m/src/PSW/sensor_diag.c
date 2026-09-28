/**
 * @file    sensor_diag.c
 * @brief   ADC 传感器故障诊断实现 — 开路/短路/间隙过大检测
 *
 * @details SAR ADC0 SARMUX0, 单 Group 自动序列, 软件触发, 软件轮询完成。
 *          SensorDiag_CalcAndProcess() 单函数完成: 触发→读缓冲→统计→诊断,
 *          100 点环形缓冲 (1s 窗口), 故障诊断带滞回 + 速度门控。
 */

#include "sensor_diag.h"
#include "wheel_speed.h"
#include "cy_project.h"
#include "cy_device_headers.h"

/* ========================================================================== */
/*  全局变量                                                                   */
/* ========================================================================== */
volatile sensor_ch_diag_t g_sensor_diag[ADC_DIAG_NUM_CHANNELS] = {0};
volatile sensor_fault_status_t g_sensor_fault_status = {0};

/* ADC 数据缓冲区: 每通道 100 点环形缓冲 */
static uint16_t s_adc_buf[ADC_DIAG_NUM_CHANNELS][ADC_DIAG_BUF_SIZE] = {{0}};
static uint8_t  s_adc_buf_idx        = 0u;
static uint8_t  s_adc_buf_fill_count = 0u;

/* ========================================================================== */
/*  ADC 故障诊断处理 — 基于 100 点平均值/峰峰值                                   */
/* ========================================================================== */
static void SensorDiag_Process(const uint16_t avg_raw[ADC_DIAG_NUM_CHANNELS],
                               const uint16_t amplitude_raw[ADC_DIAG_NUM_CHANNELS])
{
    const uint16_t vehicle_rpm = WheelSpeed_VehicleRefRpm();

    for (uint8_t i = 0u; i < ADC_DIAG_NUM_CHANNELS; i++)
    {
        if (!IS_ADC_DIAG_CH_ENABLED(i)) continue;

        g_sensor_diag[i].adc_raw    = avg_raw[i];
        g_sensor_diag[i].voltage_mv = (avg_raw[i] * ADC_VREF_MV) / ADC_12BIT_MAX;

        /* 开路故障 (低电压 ~0.55V~0.68V) — 滞回比较 */
        if (!g_sensor_diag[i].fault_open)
        {
            if (avg_raw[i] >= DIAG_OPEN_MIN && avg_raw[i] <= DIAG_OPEN_MAX)
                g_sensor_diag[i].fault_open = true;
        }
        else
        {
            if (avg_raw[i] < (DIAG_OPEN_MIN - DIAG_OPEN_HYST) ||
                avg_raw[i] > (DIAG_OPEN_MAX + DIAG_OPEN_HYST))
                g_sensor_diag[i].fault_open = false;
        }

        /* 短路故障 (高电压 ~3.0V~3.4V) — 滞回比较 */
        if (!g_sensor_diag[i].fault_short)
        {
            if (avg_raw[i] >= DIAG_SHORT_MIN && avg_raw[i] <= DIAG_SHORT_MAX)
                g_sensor_diag[i].fault_short = true;
        }
        else
        {
            if (avg_raw[i] < (DIAG_SHORT_MIN - DIAG_SHORT_HYST) ||
                avg_raw[i] > (DIAG_SHORT_MAX + DIAG_SHORT_HYST))
                g_sensor_diag[i].fault_short = false;
        }

        /* 间隙过大 (峰峰值衰减) — 速度窗口: 15 km/h  < vehicle_rpm */
        if ( vehicle_rpm > DIAG_GAP_RPM_THRESH)
        {
            if (!g_sensor_diag[i].fault_gap_too_large)
            {
                if (amplitude_raw[i] < DIAG_GAP_AMPL_MIN)
                    g_sensor_diag[i].fault_gap_too_large = true;
            }
            else
            {
                if (amplitude_raw[i] >= (DIAG_GAP_AMPL_MIN + DIAG_GAP_AMPL_HYST))
                    g_sensor_diag[i].fault_gap_too_large = false;
            }
        }
    }

    /* 更新故障状态汇总结构体 */
    g_sensor_fault_status.ch0_fault_open          = (IS_ADC_DIAG_CH_ENABLED(0) && g_sensor_diag[0].fault_open);
    g_sensor_fault_status.ch0_fault_short         = (IS_ADC_DIAG_CH_ENABLED(0) && g_sensor_diag[0].fault_short);
    g_sensor_fault_status.ch0_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(0) && g_sensor_diag[0].fault_gap_too_large);

    g_sensor_fault_status.ch1_fault_open          = (IS_ADC_DIAG_CH_ENABLED(1) && g_sensor_diag[1].fault_open);
    g_sensor_fault_status.ch1_fault_short         = (IS_ADC_DIAG_CH_ENABLED(1) && g_sensor_diag[1].fault_short);
    g_sensor_fault_status.ch1_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(1) && g_sensor_diag[1].fault_gap_too_large);

    g_sensor_fault_status.ch2_fault_open          = (IS_ADC_DIAG_CH_ENABLED(2) && g_sensor_diag[2].fault_open);
    g_sensor_fault_status.ch2_fault_short         = (IS_ADC_DIAG_CH_ENABLED(2) && g_sensor_diag[2].fault_short);
    g_sensor_fault_status.ch2_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(2) && g_sensor_diag[2].fault_gap_too_large);

    g_sensor_fault_status.ch3_fault_open          = (IS_ADC_DIAG_CH_ENABLED(3) && g_sensor_diag[3].fault_open);
    g_sensor_fault_status.ch3_fault_short         = (IS_ADC_DIAG_CH_ENABLED(3) && g_sensor_diag[3].fault_short);
    g_sensor_fault_status.ch3_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(3) && g_sensor_diag[3].fault_gap_too_large);

    g_sensor_fault_status.ch4_fault_open          = (IS_ADC_DIAG_CH_ENABLED(4) && g_sensor_diag[4].fault_open);
    g_sensor_fault_status.ch4_fault_short         = (IS_ADC_DIAG_CH_ENABLED(4) && g_sensor_diag[4].fault_short);
    g_sensor_fault_status.ch4_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(4) && g_sensor_diag[4].fault_gap_too_large);

    g_sensor_fault_status.ch5_fault_open          = (IS_ADC_DIAG_CH_ENABLED(5) && g_sensor_diag[5].fault_open);
    g_sensor_fault_status.ch5_fault_short         = (IS_ADC_DIAG_CH_ENABLED(5) && g_sensor_diag[5].fault_short);
    g_sensor_fault_status.ch5_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(5) && g_sensor_diag[5].fault_gap_too_large);

}

/* ========================================================================== */
/*  ADC 初始化和通道配置                                                        */
/* ========================================================================== */
void SensorDiag_Init(void)
{
    /* ADC 时钟: 动态读取 clk_peri, 分频到 ≤26.67MHz */
    uint32_t periFreq = 0u;
    Cy_SysClk_GetClkPeriFrequency(&periFreq);
    uint32_t adcMaxFreq = 26670000ul;
    uint32_t divNum = (periFreq + adcMaxFreq / 2) / adcMaxFreq;

    Cy_SysClk_PeriphAssignDivider(ADC_DIAG_PCLK, CY_SYSCLK_DIV_16_BIT, ADC_DIAG_CLK_DIV);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, ADC_DIAG_CLK_DIV, (divNum - 1ul));
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, ADC_DIAG_CLK_DIV);

    /* 采样时间: ceil(412ns × f_adc) × 5 */
    uint32_t actualAdcFreq = periFreq / divNum;
    uint32_t samplingCycle = (412ull * actualAdcFreq + 500000000ull) / 1000000000ull;
    if (samplingCycle < 2u) samplingCycle = 2u;
    samplingCycle *= 5u;

    /* ADC 全局初始化 */
    cy_stc_adc_config_t adcConfig =
    {
        .preconditionTime = 0u, .powerupTime = 0u, .enableIdlePowerDown = false,
        .msbStretchMode = CY_ADC_MSB_STRETCH_MODE_2CYCLE,
        .enableHalfLsbConv = 1u,
        .sarMuxEnable = true, .adcEnable = true, .sarIpEnable = true,
    };
    Cy_Adc_Init(ADC_DIAG_MACRO, &adcConfig);

    /* 引脚配置: 6 路 SARMUX 直连, 模拟模式 */
    {
        cy_stc_gpio_pin_config_t adcPinCfg =
        {
            .outVal = 0ul, .driveMode = CY_GPIO_DM_ANALOG, .hsiom = ADC_DIAG_CH0_MUX,
            .intEdge = 0ul, .intMask = 0ul, .vtrip = 0ul, .slewRate = 0ul, .driveSel = 0ul,
        };
        Cy_GPIO_Pin_Init(ADC_DIAG_CH0_PORT, ADC_DIAG_CH0_PIN, &adcPinCfg);
        adcPinCfg.hsiom = ADC_DIAG_CH1_MUX;
        Cy_GPIO_Pin_Init(ADC_DIAG_CH1_PORT, ADC_DIAG_CH1_PIN, &adcPinCfg);
        adcPinCfg.hsiom = ADC_DIAG_CH2_MUX;
        Cy_GPIO_Pin_Init(ADC_DIAG_CH2_PORT, ADC_DIAG_CH2_PIN, &adcPinCfg);
        adcPinCfg.hsiom = ADC_DIAG_CH3_MUX;
        Cy_GPIO_Pin_Init(ADC_DIAG_CH3_PORT, ADC_DIAG_CH3_PIN, &adcPinCfg);
        adcPinCfg.hsiom = ADC_DIAG_CH4_MUX;
        Cy_GPIO_Pin_Init(ADC_DIAG_CH4_PORT, ADC_DIAG_CH4_PIN, &adcPinCfg);
        adcPinCfg.hsiom = ADC_DIAG_CH5_MUX;
        Cy_GPIO_Pin_Init(ADC_DIAG_CH5_PORT, ADC_DIAG_CH5_PIN, &adcPinCfg);
    }

    /* 通道配置: CH0~CH5 单 Group 自动序列, CH5 为组尾 */
    {
        const cy_stc_adc_channel_config_t adcChBase =
        {
            .triggerSelection = CY_ADC_TRIGGER_OFF,
            .channelPriority = 0u,
            .preenptionType = CY_ADC_PREEMPTION_FINISH_RESUME,
            .isGroupEnd = false,
            .doneLevel = CY_ADC_DONE_LEVEL_PULSE,
            .extMuxSelect = 0u, .extMuxEnable = true,   /* 对齐老代码 adc.c: extMuxEnable=true */
            .preconditionMode = CY_ADC_PRECONDITION_MODE_OFF,
            .overlapDiagMode = CY_ADC_OVERLAP_DIAG_MODE_OFF,
            .sampleTime = samplingCycle,
            .calibrationValueSelect = CY_ADC_CALIBRATION_VALUE_REGULAR,
            .postProcessingMode = CY_ADC_POST_PROCESSING_MODE_NONE,
            .resultAlignment = CY_ADC_RESULT_ALIGNMENT_RIGHT,
            .signExtention = CY_ADC_SIGN_EXTENTION_UNSIGNED,
            .averageCount = 0u, .rightShift = 0u,
            .rangeDetectionMode = CY_ADC_RANGE_DETECTION_MODE_INSIDE_RANGE,
            .rangeDetectionLoThreshold = 0x0000u, .rangeDetectionHiThreshold = 0x0FFFu,
            .mask.grpDone = false, .mask.grpCancelled = false, .mask.grpOverflow = false,
            .mask.chRange = false, .mask.chPulse = false, .mask.chOverflow = false,
        };

        cy_stc_adc_channel_config_t adcCh0Cfg = adcChBase;
        adcCh0Cfg.pinAddress = CY_ADC_PIN_ADDRESS_AN4;
        adcCh0Cfg.portAddress = CY_ADC_PORT_ADDRESS_SARMUX0;
        Cy_Adc_Channel_Init(&ADC_DIAG_MACRO->CH[0u], &adcCh0Cfg);

        cy_stc_adc_channel_config_t adcCh1Cfg = adcChBase;
        adcCh1Cfg.pinAddress = CY_ADC_PIN_ADDRESS_AN5;
        adcCh1Cfg.portAddress = CY_ADC_PORT_ADDRESS_SARMUX0;
        Cy_Adc_Channel_Init(&ADC_DIAG_MACRO->CH[1u], &adcCh1Cfg);

        cy_stc_adc_channel_config_t adcCh2Cfg = adcChBase;
        adcCh2Cfg.pinAddress = CY_ADC_PIN_ADDRESS_AN8;
        adcCh2Cfg.portAddress = CY_ADC_PORT_ADDRESS_SARMUX0;
        Cy_Adc_Channel_Init(&ADC_DIAG_MACRO->CH[2u], &adcCh2Cfg);

        cy_stc_adc_channel_config_t adcCh3Cfg = adcChBase;
        adcCh3Cfg.pinAddress = CY_ADC_PIN_ADDRESS_AN9;
        adcCh3Cfg.portAddress = CY_ADC_PORT_ADDRESS_SARMUX0;
        Cy_Adc_Channel_Init(&ADC_DIAG_MACRO->CH[3u], &adcCh3Cfg);

        cy_stc_adc_channel_config_t adcCh4Cfg = adcChBase;
        adcCh4Cfg.pinAddress = CY_ADC_PIN_ADDRESS_AN11;
        adcCh4Cfg.portAddress = CY_ADC_PORT_ADDRESS_SARMUX0;
        Cy_Adc_Channel_Init(&ADC_DIAG_MACRO->CH[4u], &adcCh4Cfg);

        cy_stc_adc_channel_config_t adcCh5Cfg = adcChBase;
        adcCh5Cfg.pinAddress = CY_ADC_PIN_ADDRESS_AN12;
        adcCh5Cfg.portAddress = CY_ADC_PORT_ADDRESS_SARMUX0;
        /* CH5 不是组尾: 与 sar1_adc CH6~CH8 合成一个组(CH0~CH8), 组尾统一 CH8 */
        Cy_Adc_Channel_Init(&ADC_DIAG_MACRO->CH[5u], &adcCh5Cfg);
    }

    /* 使能全部 ADC 通道 */
    for (uint8_t ch = ADC_DIAG_FIRST_CH; ch < (ADC_DIAG_FIRST_CH + ADC_DIAG_NUM_CHANNELS); ch++)
        Cy_Adc_Channel_Enable(&ADC_DIAG_MACRO->CH[ch]);
}

/* ========================================================================== */
/*  ADC 采样 + 故障诊断 — 单函数, 主循环每 10ms 调用                             */
/*  触发转换 → 结果入 100 点缓冲 → 统计 → 诊断, 全部在此完成                      */
/* ========================================================================== */
void SensorDiag_CalcAndProcess(void)
{
    /* ---- ① 读取结果入环形缓冲 ----
     * 组转换(CH0~CH8)由 Sar1Adc_Read() 统一软件触发并等待组尾 CH8,
     * 此处仅读取 CH0~CH5 的结果 (主循环顺序: Sar1Adc_Read 先于本函数)。 */
    const uint8_t idx = s_adc_buf_idx;
    cy_stc_adc_interrupt_source_t intr = { false };

    /* ---- ② 读取结果入环形缓冲 ---- */
    for (uint8_t ch = 0u; ch < ADC_DIAG_NUM_CHANNELS; ch++)
    {
        if (!IS_ADC_DIAG_CH_ENABLED(ch)) continue;
        cy_stc_adc_ch_status_t adc_status;
        uint16_t raw = 0u;
        if (Cy_Adc_Channel_GetResult(&ADC_DIAG_MACRO->CH[ch], &raw, &adc_status) == CY_ADC_SUCCESS
            && adc_status.valid)
        {
            s_adc_buf[ch][idx] = raw;
        }
        Cy_Adc_Channel_ClearInterruptStatus(&ADC_DIAG_MACRO->CH[ch], &intr);
    }

    s_adc_buf_idx = (idx + 1u) % ADC_DIAG_BUF_SIZE;
    if (s_adc_buf_fill_count < ADC_DIAG_BUF_SIZE)
        s_adc_buf_fill_count++;

    /* ---- ③ 缓冲未满 (<1s 数据), 仅采样不诊断 ---- */
    if (s_adc_buf_fill_count < ADC_DIAG_BUF_SIZE)
        return;

    /* ---- ④ 统计: 100 点平均值/峰峰值 ---- */
    uint16_t avg_raw[ADC_DIAG_NUM_CHANNELS]       = {0};
    uint16_t amplitude_raw[ADC_DIAG_NUM_CHANNELS] = {0};
    for (uint8_t ch = 0u; ch < ADC_DIAG_NUM_CHANNELS; ch++)
    {
        if (!IS_ADC_DIAG_CH_ENABLED(ch)) continue;

        uint32_t sum = 0u;
        uint16_t vmin = 0xFFFu;
        uint16_t vmax = 0u;

        for (uint8_t k = 0u; k < ADC_DIAG_BUF_SIZE; k++)
        {
            const uint16_t v = s_adc_buf[ch][k];
            sum += v;
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }

        avg_raw[ch]       = (uint16_t)(sum / ADC_DIAG_BUF_SIZE);
        amplitude_raw[ch] = vmax - vmin;
    }

    /* ---- ⑤ 故障诊断 (滞回 + 速度门控) ---- */
    SensorDiag_Process(avg_raw, amplitude_raw);
}

/* ========================================================================== */
/*  ADC 中断处理 (保留空函数体, 当前使用软件轮询)                                */
/* ========================================================================== */
void SensorDiag_IntHandler(void)
{
    (void)0;
}
