/**
 * @file    p12_1_adc.c
 * @brief   P12.1 独立 SAR1 ADC 采集实现 (0~5V 模拟输入)
 *
 * @details P12.1 = PADS37 = ADC[1](PASS0_SAR1) 的 AN5, 走 SARMUX0 直连。
 *          参考 examples/adc/OneChannelConversion_SwTrigger 单通道例子:
 *          独立 Cy_Adc_Init + 单通道 CH0 (组头=组尾) + 软件触发,
 *          但用轮询代替中断, 契合项目 10ms 主循环节奏。
 */

#include "p12_1_adc.h"
#include "cy_project.h"
#include "cy_device_headers.h"

#if (HW_REV_P12_1_ADC == 1u)

/* ========================================================================== */
/*  内部状态                                                                   */
/* ========================================================================== */
static uint16_t s_raw = 0u;   /* ADC 原始值 (12bit) */
static uint16_t s_mv  = 0u;   /* 换算电压 (mV)     */

/* ========================================================================== */
/*  初始化                                                                     */
/* ========================================================================== */
void P12_1_Adc_Init(void)
{
    /* ---- 时钟: 独立分频器 DIV16_NO_ADC2, 分频到 ≤26.67MHz ---- */
    uint32_t periFreq = 0u;
    Cy_SysClk_GetClkPeriFrequency(&periFreq);
    uint32_t adcMaxFreq = 26670000ul;
    uint32_t divNum = (periFreq + adcMaxFreq / 2u) / adcMaxFreq;

    Cy_SysClk_PeriphAssignDivider(P12_1_ADC_PCLK, CY_SYSCLK_DIV_16_BIT, P12_1_ADC_CLK_DIV);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, P12_1_ADC_CLK_DIV, (divNum - 1ul));
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, P12_1_ADC_CLK_DIV);

    /* 采样时间: ceil(412ns × f_adc) × 5 */
    uint32_t actualAdcFreq = periFreq / divNum;
    uint32_t samplingCycle = (412ull * actualAdcFreq + 500000000ull) / 1000000000ull;
    if (samplingCycle < 2u) samplingCycle = 2u;
    samplingCycle *= 5u;

    /* ---- 引脚: P12.1 模拟直连 (GPIO + ANALOG), SAR1 AN5 ---- */
    {
        cy_stc_gpio_pin_config_t adcPinCfg =
        {
            .outVal = 0ul, .driveMode = CY_GPIO_DM_ANALOG, .hsiom = P12_1_ADC_MUX,
            .intEdge = 0ul, .intMask = 0ul, .vtrip = 0ul, .slewRate = 0ul, .driveSel = 0ul,
        };
        Cy_GPIO_Pin_Init(P12_1_ADC_PORT, P12_1_ADC_PIN, &adcPinCfg);
    }

    /* ---- ADC 实例初始化 (独立 SAR1, 不碰 SAR0) ---- */
    {
        cy_stc_adc_config_t adcConfig =
        {
            .preconditionTime = 0u, .powerupTime = 0u, .enableIdlePowerDown = false,
            .msbStretchMode = CY_ADC_MSB_STRETCH_MODE_2CYCLE,
            .enableHalfLsbConv = 1u,
            .sarMuxEnable = true, .adcEnable = true, .sarIpEnable = true,
        };
        Cy_Adc_Init(P12_1_ADC_MACRO, &adcConfig);
    }

    /* ---- 单通道 CH0: AN5, 组头=组尾 ---- */
    {
        cy_stc_adc_channel_config_t chCfg =
        {
            .triggerSelection       = CY_ADC_TRIGGER_OFF,
            .channelPriority        = 0u,
            .preenptionType         = CY_ADC_PREEMPTION_FINISH_RESUME,
            .isGroupEnd             = true,
            .doneLevel              = CY_ADC_DONE_LEVEL_PULSE,
            .pinAddress             = CY_ADC_PIN_ADDRESS_AN5,
            .portAddress            = CY_ADC_PORT_ADDRESS_SARMUX0,
            .extMuxSelect           = 0u, .extMuxEnable = true,
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
            .mask.grpDone      = true,  .mask.grpCancelled = false, .mask.grpOverflow = false,
            .mask.chRange      = false, .mask.chPulse     = false, .mask.chOverflow  = false,
        };
        Cy_Adc_Channel_Init(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH], &chCfg);
    }

    Cy_Adc_Channel_Enable(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH]);
}

/* ========================================================================== */
/*  单通道读取                                                                 */
/* ========================================================================== */
void P12_1_Adc_Read(void)
{
    cy_stc_adc_interrupt_source_t intr = { false };

    /* 软件触发 CH0 (单通道: 组头=组尾) */
    Cy_Adc_Channel_SoftwareTrigger(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH]);

    /* 轮询组尾 grpDone */
    do {
        intr = (cy_stc_adc_interrupt_source_t){ false };
        Cy_Adc_Channel_GetInterruptMaskedStatus(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH], &intr);
    } while (!intr.grpDone);

    /* 读结果 + 换算 mV */
    cy_stc_adc_ch_status_t adc_status;
    uint16_t r = 0u;
    if (Cy_Adc_Channel_GetResult(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH], &r, &adc_status) == CY_ADC_SUCCESS
        && adc_status.valid)
    {
        s_raw = r;
        s_mv  = (uint16_t)((uint32_t)r * P12_1_ADC_VREF_MV / P12_1_ADC_12BIT_MAX);
    }
    Cy_Adc_Channel_ClearInterruptStatus(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH], &intr);
}

/* ---- 查询接口 ---- */
uint16_t P12_1_GetRaw(void)       { return s_raw; }
uint16_t P12_1_GetVoltageMv(void) { return s_mv; }

#else
/* ---- 开关关闭时的空实现, 避免调用点条件编译 ---- */
void P12_1_Adc_Init(void)       { }
void P12_1_Adc_Read(void)       { }
uint16_t P12_1_GetRaw(void)     { return 0u; }
uint16_t P12_1_GetVoltageMv(void) { return 0u; }
#endif
