/**
 * @file    cmn.c
 * @brief   PSW 公共模块实现 — 系统级工具函数 + 板级初始化 + 10ms 主处理
 *
 * @details 从旧版 PSW_old/cmn.c 迁移的核心工具:
 *          | 函数                  | 说明                            |
 *          |-----------------------|--------------------------------|
 *          | delay_us()            | SysTick 微秒延时               |
 *          | SoftWD_init()         | 软件看门狗初始化 (上限 2000ms)  |
 *          | wd_feed()             | 硬狗 (GPIO 翻转) + 软狗 (WDT)   |
 *          | sysRstReason_get()    | 看门狗复位原因查询              |
 *          | system_update_start() | Bootloader 升级入口             |
 *          | system_reboot()       | NVIC 软复位                     |
 *          | board_init()          | 板级初始化 (含 CAN0/CAN1)       |
 *          | PSW_10ms_deal()       | 10ms 主处理 (喂狗+诊断+轮速+...) |
 *
 * @details 已废弃的功能 (不迁移):
 *          - CAN ID 列表        → 已迁移到 can_driver + RTE
 *          - psw_version_get()  → 已存在于 rte_psw.c, 避免重复定义
 *          - flash_init/gpio_init → SDK SystemInit 或各模块自处理
 */

#include "cmn.h"
#include "can.h"
#include "wheel_speed.h"
#include "sensor_diag.h"
#include "sar1_adc.h"
#include "p12_1_adc.h"
#include "bts724g.h"
#include "psw_data.h"
#include "RTE.h"
#include "spi_eeprom.h"
#include "eeprom.h"
#include "timer10ms.h"
#include "gpio.h"
#include "rte_psw.h"

/* ======================================================================== */
/*  阀驱动波形测试开关 (测完波形后置 0)                                       */
/*  1 = 主循环驱动单个阀 (前桥左进气 VALVE_LFI), 输出循环 PWM                  */
/* ======================================================================== */
#define VALVE_DRV_TEST_ENABLE   0u

/* ======================================================================== */
/*  RTE 变量 CAN 打印监控 (调试用)                                            */
/* ======================================================================== */

/* 调试开关: 1=启用 RTE 变量 CAN 打印, 0=关闭 */
#define RTE_DBG_PRINT_ENABLE    0u
/* 调试开关: 1=启用 PSW 变量 CAN 打印, 0=关闭 */
#define PSW_DBG_PRINT_ENABLE    0u

/* 调试开关: 1=主循环每 10ms 翻转一次 X1-9 测试点 (示波器测节拍), 0=关闭 */
/* 用法: 置 1 后烧录, 示波器量 X1-9 (老硬件 P8.1 / 新硬件 P12.1):
 *         方波周期 20ms (高 10ms 低 10ms) = 10ms 定时器准确
 *         方波周期 40ms                      = 10ms 定时器慢 2 倍 (实际 20ms) */
#define TICK_DEBUG_ENABLE       0u


#if (VALVE_DRV_TEST_ENABLE != 0u)
extern void InValActFL(uint16_t period, uint16_t htime);

/* 单阀驱动波形测试: 前桥左进气阀, 0→33%→75%→100% 占空比循环
 * 只在相位切换时调用一次, 避免每 10ms 重置 PWM counter */      
static void valve_drv_test_single(void)
{
    static uint16_t cnt = 0u;
    static uint16_t last_phase = 0xFFFFu;
    uint16_t phase;

    cnt++;
    if (cnt >= 1200u) cnt = 0u;   /* 12 秒一个周期, 循环 */

    if (cnt < 300u)      phase = 0u;   /* 0%   */
    else if (cnt < 600u) phase = 1u;   /* 33%  */
    else if (cnt < 900u) phase = 2u;   /* 75%  */
    else                 phase = 3u;   /* 100% */

    if (phase != last_phase)
    {
        last_phase = phase;
        switch (phase)
        {
        case 0: InValActFL(200u, 0u);   break;
        case 1: InValActFL(300u, 100u); break;
        case 2: InValActFL(400u, 300u); break;
        case 3: InValActFL(500u, 500u); break;
        }
    }
}
#endif

/* ======================================================================== */
/*  delay_us — 微秒延时                                                      */
/* ======================================================================== */
void delay_us(uint32_t n)
{
    Cy_SysTick_DelayInUs(n);
}

/* ======================================================================== */
/*  periph_divider — 外设时钟分频器 (老代码 cmn.c 迁移)                        */
/*  动态读取 clk_peri 实际频率, 按目标频率自动计算分频值, 不硬编码源频率。      */
/*  改标定/晶振后, 各外设时钟自动跟随, 无需手动改分频值。                       */
/* ======================================================================== */
void periph_divider(en_clk_dst_t ipBlock, cy_en_divider_types_t div_type, uint32_t clk_no, uint32_t targetFreq)
{
    if (div_type >= CY_SYSCLK_DIV_TYPE_NUM)
    {
        return;
    }
    uint32_t divIntNum = 0;
    uint32_t divFractNum = 0;
    uint32_t periFreq = 0;
    Cy_SysClk_GetClkPeriFrequency(&periFreq);

    Cy_SysClk_PeriphAssignDivider(ipBlock, div_type, clk_no);

    if (div_type == CY_SYSCLK_DIV_8_BIT || div_type == CY_SYSCLK_DIV_16_BIT)
    {
        divIntNum = DIV_ROUND_UP(periFreq, targetFreq);
        Cy_SysClk_PeriphSetDivider(div_type, clk_no, (divIntNum - 1ul));
    }
    else
    {
        divIntNum = periFreq / targetFreq;
        divFractNum = (uint32_t)((periFreq / (double)targetFreq - divIntNum) * 32.0f + 0.5f);
        Cy_SysClk_PeriphSetFracDivider(div_type, clk_no, (divIntNum - 1ul), divFractNum);
    }
    /* Enable the divider */
    Cy_SysClk_PeriphEnableDivider(div_type, clk_no);
}

/* ======================================================================== */
/*  SoftWD_init — 软件看门狗初始化                                             */
/* ======================================================================== */
void SoftWD_init(void)
{
    Cy_SysReset_ClearAllResetReasons();      /* 清除历史复位原因 */
    Cy_WDT_Init();
    Cy_WDT_Unlock();
    Cy_WDT_SetUpperLimit(2000ul);            /* 上限 2000ms */
    Cy_WDT_SetDebugRun(CY_WDT_ENABLE);       /* 调试模式下继续计数 */
    Cy_WDT_Lock();
    Cy_WDT_Enable();
}

/* ======================================================================== */
/*  看门狗跳过标记 (BSWdebug 测试用例动态控制)                                  */
/* ======================================================================== */
uint8_t g_skipSoftWD = 0u;
uint8_t g_skipHardWD = 0u;

/* ======================================================================== */
/*  wd_feed — 双看门狗喂狗                                                    */
/* ======================================================================== */
void wd_feed(void)
{
    if (g_skipHardWD == 0u)
    {
        Cy_GPIO_Inv(WD_HW_GPIO_PORT, WD_HW_GPIO_PIN);   /* 硬件看门狗: P5.0 翻转 */
    }
    if (g_skipSoftWD == 0u)
    {
        Cy_WDT_ClearWatchdog();                          /* 软件看门狗: WDT 清零 */
    }
}

/* ======================================================================== */
/*  sysRstReason_get — 复位原因查询 (1=WDT, 0=其他)                           */
/* ======================================================================== */
uint8_t sysRstReason_get(void)
{
    uint32_t resetReason;

    resetReason = Cy_SysReset_GetResetReason();
    Cy_SysReset_ClearAllResetReasons();

    if ((resetReason & CY_SYSRESET_WDT) != 0ul)
    {
        return 1;
    }
    return 0;
}

/* ======================================================================== */
/*  system_update_start — 系统升级入口                                        */
/* ======================================================================== */
void system_update_start(uint32_t updateFlag)
{
    CY_SET_REG32(RAM_1_ADDRESS, updateFlag);

    /* 等待 SRAM1 写入完成 */
    while (0ul == (Cy_Cpu_SramWriteBufferStatus(CY_CPU_SRAM1)))
    {
        ;
    }

    /* SRAM1 配置为保持模式 (复位后数据不丢失) */
    Cy_Cpu_SramPowerModeSet(CY_CPU_SRAM1, CY_CPU_SRAM_PM_RETAINED, 0ul);

    NVIC_SystemReset();
}

/* ======================================================================== */
/*  system_reboot — 系统软复位                                                */
/* ======================================================================== */
void system_reboot(void)
{
    NVIC_SystemReset();
}

/* ======================================================================== */
/*  CAN RX 过滤表 — 开发阶段硬编码占位, 正式版由 COM 层通过 RTE 接口下发          */
/*                                                                            */
/*  CAN0 = 公共CAN (底盘CAN, J1939) — 需按整车网络协议适配                       */
/*  CAN1 = 调试CAN (诊断/标定工具) — 诊断 ID                                   */
/* ======================================================================== */

/* ======================================================================== */
/*  board_init — 板级初始化 (含 GPIO/CAN/继电器)                              */
/*                                                                           */
/*  旧版等效: PSW_old/cmn.c board_init()                                     */
/*  flash_init → 不需要 (SDK SystemInit 已处理)                              */
/*  gpio_init → Gpio_Init (gpio.c)                                          */
/*  SCC3000 → 废弃 (新版用 sensor_diag ADC 轮询)                             */
/*  pwm_comp → Bts724g_Init (BTS724G SPI 驱动)                              */
/*  pwm_cap  → WheelSpeed_Init (TCPWM 中断捕获)                             */
/*  adc_init → SensorDiag_Init + Sar1Adc_Init (拆分为两个模块)               */
/*                                                                           */
/*  CAN: 由上层通过 can_info_cfg + can0_bitrate_set 配置后, 此处 can_init()   */
/*       初始化。CAN ID 白名单不再硬编码, 交由 COM/应用层下发。                  */
/* ======================================================================== */
void board_init(void)
{
    SystemInit();
    __enable_irq();

    /* PSW 版本号赋值 (RTEPSW_Version 定义在 RTE.c, 此处只赋值不定义) */
    RtePsw_VersionInit();

    /* WDT 初始化 */
    SoftWD_init();

    /* GPIO 引脚初始化 (继电器/UB/指示灯/接插件, 上电默认状态) */
    Gpio_Init();

    /* 闭合总继电器 (P14.3),*/
    Gpio_RelayCtrl(1u);

    /* CAN0 + CAN1 初始化 (公共CAN + 调试CAN, ID 白名单由 can_info_cfg 配置) */
    can_init();

    /* CAN 初始化耗时较长, 中间喂一次狗 */
    wd_feed();

    /* ADC 传感器诊断  */
    SensorDiag_Init();

    /* SAR1 ADC — 压力传感器 + VPOWER */
    Sar1Adc_Init();

    /* P12.1 独立 SAR1 ADC — 0~5V 模拟采集 (HW_REV_P12_1_ADC=0 时空实现) */
    P12_1_Adc_Init();

    /* 轮速采集 — 6 通道 TCPWM 捕获 */
    WheelSpeed_Init();

    /* 阀驱动 — BTS724G SPI 初始化 */
    Bts724g_Init();

    /* SPI EEPROM 传输层 + 命令层 */
    SpiEeprom_Init();
    Eeprom_Init();

    wd_feed();

    /* 10ms 周期定时器 (最后启动, ISR 开始产生节拍) */
    Timer10ms_Init();

    wd_feed();
}

/* ======================================================================== */
/*  PSW_10ms_deal — 10ms 周期主处理                                          */
/*                                                                           */
/*  旧版等效: PSW_old/cmn.c PSW_10ms_deal()                                  */
/*  SCC3000/SCC3134 ESC 传感器 → ADC 轮询 (sensor_diag)                      */
/*  DMA 轮速 → TCPWM 中断 (wheel_speed)                                      */
/*  FarErrorCheck → ValveDiag_Process (bts724g)                              */
/*  adc_calc_all_result → Sar1Adc_Read (sar1_adc)                            */
/*                                                                           */
/*  主循环可简化为:                                                           */
/*    if (g_b10msTick) { PSW_10ms_deal(); }                                  */
/* ======================================================================== */
void PSW_10ms_deal(void)
{
    /* ① 喂狗 — 硬狗 (GPIO 翻转) + 软狗 (WDT 清零) */
    wd_feed();

    /* ② 压力传感器 + VPOWER 电压读取 (SAR1 ADC, 必须先于阀诊断) */
    Sar1Adc_Read();

    /* ②.5 P12.1 模拟量读取 (HW_REV_P12_1_ADC=0 时空实现) */
    P12_1_Adc_Read();

    /* ③ PSW* 中间变量刷新 (VPOWER 电压 → PSWvIgn, 阀诊断门控依赖) */
    PSWData_Refresh();

    /* ④ 阀驱动波形测试 (置 VALVE_DRV_TEST_ENABLE=0 关闭) */
#if (VALVE_DRV_TEST_ENABLE != 0u)
    valve_drv_test_single();
#endif

    /* ④ 阀诊断状态机 + 芯片级故障汇总 (依赖 PSWvIgn 门控) */
    ValveDiag_Process();

    /* ⑤ 轮速计算 + 中值滤波 + CAN 发送 */
    WheelSpeed_CalcAndSend();

    /* ⑥ ADC 传感器诊断 (开路/短路/间隙) */
    SensorDiag_CalcAndProcess();

    /* ⑥ 阀诊断调试输出 (仅 VALVE_DIAG_DEBUG=1) */
#if (VALVE_DIAG_DEBUG != 0u)
    ValveDiag_DebugDump();
#endif

    /* ⑦ 继电器 VPOWER 判定节拍推进 (无 pending 时极速返回) */
    Gpio_RelayTick();

    /* ⑧ RTE 数据写入 (PSW* 已在步骤③刷新) */
    PSWDataToRte();

    /* ⑧.5 RTE 变量 CAN 打印监控 (RTE_DBG_PRINT_ENABLE=0 关闭) */
#if (RTE_DBG_PRINT_ENABLE != 0u)
    RTE_CanPrint_Monitor();
#endif

    /* ⑧.6 PSW 变量 CAN 打印监控 (PSW_DBG_PRINT_ENABLE=1 开启) */
#if (PSW_DBG_PRINT_ENABLE != 0u)
    PSW_CanPrint_Monitor();
#endif

    /* ⑨ CAN0/CAN1 busoff 自动恢复 */
    can0_busoff_resume();
    can1_busoff_resume();

    /* ⑩ 清除 10ms 节拍标志 */
    g_b10msTick = false;

    /* ⑪ 临时: 10ms 节拍验证 — 翻转 X1-9 测试点 (测完把 TICK_DEBUG_ENABLE 置 0) */
#if (TICK_DEBUG_ENABLE != 0u)
    {
        static uint8_t s_tick_toggle = 0u;
        s_tick_toggle ^= 1u;        /* 每 10ms 翻转一次 */
        CtrlX1_9(s_tick_toggle);
    }
#endif
}

/* ======================================================================== */
/*  RTE 变量 CAN 打印监控 (调试用, RTE_DBG_PRINT_ENABLE 开启时编译)           */
/*                                                                           */
/*  用调试 CAN (CAN1, 无发送白名单) 周期打印 RTE 变量, 监控 PSWDataToRte()   */
/*  是否正常更新 RTE。                                                        */
/*                                                                            */
/*  7 帧轮转, 每 RTE_DBG_PERIOD_MS 发一帧, 7 帧一个完整周期:                  */
/*    帧0 (ID+0): RTEvIgn / RTEBrkPreX4_14 / RTEBrkPreX4_15               */
/*    帧1 (ID+1): RTEWheelSpeedFL / FR / RL                                  */
/*    帧2 (ID+2): RTEWheelSpeedRR / XL / XR                                  */
/*    帧3 (ID+3): 轮速传感器故障 (Open/Short/Gap × 6 通道)                    */
/*    帧4 (ID+4): ABS 阀故障 FL/FR/RL/RR (进/排 × 开/短)                     */
/*    帧5 (ID+5): ABS 阀故障 XL/XR/Tr + ASR 阀故障                            */
/*    帧6 (ID+6): 压力/电源/EEprom/Relay/Chip/ESCM + EEPROM 状态              */
/* ======================================================================== */
#if (RTE_DBG_PRINT_ENABLE != 0u)

#define RTE_DBG_CAN_ID       0x700u     /* 调试报文基准 ID (11bit 标准帧) */
#define RTE_DBG_PERIOD_MS    100u       /* 单帧打印周期 (10ms 整数倍)     */
#define RTE_DBG_FRAME_CNT    7u         /* 帧总数 (0~6)                   */

/* 故障标志 → 单 bit 值 (位域/整型字段统一转 0/1) */
#define RTE_BIT(v)  ((uint8_t)((v) ? 1u : 0u))

void RTE_CanPrint_Monitor(void)
{
    static uint16_t tick  = 0u;
    static uint8_t  frame = 0u;
    uint8_t data[8];

    /* 节拍计数, 每 RTE_DBG_PERIOD_MS 发一帧 */
    tick++;
    if (tick < (RTE_DBG_PERIOD_MS / 10u))
    {
        return;
    }
    tick = 0u;

    data[0] = 0xFFu; data[1] = 0xFFu; data[2] = 0xFFu; data[3] = 0xFFu;
    data[4] = 0xFFu; data[5] = 0xFFu; data[6] = 0xFFu; data[7] = 0xFFu;

    switch (frame)
    {
    case 0u:    /* 帧0: VPOWER + 前/后桥压力 */
        data[0] = (uint8_t)(RTEvIgn >> 8);
        data[1] = (uint8_t)(RTEvIgn & 0xFFu);
        data[2] = (uint8_t)(RTEBrkPreX4_14 >> 8);
        data[3] = (uint8_t)(RTEBrkPreX4_14 & 0xFFu);
        data[4] = (uint8_t)(RTEBrkPreX4_15 >> 8);
        data[5] = (uint8_t)(RTEBrkPreX4_15 & 0xFFu);
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 0u, data, 6u);
        break;

    case 1u:    /* 帧1: 轮速 FL/FR/RL */
        data[0] = (uint8_t)(RTEWheelSpeedFL >> 8);
        data[1] = (uint8_t)(RTEWheelSpeedFL & 0xFFu);
        data[2] = (uint8_t)(RTEWheelSpeedFR >> 8);
        data[3] = (uint8_t)(RTEWheelSpeedFR & 0xFFu);
        data[4] = (uint8_t)(RTEWheelSpeedRL >> 8);
        data[5] = (uint8_t)(RTEWheelSpeedRL & 0xFFu);
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 1u, data, 6u);
        break;

    case 2u:    /* 帧2: 轮速 RR/XL/XR */
        data[0] = (uint8_t)(RTEWheelSpeedRR >> 8);
        data[1] = (uint8_t)(RTEWheelSpeedRR & 0xFFu);
        data[2] = (uint8_t)(RTEWheelSpeedXL >> 8);
        data[3] = (uint8_t)(RTEWheelSpeedXL & 0xFFu);
        data[4] = (uint8_t)(RTEWheelSpeedXR >> 8);
        data[5] = (uint8_t)(RTEWheelSpeedXR & 0xFFu);
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 2u, data, 6u);
        break;

    case 3u:    /* 帧3: 轮速传感器故障 (bit0~5: FL/FR/RL/RR/XL/XR) */
        data[0] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenFL)  << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenFR)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenRL)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenRR)  << 3u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenXL)  << 4u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenXR)  << 5u) );
        data[1] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrWssShortFL) << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssShortFR) << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssShortRL) << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssShortRR) << 3u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssShortXL) << 4u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssShortXR) << 5u) );
        data[2] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrWssGapFL)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssGapFR)   << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssGapRL)   << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssGapRR)   << 3u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssGapXL)   << 4u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssGapXR)   << 5u) );
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 3u, data, 3u);
        break;

    case 4u:    /* 帧4: ABS 阀故障 FL/FR/RL/RR (bit0~3: InOpen/InShort/OutOpen/OutShort) */
        data[0] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenFL)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortFL)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenFL)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortFL) << 3u) );
        data[1] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenFR)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortFR)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenFR)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortFR) << 3u) );
        data[2] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenRL)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortRL)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenRL)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortRL) << 3u) );
        data[3] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenRR)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortRR)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenRR)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortRR) << 3u) );
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 4u, data, 4u);
        break;

    case 5u:    /* 帧5: ABS 阀 XL/XR/Tr + ASR 阀 */
        data[0] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenXL)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortXL)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenXL)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortXL) << 3u) );
        data[1] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenXR)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortXR)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenXR)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortXR) << 3u) );
        data[2] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenTr)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortTr)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenTr)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortTr) << 3u) );
        data[3] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrASRValveOpenF)  << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrASRValveShortF) << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrASRValveOpenR)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrASRValveShortR) << 3u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrASRValveOpenX)  << 4u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrASRValveShortX) << 5u) );
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 5u, data, 4u);
        break;

    case 6u:    /* 帧6: 压力/电源/EEprom/Relay/Chip/ESCM + EEPROM 状态 */
        data[0] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrPreF) << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrPreR) << 1u)
                           | (RTE_BIT(RTEfErrPreX4_14)        << 2u)
                           | (RTE_BIT(RTEfErrPreX4_15)        << 3u) );
        data[1] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrVbatLo) << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrVbatHi) << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrVpwrLo) << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrVpwrHi) << 3u) );
        data[2] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrEEprom)       << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrEEpromOutBnd) << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrRelay)        << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrDriveChip)    << 3u) );
        data[3] = (uint8_t)( (RTE_BIT(RTEfCOMErr.RTEfErrESCMNoCalib) << 0u)
                           | (RTE_BIT(RTEfCOMErr.RTEfErrESCM)        << 1u)
                           | (RTE_BIT(RTEfCOMErr.RTEfErrESCMSgn)     << 2u) );
        data[4] = RTEfEESt;    /* EEPROM 状态: 255=首次写, 86=非首次写 */
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 6u, data, 5u);
        break;

    default:
        frame = 0u;
        break;
    }

    frame++;
    if (frame >= RTE_DBG_FRAME_CNT)
    {
        frame = 0u;
    }
}

#endif /* RTE_DBG_PRINT_ENABLE */

/* ======================================================================== */
/*  PSW 变量 CAN 打印监控 (调试用, PSW_DBG_PRINT_ENABLE 开启时编译)            */
/*                                                                           */
/*  用调试 CAN (CAN1, 无发送白名单) 周期打印 PSW* 中间变量, 监控驱动层 →       */
/*  PSW* 是否正常更新。与 RTE 层打印 (0x700 起) 对比, 可定位 PSWDataToRte()   */
/*  转换/掩蔽问题。PSW 特有: 芯片级故障 / 压力原始 kpa / 轮速 0.1km/h 原始值   */
/*  / PSWTaskTime / resetReason。                                            */
/*                                                                           */
/*  8 帧轮转, 每 PSW_DBG_PERIOD_MS 发一帧, 8 帧一个完整周期:                  */
/*    帧0 (ID+0): PSWTaskTime / PSWvIgn / 前桥压力 / 后桥压力 (原始 kpa)      */
/*    帧1 (ID+1): PSWWheelSpeedFL / FR / RL (0.1km/h)                        */
/*    帧2 (ID+2): PSWWheelSpeedRR / XL / XR                                  */
/*    帧3 (ID+3): 轮速传感器故障 (Open/Short/Gap × 6 通道)                    */
/*    帧4 (ID+4): ABS 阀故障 FL/FR/RL/RR (进/排 × 开/短)                     */
/*    帧5 (ID+5): ABS 阀故障 XL/XR/Tr + ASR 阀故障                            */
/*    帧6 (ID+6): 驱动芯片故障 (byte0=Open, byte1=Short, bit0~4=U6/U9/U12/U13/U19) */
/*    帧7 (ID+7): 压力/电源/Relay/EEprom 故障 + 重启原因 resetReason          */
/* ======================================================================== */
#if (PSW_DBG_PRINT_ENABLE != 0u)

#define PSW_DBG_CAN_ID       0x710u     /* 调试报文基准 ID (11bit 标准帧) */
#define PSW_DBG_PERIOD_MS    100u       /* 单帧打印周期 (10ms 整数倍)     */
#define PSW_DBG_FRAME_CNT    8u         /* 帧总数 (0~7)                   */

/* 故障标志 → 单 bit 值 (PSW 层独立宏, 不依赖 RTE 块) */
#define PSW_BIT(v)  ((uint8_t)((v) ? 1u : 0u))

void PSW_CanPrint_Monitor(void)
{
    static uint16_t tick  = 0u;
    static uint8_t  frame = 0u;
    uint8_t data[8];

    /* 节拍计数, 每 PSW_DBG_PERIOD_MS 发一帧 */
    tick++;
    if (tick < (PSW_DBG_PERIOD_MS / 10u))
    {
        return;
    }
    tick = 0u;

    data[0] = 0xFFu; data[1] = 0xFFu; data[2] = 0xFFu; data[3] = 0xFFu;
    data[4] = 0xFFu; data[5] = 0xFFu; data[6] = 0xFFu; data[7] = 0xFFu;

    switch (frame)
    {
    case 0u:    /* 帧0: 任务时间 + VPOWER + 前/后桥压力 (原始 kpa) */
        data[0] = PSWTaskTime;
        data[1] = (uint8_t)(PSWvIgn >> 8);
        data[2] = (uint8_t)(PSWvIgn & 0xFFu);
        data[3] = (uint8_t)(PSWBrkPreX2_7 >> 8);
        data[4] = (uint8_t)(PSWBrkPreX2_7 & 0xFFu);
        data[5] = (uint8_t)(PSWBrkPreX3_10 >> 8);
        data[6] = (uint8_t)(PSWBrkPreX3_10 & 0xFFu);
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 0u, data, 7u);
        break;

    case 1u:    /* 帧1: 轮速 FL/FR/RL (0.1km/h) */
        data[0] = (uint8_t)(PSWWheelSpeedFL >> 8);
        data[1] = (uint8_t)(PSWWheelSpeedFL & 0xFFu);
        data[2] = (uint8_t)(PSWWheelSpeedFR >> 8);
        data[3] = (uint8_t)(PSWWheelSpeedFR & 0xFFu);
        data[4] = (uint8_t)(PSWWheelSpeedRL >> 8);
        data[5] = (uint8_t)(PSWWheelSpeedRL & 0xFFu);
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 1u, data, 6u);
        break;

    case 2u:    /* 帧2: 轮速 RR/XL/XR (0.1km/h) */
        data[0] = (uint8_t)(PSWWheelSpeedRR >> 8);
        data[1] = (uint8_t)(PSWWheelSpeedRR & 0xFFu);
        data[2] = (uint8_t)(PSWWheelSpeedXL >> 8);
        data[3] = (uint8_t)(PSWWheelSpeedXL & 0xFFu);
        data[4] = (uint8_t)(PSWWheelSpeedXR >> 8);
        data[5] = (uint8_t)(PSWWheelSpeedXR & 0xFFu);
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 2u, data, 6u);
        break;

    case 3u:    /* 帧3: 轮速传感器故障 (bit0~5: FL/FR/RL/RR/XL/XR) */
        data[0] = (uint8_t)( (PSW_BIT(PSWErrWssOpenFL)  << 0u)
                           | (PSW_BIT(PSWErrWssOpenFR)  << 1u)
                           | (PSW_BIT(PSWErrWssOpenRL)  << 2u)
                           | (PSW_BIT(PSWErrWssOpenRR)  << 3u)
                           | (PSW_BIT(PSWErrWssOpenXL)  << 4u)
                           | (PSW_BIT(PSWErrWssOpenXR)  << 5u) );
        data[1] = (uint8_t)( (PSW_BIT(PSWErrWssShortFL) << 0u)
                           | (PSW_BIT(PSWErrWssShortFR) << 1u)
                           | (PSW_BIT(PSWErrWssShortRL) << 2u)
                           | (PSW_BIT(PSWErrWssShortRR) << 3u)
                           | (PSW_BIT(PSWErrWssShortXL) << 4u)
                           | (PSW_BIT(PSWErrWssShortXR) << 5u) );
        data[2] = (uint8_t)( (PSW_BIT(PSWErrWssGapFL)   << 0u)
                           | (PSW_BIT(PSWErrWssGapFR)   << 1u)
                           | (PSW_BIT(PSWErrWssGapRL)   << 2u)
                           | (PSW_BIT(PSWErrWssGapRR)   << 3u)
                           | (PSW_BIT(PSWErrWssGapXL)   << 4u)
                           | (PSW_BIT(PSWErrWssGapXR)   << 5u) );
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 3u, data, 3u);
        break;

    case 4u:    /* 帧4: ABS 阀故障 FL/FR/RL/RR (bit0~3: InOpen/InShort/OutOpen/OutShort) */
        data[0] = (uint8_t)( (PSW_BIT(PSWErrInValOpenFL)   << 0u)
                           | (PSW_BIT(PSWErrInValShortFL)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenFL)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortFL) << 3u) );
        data[1] = (uint8_t)( (PSW_BIT(PSWErrInValOpenFR)   << 0u)
                           | (PSW_BIT(PSWErrInValShortFR)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenFR)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortFR) << 3u) );
        data[2] = (uint8_t)( (PSW_BIT(PSWErrInValOpenRL)   << 0u)
                           | (PSW_BIT(PSWErrInValShortRL)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenRL)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortRL) << 3u) );
        data[3] = (uint8_t)( (PSW_BIT(PSWErrInValOpenRR)   << 0u)
                           | (PSW_BIT(PSWErrInValShortRR)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenRR)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortRR) << 3u) );
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 4u, data, 4u);
        break;

    case 5u:    /* 帧5: ABS 阀 XL/XR/Tr + ASR 阀 */
        data[0] = (uint8_t)( (PSW_BIT(PSWErrInValOpenXL)   << 0u)
                           | (PSW_BIT(PSWErrInValShortXL)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenXL)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortXL) << 3u) );
        data[1] = (uint8_t)( (PSW_BIT(PSWErrInValOpenXR)   << 0u)
                           | (PSW_BIT(PSWErrInValShortXR)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenXR)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortXR) << 3u) );
        data[2] = (uint8_t)( (PSW_BIT(PSWErrInValOpenTr)   << 0u)
                           | (PSW_BIT(PSWErrInValShortTr)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenTr)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortTr) << 3u) );
        data[3] = (uint8_t)( (PSW_BIT(PSWErrASRValveOpenF)  << 0u)
                           | (PSW_BIT(PSWErrASRValveShortF) << 1u)
                           | (PSW_BIT(PSWErrASRValveOpenR)  << 2u)
                           | (PSW_BIT(PSWErrASRValveShortR) << 3u)
                           | (PSW_BIT(PSWErrASRValveOpenX)  << 4u)
                           | (PSW_BIT(PSWErrASRValveShortX) << 5u) );
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 5u, data, 4u);
        break;

    case 6u:    /* 帧6: 驱动芯片故障 (byte0=Open, byte1=Short, bit0~4=U6/U9/U12/U13/U19) */
        data[0] = (uint8_t)( (PSW_BIT(PSWErrOpenDrive724_U6)  << 0u)
                           | (PSW_BIT(PSWErrOpenDrive724_U9)  << 1u)
                           | (PSW_BIT(PSWErrOpenDrive724_U12) << 2u)
                           | (PSW_BIT(PSWErrOpenDrive724_U13) << 3u)
                           | (PSW_BIT(PSWErrOpenDrive724_U19) << 4u) );
        data[1] = (uint8_t)( (PSW_BIT(PSWErrShortDrive724_U6)  << 0u)
                           | (PSW_BIT(PSWErrShortDrive724_U9)  << 1u)
                           | (PSW_BIT(PSWErrShortDrive724_U12) << 2u)
                           | (PSW_BIT(PSWErrShortDrive724_U13) << 3u)
                           | (PSW_BIT(PSWErrShortDrive724_U19) << 4u) );
        data[2] = PSWErrDriveChip;   /* 芯片总故障 */
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 6u, data, 3u);
        break;

    case 7u:    /* 帧7: 压力/电源/Relay/EEprom 故障 + 重启原因 */
        data[0] = (uint8_t)( (PSW_BIT(PSWfErrPreX2_7)  << 0u)
                           | (PSW_BIT(PSWfErrPreX3_10) << 1u)
                           | (PSW_BIT(PSWfErrPreF)     << 2u)
                           | (PSW_BIT(PSWfErrPreR)     << 3u) );
        data[1] = (uint8_t)( (PSW_BIT(PSWErrVpwrLo)        << 0u)
                           | (PSW_BIT(PSWErrVpwrHi)        << 1u)
                           | (PSW_BIT(PSWErrRelay)         << 2u)
                           | (PSW_BIT(PSWfErrEEprom)       << 3u)
                           | (PSW_BIT(PSWfErrEEpromOutBnd) << 4u) );
        data[2] = (uint8_t)(resetReason >> 24);
        data[3] = (uint8_t)(resetReason >> 16);
        data[4] = (uint8_t)(resetReason >> 8);
        data[5] = (uint8_t)(resetReason & 0xFFu);
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 7u, data, 6u);
        break;

    default:
        frame = 0u;
        break;
    }

    frame++;
    if (frame >= PSW_DBG_FRAME_CNT)
    {
        frame = 0u;
    }
}

#endif /* PSW_DBG_PRINT_ENABLE */

