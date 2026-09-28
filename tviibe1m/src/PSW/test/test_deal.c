/**
 * @file    test_deal.c
 * @brief   测试/调试层实现 — 阀波形测试 + CAN 调试上报 + 节拍验证
 *
 * @details 本文件收纳原先散落在 cmn.c 的测试与调试上报代码:
 *            - 单阀 PWM 波形扫描        (TEST_VALVE_DRV_WAVE)
 *            - RTE 变量 CAN 打印 0x700  (TEST_RTE_CAN_PRINT)
 *            - PSW 变量 CAN 打印 0x710  (TEST_PSW_CAN_PRINT)
 *            - 10ms 节拍翻转 P22.0       (TEST_TICK_DEBUG)
 *            - 阀诊断明细输出           (bts724g.h 的 VALVE_DIAG_DEBUG, 非本层开关)
 *
 *          开关全在 test_cfg.h; 三个入口的调用位置约束见 test_deal.h。
 *          总开关 = 0 时本文件只提供三个空入口, 保证 cmn.c 的调用照常链接。
 */

#include "test_cfg.h"
#include "test_deal.h"

#if (TEST_DEAL_ENABLE != 0u)

#include "cmn.h"
#include "can.h"
#include "gpio.h"
#include "psw_data.h"
#include "bts724g.h"
#include "RTE.h"
#include "rte_psw.h"

/* ========================================================================== */
/*  阀波形测试 — 排气阀 (前段高) 占空比 5 档扫描, 每段 2.4s, 12s 循环          */
/*                                                                            */
/*    0% → 恒低    33% → 高10ms/30ms   50% → 15ms/30ms                         */
/*    75% → 22.5ms/30ms   100% → 恒高                                         */
/*  用排气阀是因为 cc0_start = PWM_CC0_OFFSET(0), 可验证该改动;               */
/*  若要测进气阀 (后段高), 把下面 OutValActFL 换成 InValActFL 即可。           */
/*  只在相位切换时下发一次, 避免每 10ms 重置 PWM counter。                      */
/* ========================================================================== */
#if (TEST_VALVE_DRV_WAVE != 0u)

extern void InValActFL(uint16_t period, uint16_t htime);   /* 进气阀: 后段高 */
extern void OutValActFL(uint16_t period, uint16_t htime);  /* 排气阀: 前段高 */

static void valve_drv_test_single(void)
{
    static uint16_t cnt = 0u;
    static uint16_t last_phase = 0xFFFFu;
    uint16_t phase;

    cnt++;
    if (cnt >= 1200u) cnt = 0u;   /* 12 秒一个周期, 循环 */

    if (cnt < 240u)      phase = 0u;   /* 段0: 0%   */
    else if (cnt < 480u) phase = 1u;   /* 段1: 33%  */
    else if (cnt < 720u) phase = 2u;   /* 段2: 50%  */
    else if (cnt < 960u) phase = 3u;   /* 段3: 75%  */
    else                 phase = 4u;   /* 段4: 100% */

    if (phase != last_phase)
    {
        last_phase = phase;

        /* 切段前先关阀 (段0 即保持关闭 → 0%) */
        OutValActFL(300u, 0u);

        switch (phase)
        {
        case 0: break;                            /* 0%   恒低                (LFO P23.3) */
        case 1: OutValActFL(300u, 100u); break;   /* 33%  高 10.0ms / 周期 30ms */
        case 2: OutValActFL(300u, 150u); break;   /* 50%  高 15.0ms / 周期 30ms */
        case 3: OutValActFL(300u, 225u); break;   /* 75%  高 22.5ms / 周期 30ms */
        case 4: OutValActFL(300u, 300u); break;   /* 100% 恒高 (真恒高)         */
        }
    }
}

#endif /* TEST_VALVE_DRV_WAVE */

/* ========================================================================== */
/*  RTE 变量 CAN 打印监控 (TEST_RTE_CAN_PRINT)                                 */
/*                                                                           */
/*  用调试 CAN (CAN1, 无发送白名单) 周期打印 RTE 变量, 监控 PSWDataToRte()    */
/*  是否正常更新 RTE。                                                        */
/*                                                                           */
/*  7 帧轮转, 每 RTE_DBG_PERIOD_MS 发一帧, 7 帧一个完整周期:                  */
/*    帧0 (ID+0): RTEvIgn / RTEBrkPreX4_14 / RTEBrkPreX4_15                   */
/*    帧1 (ID+1): RTEWheelSpeedFL / FR / RL                                   */
/*    帧2 (ID+2): RTEWheelSpeedRR / XL / XR                                   */
/*    帧3 (ID+3): 轮速传感器故障 (Open/Short/Gap × 6 通道)                     */
/*    帧4 (ID+4): ABS 阀故障 FL/FR/RL/RR (进/排 × 开/短)                      */
/*    帧5 (ID+5): ABS 阀故障 XL/XR/Tr + ASR 阀故障                            */
/*    帧6 (ID+6): 压力/电源/EEprom/Relay/Chip/ESCM + EEPROM 状态               */
/* ========================================================================== */
#if (TEST_RTE_CAN_PRINT != 0u)

#define RTE_DBG_CAN_ID       0x700u     /* 调试报文基准 ID (11bit 标准帧) */
#define RTE_DBG_PERIOD_MS    100u       /* 单帧打印周期 (10ms 整数倍)     */
#define RTE_DBG_FRAME_CNT    7u         /* 帧总数 (0~6)                   */

/* 故障标志 → 单 bit 值 (位域/整型字段统一转 0/1) */
#define RTE_BIT(v)  ((uint8_t)((v) ? 1u : 0u))

static void rte_can_print_monitor(void)
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

#endif /* TEST_RTE_CAN_PRINT */

/* ========================================================================== */
/*  PSW 变量 CAN 打印监控 (TEST_PSW_CAN_PRINT)                                 */
/*                                                                           */
/*  用调试 CAN (CAN1, 无发送白名单) 周期打印 PSW* 中间变量, 监控驱动层 →      */
/*  PSW* 是否正常更新。与 RTE 层打印 (0x700 起) 对比, 可定位 PSWDataToRte()   */
/*  转换/掩蔽问题。PSW 特有: 芯片级故障 / 压力原始 kpa / 轮速 0.1km/h 原始值   */
/*  / PSWTaskTime / resetReason。                                            */
/*                                                                           */
/*  8 帧轮转, 每 PSW_DBG_PERIOD_MS 发一帧, 8 帧一个完整周期:                  */
/*    帧0 (ID+0): PSWTaskTime / PSWvIgn / 前桥压力 / 后桥压力 (原始 kpa)      */
/*    帧1 (ID+1): PSWWheelSpeedFL / FR / RL (0.1km/h)                        */
/*    帧2 (ID+2): PSWWheelSpeedRR / XL / XR                                  */
/*    帧3 (ID+3): 轮速传感器故障 (Open/Short/Gap × 6 通道)                     */
/*    帧4 (ID+4): ABS 阀故障 FL/FR/RL/RR (进/排 × 开/短)                      */
/*    帧5 (ID+5): ABS 阀故障 XL/XR/Tr + ASR 阀故障                            */
/*    帧6 (ID+6): 驱动芯片故障 (byte0=Open, byte1=Short, bit0~4=U6/U9/U12/U13/U19) */
/*    帧7 (ID+7): 压力/电源/Relay/EEprom 故障 + 重启原因 resetReason          */
/* ========================================================================== */
#if (TEST_PSW_CAN_PRINT != 0u)

#define PSW_DBG_CAN_ID       0x710u     /* 调试报文基准 ID (11bit 标准帧) */
#define PSW_DBG_PERIOD_MS    100u       /* 单帧打印周期 (10ms 整数倍)     */
#define PSW_DBG_FRAME_CNT    8u         /* 帧总数 (0~7)                   */

/* 故障标志 → 单 bit 值 (PSW 层独立宏, 不依赖 RTE 块) */
#define PSW_BIT(v)  ((uint8_t)((v) ? 1u : 0u))

static void psw_can_print_monitor(void)
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
                           | (PSW_BIT(PSWfErrPreX3_10) << 1u));
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

#endif /* TEST_PSW_CAN_PRINT */

/* ========================================================================== */
/*  测试层入口                                                                 */
/* ========================================================================== */

/** ④ 阀类测试 (须早于 ValveDiag_Process) */
void TestDeal_Early(void)
{
#if (TEST_VALVE_DRV_WAVE != 0u)
    valve_drv_test_single();
#endif
}

/** ⑥ 阀诊断明细输出 (开关在 bts724g.h 的 VALVE_DIAG_DEBUG) */
void TestDeal_Mid(void)
{
#if (VALVE_DIAG_DEBUG != 0u)
    ValveDiag_DebugDump();
#endif
}

#include "PSWdebug.h"   /* 测试用例框架入口 (声明见该头文件) */

/** ⑧.5~⑧.6 CAN 调试上报 + ⑪ 节拍验证 + PSWdebug 用例 (须晚于 PSWDataToRte) */
void TestDeal_Late(void)
{
#if (TEST_RTE_CAN_PRINT != 0u)
    rte_can_print_monitor();
#endif

#if (TEST_PSW_CAN_PRINT != 0u)
    psw_can_print_monitor();
#endif

#if (TEST_TICK_DEBUG != 0u)
    {
        static uint8_t s_tick_toggle = 0u;
        s_tick_toggle ^= 1u;        /* 每 10ms 翻转一次 */
        TestMark_Ctrl(s_tick_toggle);
    }
#endif

    /* PSWdebug 测试用例: CAN 下发编号 (RTEfDbgMsgSW3) → 注册 → 每 10ms 周期执行
     * 位置与老工程 10ms 主处理中 PSWDataToRte() 之后一致 */
    PSWdebug();
}

#else   /* TEST_DEAL_ENABLE == 0: 空入口, 保证 cmn.c 的调用照常链接 */

void TestDeal_Early(void) { }
void TestDeal_Mid(void)   { }
void TestDeal_Late(void)  { }

#endif /* TEST_DEAL_ENABLE */
