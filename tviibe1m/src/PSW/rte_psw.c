/**
 * @file    rte_psw.c
 * @brief   RTE PSW 层实现 — 驾驶室版
 *
 * @details 17 阀 → BTS724G + ValvePwm 映射, RTE 0.1ms → 10μs/步换算。
 *          PWM 双比较模式: CC0→SET(起点) CC1→CLEAR(终点) overflow→CLEAR(基态)
 *          进气阀(后段高): CC0=period-htime, CC1=period
 *          排气/ASR(前段高): CC0=0(周期起点), CC1=htime
 */

#include "RTE.h"
#include "bts724g.h"
#include "valve_pwm.h"
#include "eeprom.h"
#include "gpio.h"
#include "psw_data.h"
#include <string.h>

/* ========================================================================== */
/*  常量与宏                                                                   */
/* ========================================================================== */

/* RTE 0.1ms → ValvePwm 10μs 步数 */
#define RTE_TO_PWM_STEPS(v)      ((uint16_t)((v) * 10u))

/* RTE 层周期上限 (单位 0.1ms): 5000 = 500ms */
#define RTE_PERIOD_MAX           5000u

/** @brief RTE 阀控参数钳位 (上限 / 非法关阀 / 占空比钳位)
 *  @note  无钳位时 htime > period 会使 cc0_start 下溢成大数 → 底层直接 return
 *         → 阀完全不动作且无任何提示 (静默失效)。 */
static void Psw_ValActClamp(uint16_t *period, uint16_t *htime)
{
    if (*period > RTE_PERIOD_MAX) { *period = RTE_PERIOD_MAX; }   /* 周期上限   */
    if (*period == 0u)            { *period = 1u; *htime = 0u; }  /* 非法 → 关阀 */
    else if (*htime > *period)    { *htime  = *period; }          /* 占空比钳位 */
}

/* ========================================================================== */
/*  全局/静态状态                                                              */
/* ========================================================================== */

/* ========================================================================== */
/*  内部辅助函数                                                               */
/* ========================================================================== */

/** @brief 进气阀 PWM (后段高, LOW→HIGH): CC0=period-htime, CC1=period */
static void Psw_InValAct(valve_id_t vid, uint16_t period, uint16_t htime)
{
    if (vid >= VALVE_NUM_TOTAL) return;

    Psw_ValActClamp(&period, &htime);

    if (htime == 0u)
    {
        Bts724g_NotifyValveActive(vid, false);
        Bts724g_SetValve(vid, false);
        return;
    }

    uint16_t pwm_period = RTE_TO_PWM_STEPS(period);
    uint16_t pwm_htime  = RTE_TO_PWM_STEPS(htime);
    uint16_t cc0_start  = pwm_period - pwm_htime;

    Bts724g_NotifyValveActive(vid, true);
    /* 运行中动态调参: 参数立即生效 + 保持相位连续。
     * 参数未变化由内部幂等短路直接返回, 不会反复重置周期。 */
    ValvePwm_SetDutyLive(vid, pwm_period, cc0_start, pwm_htime);
}

/** @brief 排气/ASR 阀 PWM (前段高, HIGH→LOW): CC0=OFFSET(0, 周期起点), CC1=htime */
static void Psw_OutValAct(valve_id_t vid, uint16_t period, uint16_t htime)
{
    if (vid >= VALVE_NUM_TOTAL) return;

    Psw_ValActClamp(&period, &htime);

    if (htime == 0u)
    {
        Bts724g_NotifyValveActive(vid, false);
        Bts724g_SetValve(vid, false);
        return;
    }

    uint16_t pwm_period = RTE_TO_PWM_STEPS(period);
    uint16_t pwm_htime  = RTE_TO_PWM_STEPS(htime);

    Bts724g_NotifyValveActive(vid, true);
    /* 同 Psw_InValAct: 立即生效 + 相位连续 (参数未变由幂等短路挡掉) */
    ValvePwm_SetDutyLive(vid, pwm_period, PWM_CC0_OFFSET, pwm_htime);
}

/* ========================================================================== */
/*  ASR 低边开关 (委托 gpio.c 实现)                                            */
/* ========================================================================== */

void     ASRFLowSideSw(uint32_t sw)       { Gpio_ASRFLowSideSw(sw); }
uint32_t ASRFLowSideSt(void)              { return Gpio_ASRFLowSideSt(); }
void     ASRRLowSideSw(uint8_t sw)        { Gpio_ASRRLowSideSw(sw); }
uint32_t ASRRLowSideSt(void)             { return Gpio_ASRRLowSideSt(); }

/* ========================================================================== */
/*  ABS 阀控制                                                                 */
/* ========================================================================== */

void InValActFL(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_LFI, period, htime); }
void OutValActFL(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_LFO, period, htime); }

void InValActFR(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_RFI, period, htime); }
void OutValActFR(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_RFO, period, htime); }

void InValActRL(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_LRI, period, htime); }
void OutValActRL(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_LRO, period, htime); }

void InValActRR(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_RRI, period, htime); }
void OutValActRR(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_RRO, period, htime); }

void InValActXL(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_LXI, period, htime); }
void OutValActXL(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_LXO, period, htime); }

void InValActXR(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_RXI, period, htime); }
void OutValActXR(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_RXO, period, htime); }

void InValActTr(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_TRFIN, period, htime); }
void OutValActTr(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_TRFOUT, period, htime); }

/* ========================================================================== */
/*  ASR 阀控制                                                               */
/* ========================================================================== */

void ASRValActF(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_FA_ASR, period, htime); }
void ASRValActR(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_DA_ASR, period, htime); }
void ASRValActX(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_TR_ASR, period, htime); }

/* ========================================================================== */
/*  继电器 / UB / 接插件引脚 (委托 gpio.c 实现)                                */
/* ========================================================================== */

void RelayCtrl(uint8_t state)       { Gpio_RelayCtrl(state); }
void UBCtrl(uint8_t state)          { Gpio_UBCtrl(state); }
void X1_2_staSet(uint8_t setValue)  { Gpio_HSALampCtrl(setValue); }
void X1_13_staSet(uint8_t setValue) { Gpio_ASRLampCtrl(setValue); }
void X1_15_staSet(uint8_t setValue) { Gpio_ABSWLampCtrl(setValue); }
int8_t Xn_pin_StaGet(uint8_t xNum, uint8_t xPin) { return Gpio_Xn_pin_StaGet(xNum, xPin); }

/* ========================================================================== */
/*  版本信息                                                                   */
/* ========================================================================== */

/**
 * @brief PSW 层版本号数组 RTEPSW_Version 定义 (唯一定义, 随 PSW 库交付)
 * @note  由底层 PSW 库定义并维护, 上层不再定义, RTE.h 中仅保留 extern 声明供上层引用。
 *        底层每次修改按当天日期更新 [1][2][3] = 年月日 BCD 码, [4] = 当天第 N 次修改。
 */
uint8_t RTEPSW_Version[5];

/**
 * @brief 初始化 RTEPSW_Version (BCD 码 {底层01, 年低位, 月, 日, 修改当天版本号})
 * @note  定义在本文件上方, 由底层 PSW 库维护 (上层不再定义)。
 *        每次修改底层工程按当天日期更新, [4] 为当天第 N 次修改 (次日归 0x01)。
 */
void RtePsw_VersionInit(void)
{
    RTEPSW_Version[0] = 0x01u;   /* 底层01*/
    RTEPSW_Version[1] = 0x26u;   /* 年低位 26 → 2026 */
    RTEPSW_Version[2] = 0x10u;   /* 月 */
    RTEPSW_Version[3] = 0x10u;   /* 日 */
    RTEPSW_Version[4] = 0x01u;   /* 当天第 1 次修改 */
}

uint8_t psw_version_get(uint8_t *verStr, uint8_t bufLen)
{
    if (bufLen < 8u) return 0u;
    const char version[] = "PSW_CAB_V1.0.0";
    uint8_t len = (uint8_t)sizeof(version);
    if (len > bufLen) len = bufLen;
    (void)memcpy(verStr, version, len);
    return len;
}

/* ========================================================================== */
/*  EEPROM 读写 (5 分区: ERR_CODE / ASW / PSW / COM / BOOT)                    */
/*                                                                            */
/*  故障上报下沉到 eeprom.c Eeprom_Write/Read, 本层纯薄封装                     */
/* ========================================================================== */

uint32_t ERR_CODE_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_ERR_CODE_BASE, wAddr, data, len, EEPROM_ERR_CODE_SIZE); }
uint32_t ERR_CODE_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_ERR_CODE_BASE, rAddr, buf, len, EEPROM_ERR_CODE_SIZE); }

uint32_t ASW_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_ASW_BASE, wAddr, data, len, EEPROM_ASW_CFG_SIZE); }
uint32_t ASW_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_ASW_BASE, rAddr, buf, len, EEPROM_ASW_CFG_SIZE); }

uint32_t PSW_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_PSW_BASE, wAddr, data, len, EEPROM_PSW_SIZE); }
uint32_t PSW_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_PSW_BASE, rAddr, buf, len, EEPROM_PSW_SIZE); }

uint32_t COM_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_COM_BASE, wAddr, data, len, EEPROM_COM_SIZE); }
uint32_t COM_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_COM_BASE, rAddr, buf, len, EEPROM_COM_SIZE); }

uint32_t BOOT_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_BOOT_BASE, wAddr, data, len, EEPROM_BOOT_SIZE); }
uint32_t BOOT_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_BOOT_BASE, rAddr, buf, len, EEPROM_BOOT_SIZE); }

/* ========================================================================== */
/*  SCC3000 (ESC 传感器) 桩函数 — 已废弃                                        */
/*                                                                            */
/*  SCC3000/IMU 驱动不在本工程范围, ESC 传感器数据 (YawRate/LatAcc/LongiAcc 等) */
/*  由外部模块负责。此处仅为满足 COM 层链接提供桩函数。                           */
/* ========================================================================== */

/** @brief ESC 标定请求 (桩: 无实际操作) */
void SCC3000_calibration_req(void)
{
    (void)0;
}

/** @brief ESC 反标定请求 (桩: 无实际操作) */
void SCC3000_UnCalibration_req(void)
{
    (void)0;
}

/** @brief ESC 标定结果反馈 (桩: 恒返回 0 = 无标定流程) */
uint8_t SCC3000_calibration_resp(void)
{
    return 0u;
}

/** @brief 获取 ESC 是否故障 (桩: 恒返回 0 = 无故障) */
uint8_t SCC3000_Err(void)
{
    return 0u;
}

/** @brief 获取 ECU 类型 (桩: 恒返回 0 = 驾驶室版, 待后续按配置实现) */
uint8_t get_ECU_type(void)
{
    return 0u;
}
