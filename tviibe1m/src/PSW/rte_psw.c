/**
 * @file    rte_psw.c
 * @brief   RTE PSW 层实现 — 驾驶室版
 *
 * @details 17 阀 → BTS724G + ValvePwm 映射, RTE 0.1ms → 10μs/步换算。
 *          PWM 双比较模式: CC0→SET(起点) CC1→CLEAR(终点) overflow→CLEAR(基态)
 *          进气阀(后段高): CC0=period-htime, CC1=period
 *          排气/ASR(前段高): CC0=5, CC1=5+htime
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

/* ========================================================================== */
/*  全局/静态状态                                                              */
/* ========================================================================== */

/* ========================================================================== */
/*  内部辅助函数                                                               */
/* ========================================================================== */

/** @brief 进气阀 PWM (后段高, LOW→HIGH): CC0=period-htime, CC1=period */
static void Psw_InValAct(valve_id_t vid, uint16_t period, uint16_t htime)
{
    if (period == 0u || htime == 0u)
    {
        Bts724g_NotifyValveActive(vid, false);
        Bts724g_SetValve(vid, false);
        return;
    }

    uint16_t pwm_period = RTE_TO_PWM_STEPS(period);
    uint16_t pwm_htime  = RTE_TO_PWM_STEPS(htime);
    uint16_t cc0_start  = pwm_period - pwm_htime;

    Bts724g_NotifyValveActive(vid, true);
    ValvePwm_SetDutyNoReset(vid, pwm_period, cc0_start, pwm_htime);
}

/** @brief 排气/ASR 阀 PWM (前段高, HIGH→LOW): CC0=OFFSET(5), CC1=5+htime */
static void Psw_OutValAct(valve_id_t vid, uint16_t period, uint16_t htime)
{
    if (period == 0u || htime == 0u)
    {
        Bts724g_NotifyValveActive(vid, false);
        Bts724g_SetValve(vid, false);
        return;
    }

    uint16_t pwm_period = RTE_TO_PWM_STEPS(period);
    uint16_t pwm_htime  = RTE_TO_PWM_STEPS(htime);

    Bts724g_NotifyValveActive(vid, true);
    ValvePwm_SetDutyNoReset(vid, pwm_period, PWM_CC0_OFFSET, pwm_htime);
}

/* ========================================================================== */
/*  ASR 低边开关 (委托 gpio.c 实现)                                            */
/* ========================================================================== */

void     ASRFLowSideSw(uint32_t sw)       { Gpio_ASRFLowSideSw(sw); }
uint32_t ASRFLowSideSt(void)              { return Gpio_ASRFLowSideSt(); }
void     ASRRLowSideSw(uint8_t sw)        { Gpio_ASRRLowSideSw(sw); }
uint32_t ASRRLowSideSt(void)             { return Gpio_ASRRLowSideSt(); }
void     ASRFLowSideEnable(uint8_t e)     { Gpio_ASRFLowSideEnable(e); }
void     ASRRLowSideEnable(uint8_t e)     { Gpio_ASRRLowSideEnable(e); }

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
 * @brief 初始化 RTEPSW_Version (BCD 码 {年高位, 年低位, 月, 日, 修改当天版本号})
 * @note  定义在 RTE.c (上层维护), 本函数只赋值, 不重复定义。
 *        每次修改底层工程按当天日期更新, [4] 为当天第 N 次修改 (次日归 0x01)。
 */
void RtePsw_VersionInit(void)
{
    RTEPSW_Version[0] = 0x20u;   /* 年高位 20 */
    RTEPSW_Version[1] = 0x26u;   /* 年低位 26 → 2026 */
    RTEPSW_Version[2] = 0x09u;   /* 月 09 */
    RTEPSW_Version[3] = 0x28u;   /* 日 28 */
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
    { return Eeprom_Write(EEPROM_ASW_BASE, EEPROM_ERR_CODE_OFFSET + wAddr, data, len, EEPROM_ASW_SIZE); }
uint32_t ERR_CODE_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_ASW_BASE, EEPROM_ERR_CODE_OFFSET + rAddr, buf, len, EEPROM_ASW_SIZE); }

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
