/**
 * @file    valve_pwm.h
 * @brief   TCPWM 电磁阀 PWM 控制 (CC0+CC1 双比较模式, 覆盖全部 17 阀)
 * @note    双比较模式: CC0→SET(脉冲起点), CC1→CLEAR(脉冲终点)
 *          周期 = period 步, 高电平 = high_time 步 (1步=10μs @100kHz)
 *          period 范围: 1000~50000 (10ms~500ms)
 *          SetDuty: 直写 CC0/CC1/PERIOD 后强制 counter 溢出生效 → 零延迟
 *          全部 17 阀走 TCPWM0 (15 GRP0 + 2 GRP1),
 *          Bts724g_SetValve() 自动分发到 PWM 路径。
 */

#ifndef VALVE_PWM_H
#define VALVE_PWM_H

#include "bts724g.h"

/* CC0 基础偏移: 排气阀/ASR 阀 (前段高) 的脉冲起点, 避免 counter=0 race */
#define PWM_CC0_OFFSET  5u

/* ========================================================================== */
/*  PWM 阀查表 (外部模块判断阀是否为 PWM 模式)                                  */
/* ========================================================================== */
bool ValvePwm_IsPwmValve(valve_id_t valve_id);

/* ========================================================================== */
/*  初始化                                                                      */
/* ========================================================================== */

/**
 * @brief   初始化 PWM 阀的 TCPWM 时钟和 GPIO。
 *          由 Bts724g_Init() 最后调用。
 */
void ValvePwm_Init(void);

/* ========================================================================== */
/*  控制接口                                                                    */
/* ========================================================================== */

/**
 * @brief   设置阀的 PWM 参数 (周期 + 脉冲起点 + 高电平时间)。
 * @param   valve_id   阀索引
 * @param   period     周期步数 (1 步 = 10μs @100kHz), 推荐 1000~50000 (10ms~500ms)
 * @param   cc0_start  脉冲起点 (SET 位置), CC0 写入值
 *                     ⇢ 进气阀 (后段高): period - htime
 *                     ⇢ 排气阀 (前段高): PWM_CC0_OFFSET (5)
 * @param   high_time  高电平持续步数, CC1 = cc0_start + high_time
 *                     0=全关, high_time≥period → 恒 HIGH
 *
 * @note    CC1 = cc0_start + high_time, overflow → CLEAR (基态 LOW)
 *          仅对 PWM 模式阀有效; GPIO 阀此调用无操作。
 */
void ValvePwm_SetDuty(valve_id_t valve_id, uint16_t period,
                      uint16_t cc0_start, uint16_t high_time);

/**
 * @brief   动态调参 — 不清 counter 的 SetDuty (周期边界平滑衔接)。
 * @param   参数同 ValvePwm_SetDuty。
 * @note    与 SetDuty 的区别: SetDuty 写后强制 counter 归零立即重开新周期;
 *          本函数【不清 counter】, counter 保持当前值继续跑, 到新周期终点才溢出。
 *          适用于 ABS 控阀在 PWM 运行中无缝改周期/占空比。
 *          边界: 新周期 < 当前 counter 值时会立即 overflow (等同立即切换)。
 */
void ValvePwm_SetDutyNoReset(valve_id_t valve_id, uint16_t period,
                             uint16_t cc0_start, uint16_t high_time);

/**
 * @brief   开关 PWM 阀 (on: 100% duty, off: 0% duty)。
 * @param   valve_id  阀索引
 * @param   on        true=全开, false=全关
 *
 * @note    由 Bts724g_SetValve() 自动分发调用。
 */
void ValvePwm_SetOnOff(valve_id_t valve_id, bool on);

#endif /* VALVE_PWM_H */
