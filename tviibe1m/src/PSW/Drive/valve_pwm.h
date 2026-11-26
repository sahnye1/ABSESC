/**
 * @file    valve_pwm.h
 * @brief   TCPWM 电磁阀 PWM 控制 (CC0+CC1 双比较模式, 覆盖全部 17 阀)
 * @note    双比较模式: CC0→SET(脉冲起点), CC1→CLEAR(脉冲终点)
 *          周期 = period 步, 高电平 = high_time 步 (1步=10μs @100kHz)
 *          period 范围: 1000~50000 (10ms~500ms) 为常规控阀推荐值;
 *                       硬上限 65535 步 = 655.35ms (16bit 计数),
 *                       VHT 保压标定测试用到 52000 步 (520ms)
 *          SetDuty: 直写 CC0/CC1/PERIOD 后强制 counter 溢出生效 → 零延迟
 *          全部 17 阀走 TCPWM0 (15 GRP0 + 2 GRP1),
 *          Bts724g_SetValve() 自动分发到 PWM 路径。
 */

#ifndef VALVE_PWM_H
#define VALVE_PWM_H

#include "bts724g.h"

/* "前段高"(排气/ASR 阀) 的脉冲起点 = 周期起点 (0)
 *  ⚠ 必须为 0, 禁止改回非 0: 非 0 时每个"恒高"周期都会在起点丢一段高电平
 *    (用 5 时排气阀实测占空仅 ~70% → 保持不住、保压泄漏; 2026-09-15 台架确诊)
 *  原理: 置 0 后 counter=0 处 CC0 的 SET 覆盖 overflow 的 CLEAR → 真恒高、无毛刺 */
#define PWM_CC0_OFFSET  0u

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
 * @brief   动态调参 — 参数立即生效 + 相位连续 (运行中改 period/htime 用)。
 * @param   valve_id   阀索引
 * @param   period     周期步数 (1 步 = 10μs @100kHz), 推荐 1000~50000 (10ms~500ms)
 * @param   cc0_start  脉冲起点 (SET 位置), CC0 写入值
 *                     ⇢ 进气阀 (后段高): period - htime
 *                     ⇢ 排气阀 (前段高): PWM_CC0_OFFSET (0, 周期起点)
 * @param   high_time  高电平持续步数, CC1 = cc0_start + high_time
 *                     0=全关, high_time≥period → 恒 HIGH
 *
 * @details 写新参数并归零建立基态 → 按需补一次 CC0(SET) 事件 → 恢复原 counter。
 *          效果: 新参数立即生效, 且不丢失当前相位 (不会截断脉冲, 也不会让高电平延续过久)。
 *
 * @note    每步之间延时 11μs (≥1 拍), 总耗时 ≤44μs, 且仅参数变化时发生;
 *          参数未变化由内部幂等短路直接返回。请勿在中断中调用。
 */
void ValvePwm_SetDutyLive(valve_id_t valve_id, uint16_t period,
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
