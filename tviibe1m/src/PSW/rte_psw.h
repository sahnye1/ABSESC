/**
 * @file    rte_psw.h
 * @brief   RTE PSW 层实现 — 驾驶室版 (Cabin Version)
 *
 * @details 实现 RTE.h 中声明的所有 PSW 层函数:
 *          - 17 路阀 PWM 动作封装 (InVal/OutVal/ASRVal 映射到 ValvePwm_SetDuty)
 *          - 继电器/UB/低边开关/接插件/CAN/flash/SCC3000 实现
 *          - 阀故障状态同步 (由 psw_data.c:PSWDataToRte 集中处理)
 *          - 报警灯控制 (ABS/HSA/ASR) 由 COM 层实现
 *
 * @note    本文件对应底盘版 PSW 库的角色，驾驶室版自行实现。
 *          P0: 阀动作封装 (完整实现)
 *          P1: CAN 接口 (已对接 can_driver)
 *          P1: GPIO (继电器/UB/低边开关/接插件/报警灯 — 已对接 gpio.h)
 *          P1: Flash/EEPROM (已对接 eeprom.c)          
 */

#ifndef RTE_PSW_H
#define RTE_PSW_H

/* RTE.h 已声明所有对外函数，本头文件仅作模块内部使用 */

/* 版本号初始化: 给 RTEPSW_Version 赋值 (定义在 RTE.c, 不重复定义) */
void RtePsw_VersionInit(void);

#endif /* RTE_PSW_H */
