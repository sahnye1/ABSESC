/**
 * @file    cmn.h
 * @brief   PSW 公共模块头文件 — 系统级宏定义 + 工具函数声明
 *
 * @details 从旧版 PSW_old/cmn.h 中提取仍在使用或通用的定义:
 *          - 公共宏 (PI / DIV_ROUND_UP)
 *          - SRAM1 起始地址
 *          - 工具函数声明 (喂狗/复位/升级/延时等)
 *
 * @note    GPIO 引脚定义统一在 gpio.h, 各模块专用配置 (PWM/CAN/ADC/SPI)
 *          在各自模块头文件, 不在此处重复。
 */

#ifndef __CMN_H__
#define __CMN_H__

#include "cy_project.h"
#include "cy_device_headers.h"

/* ---- 公共宏 ---- */
#define PI                  (3.141592654f)      /* 圆周率 */
#define DIV_ROUND_UP(a, b)  (((a) + (b) / 2) / (b))  /* 四舍五入整数除法 */

/* ---- SRAM1 起始地址 (system_update_start 用) ---- */
#define RAM_1_ADDRESS       0x08010000

/* ---- 看门狗跳过标记 (BSWdebug 测试用例用) ---- */
extern uint8_t g_skipSoftWD;   /* 1 = 停喂软狗, 0 = 正常 */
extern uint8_t g_skipHardWD;   /* 1 = 停喂硬狗, 0 = 正常 */

/* ---- 函数声明 ---- */

/**
 * @brief 外设时钟分频器 (动态读取 clk_peri, 按目标频率自动计算分频值)
 * @param ipBlock    外设时钟目标 (en_clk_dst_t)
 * @param div_type   分频器类型 (8-bit / 16-bit / 24.5-bit 等)
 * @param clk_no     分频器编号 (uint32_t, 兼容各类分频器编号空间)
 * @param targetFreq 目标频率 (Hz), 不硬编码源频率
 * @note  改标定/晶振后各外设时钟自动跟随, 无需手动改分频值。
 */
void periph_divider(en_clk_dst_t ipBlock, cy_en_divider_types_t div_type, uint32_t clk_no, uint32_t targetFreq);

/**
 * @brief 微秒级延时 (基于 SysTick)
 * @param n 延时微秒数
 */
void delay_us(uint32_t n);

/**
 * @brief 软件看门狗初始化
 * @note  上限 2000ms, 调试模式下继续计数
 */
void SoftWD_init(void);

/**
 * @brief 双看门狗喂狗
 * @note  硬件看门狗: WD_HW_GPIO (P5.0) 翻转
 *        软件看门狗: Cy_WDT_ClearWatchdog()
 * @note  与 PSW/wd_feed.h 中的 static inline wd_feed() (引脚 P5.0) 等价,
 *        当前 cmn.c 版本供主流程调用, wd_feed.h 版本供 PSW 内部模块调用。
 */
void wd_feed(void);

/**
 * @brief 获取系统上次复位原因
 * @return 1 = 看门狗复位; 0 = 其他复位 (上电/软件/硬件复位等)
 * @note  调用后会自动清除复位原因寄存器 (仅返回一次有效)
 */
uint8_t sysRstReason_get(void);

/**
 * @brief 系统升级启动 (Bootloader 入口)
 * @param updateFlag 升级标志, 写入 SRAM1 起始地址
 * @note  先将标志写入 SRAM1 (保持数据), 再触发 NVIC_SystemReset。
 *        Bootloader 复位后检查 SRAM1 地址, 若标志匹配则进入升级流程。
 *        此函数不会返回。
 */
void system_update_start(uint32_t updateFlag);

/**
 * @brief 系统软复位
 * @note 等同于 NVIC_SystemReset()
 */
void system_reboot(void);

/**
 * @brief 板级初始化 — SystemInit + 所有模块 Init (不含 CAN)
 *
 * @details 替代旧版 PSW_old/cmn.c board_init(), 使用新版驱动架构:
 *          SCC3000 → 废弃, pwm_comp → Bts724g_Init,
 *          pwm_cap → WheelSpeed_Init, adc → SensorDiag_Init + Sar1Adc_Init
 *
 * @note    CAN 初始化 (Can0_Init/Can1_Init) 由应用层在 board_init() 之前调用,
 *          因为需要应用层下发的过滤表。
 *
 *          典型调用顺序:
 *            Can0_Init(filter0, n0);
 *            Can1_Init(filter1, n1);
 *            board_init();
 *            for(;;) { if(g_b10msTick) PSW_10ms_deal(); }
 */
void board_init(void);

/**
 * @brief 10ms 周期主处理 — 喂狗 + 全部诊断 + 轮速 + RTE + busoff 恢复
 *
 * @details 替代旧版 PSW_old/cmn.c PSW_10ms_deal(), 使用新版驱动架构:
 *          SCC3000/SCC3134 ESC → ADC 轮询 (sensor_diag)
 *          DMA 轮速 → TCPWM 中断 (wheel_speed)
 *          FarErrorCheck → ValveDiag_Process (bts724g)
 *          adc_calc_all_result → Sar1Adc_Read (sar1_adc)
 *
 * @note    main_cm4.c 主循环可简化为:
 *          if (g_b10msTick) { PSW_10ms_deal(); }
 */
void PSW_10ms_deal(void);

/**
 * @brief RTE 变量 CAN 打印监控 (调试用)
 * @note  仅在 RTE_DBG_PRINT_ENABLE != 0 时编译实现。用调试 CAN (CAN1,
 *        无发送白名单) 周期打印 RTE 变量, 监控 PSWDataToRte() 是否正常
 *        更新 RTE。7 帧轮转: 帧0~2 数值 (VPOWER/压力/轮速),
 *        帧3~6 全部故障标志 (轮速传感器/ABS+ASR 阀/压力/电源/Flash/
 *        Relay/DriveChip/ESCM) + EEPROM 状态。
 */
void RTE_CanPrint_Monitor(void);

/**
 * @brief PSW 变量 CAN 打印监控 (调试用)
 * @note  仅在 PSW_DBG_PRINT_ENABLE != 0 时编译实现。用调试 CAN (CAN1,
 *        无发送白名单) 周期打印 PSW* 中间变量, 监控驱动层 → PSW* 更新。
 *        8 帧轮转: 帧0~2 数值 (任务时间/VPOWER/压力/轮速),
 *        帧3~6 全部故障标志 (轮速传感器/ABS+ASR 阀/驱动芯片),
 *        帧7 压力/电源/Relay/Flash 故障 + 重启原因。
 */
void PSW_CanPrint_Monitor(void);

#endif /* __CMN_H__ */
