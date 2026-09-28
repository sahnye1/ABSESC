/**
 * @file    psw_data.h
 * @brief   PSW 中间变量层 — 驱动 → PSW* → RTE* 数据转发
 *
 * @details 各驱动模块写入 PSW* 中间变量, PSWDataToRte() 统一读取
 *          PSW* 并填充 RTE* 全局变量 (含数据验证/掩蔽)。
 *          BSWdebug 测试用例可直接读取 PSW* 变量。
 */

#ifndef PSW_DATA_H
#define PSW_DATA_H

#include <stdint.h>

/* ========================================================================== */
/*  核心数值                                                                   */
/* ========================================================================== */

extern uint8_t  PSWTaskTime;         /* 主任务执行时间, 0.1ms */
extern uint16_t PSWvIgn;             /* VPOWER 电压, fact:0.1V */

/* ========================================================================== */
/*  压力传感器                                                                 */
/* ========================================================================== */

extern uint16_t PSWBrkPreX2_7;       /* X2_7脚压力值, kpa */
extern uint16_t PSWBrkPreX3_10;      /* X3_10脚压力值, kpa */
extern uint8_t  PSWfErrPreX2_7;      /* X2_7脚压力故障, 0=正常 1=故障 */
extern uint8_t  PSWfErrPreX3_10;     /* X3_10脚压力故障 */
extern uint8_t  PSWfErrPreF;         /* 前桥压力故障 */
extern uint8_t  PSWfErrPreR;         /* 后桥压力故障 */

/* ========================================================================== */
/*  轮速 (0.1km/h)                                                             */
/* ========================================================================== */

extern uint16_t PSWWheelSpeedFL;
extern uint16_t PSWWheelSpeedFR;
extern uint16_t PSWWheelSpeedRL;
extern uint16_t PSWWheelSpeedRR;
extern uint16_t PSWWheelSpeedXL;
extern uint16_t PSWWheelSpeedXR;

/* ========================================================================== */
/*  轮速传感器故障 (每通道: 开路/短路/间隙过大)                                 */
/* ========================================================================== */

extern uint8_t PSWErrWssOpenFL;
extern uint8_t PSWErrWssShortFL;
extern uint8_t PSWErrWssGapFL;

extern uint8_t PSWErrWssOpenFR;
extern uint8_t PSWErrWssShortFR;
extern uint8_t PSWErrWssGapFR;

extern uint8_t PSWErrWssOpenRL;
extern uint8_t PSWErrWssShortRL;
extern uint8_t PSWErrWssGapRL;

extern uint8_t PSWErrWssOpenRR;
extern uint8_t PSWErrWssShortRR;
extern uint8_t PSWErrWssGapRR;

extern uint8_t PSWErrWssOpenXL;
extern uint8_t PSWErrWssShortXL;
extern uint8_t PSWErrWssGapXL;

extern uint8_t PSWErrWssOpenXR;
extern uint8_t PSWErrWssShortXR;
extern uint8_t PSWErrWssGapXR;

/* ========================================================================== */
/*  ABS 阀故障 (开路/短路, 按轴+进/排)                                          */
/* ========================================================================== */

extern uint8_t PSWErrInValOpenFL;     extern uint8_t PSWErrInValShortFL;
extern uint8_t PSWErrOutValOpenFL;    extern uint8_t PSWErrOutValShortFL;

extern uint8_t PSWErrInValOpenFR;     extern uint8_t PSWErrInValShortFR;
extern uint8_t PSWErrOutValOpenFR;    extern uint8_t PSWErrOutValShortFR;

extern uint8_t PSWErrInValOpenRL;     extern uint8_t PSWErrInValShortRL;
extern uint8_t PSWErrOutValOpenRL;    extern uint8_t PSWErrOutValShortRL;

extern uint8_t PSWErrInValOpenRR;     extern uint8_t PSWErrInValShortRR;
extern uint8_t PSWErrOutValOpenRR;    extern uint8_t PSWErrOutValShortRR;

extern uint8_t PSWErrInValOpenXL;     extern uint8_t PSWErrInValShortXL;
extern uint8_t PSWErrOutValOpenXL;    extern uint8_t PSWErrOutValShortXL;

extern uint8_t PSWErrInValOpenXR;     extern uint8_t PSWErrInValShortXR;
extern uint8_t PSWErrOutValOpenXR;    extern uint8_t PSWErrOutValShortXR;

extern uint8_t PSWErrInValOpenTr;     extern uint8_t PSWErrInValShortTr;
extern uint8_t PSWErrOutValOpenTr;    extern uint8_t PSWErrOutValShortTr;

/* ========================================================================== */
/*  ASR 阀故障                                                                 */
/* ========================================================================== */

extern uint8_t PSWErrASRValveOpenF;   extern uint8_t PSWErrASRValveShortF;
extern uint8_t PSWErrASRValveOpenR;   extern uint8_t PSWErrASRValveShortR;
extern uint8_t PSWErrASRValveOpenX;   extern uint8_t PSWErrASRValveShortX;

/* ========================================================================== */
/*  驱动芯片故障                                                               */
/* ========================================================================== */

extern uint8_t PSWErrOpenDrive724_U6;
extern uint8_t PSWErrOpenDrive724_U9;
extern uint8_t PSWErrOpenDrive724_U12;
extern uint8_t PSWErrOpenDrive724_U13;
extern uint8_t  PSWErrOpenDrive724_U19;
extern uint8_t PSWErrShortDrive724_U6;
extern uint8_t PSWErrShortDrive724_U9;
extern uint8_t PSWErrShortDrive724_U12;
extern uint8_t PSWErrShortDrive724_U13;
extern uint8_t PSWErrShortDrive724_U19;
extern uint8_t PSWErrDriveChip;       /* 芯片总故障: >0 = 故障 */

/* ========================================================================== */
/*  VPOWER 电源故障 (仅 VPOWER 一路, 无独立电池检测)                             */
/* ========================================================================== */

extern uint8_t PSWErrVpwrLo;          /* VPower 低压 */
extern uint8_t PSWErrVpwrHi;          /* VPower 高压 */

/* ========================================================================== */
/*  EEPROM 故障 (原 Flash, 随 RTE.h 改名为 EEprom)                              */
/* ========================================================================== */

extern uint8_t PSWfErrEEprom;         /* EEPROM 操作故障 */
extern uint8_t PSWfErrEEpromOutBnd;   /* EEPROM 写超范围故障 */
extern uint8_t PSWErrRelay;           /* 继电器故障 */

/* ========================================================================== */
/*  重启原因                                                                   */
/* ========================================================================== */

extern uint32_t resetReason;     //模块重启的原因

/* ========================================================================== */
/*  函数声明                                                                   */
/* ========================================================================== */

/**
 * @brief 从各驱动模块刷新 PSW* 中间变量 (每 10ms 主循环调用)
 *
 * @details 按顺序读取 sar1_adc / wheel_speed / sensor_diag / bts724g / cmn
 *          的内部状态, 填充所有 PSW* 变量。
 */
void PSWData_Refresh(void);

/**
 * @brief 集中式 RTE 填充 (从 PSW* 中间变量读取, 验证后写入 RTE)
 */
void PSWDataToRte(void);

#endif /* PSW_DATA_H */
