/**
 * @file    PSWdebug.c
 * @brief   测试用例框架 — 函数指针表 + CAN 动态激活 + 10ms 周期执行
 *
 * @details 激活机制 (PSWdebug 每 10ms 调用):
 *          ① 读取 RTEfDbgMsgSW3 (CAN 下发的测试编号)
 *          ② 匹配 g_debugCase_list[] → 注册函数指针到 g_debugFuncList[]
 *          ③ 遍历 g_debugFuncList[], 逐个执行已注册的测试函数
 *
 * @details 测试编号约定 (_XX_XXX_testCase):
 *          +-------+-----------+----------------------------------+
 *          | 编号   | 大类       | 说明                             |
 *          +--------+-----------+----------------------------------+
 *          | 0x     | 通信       | 00=CAN0 01=CAN1 02=UART          |
 *          | 1x     | 系统       | 10=软件复位                       |
 *          | 2x     | 看门狗     | 20=软狗 21=硬狗                   |
 *          | 3x     | 定时器     | 30=10ms timer                    |
 *          | 4x     | 电源       | 40=低边开关(ASR) 41=UB供电        |
 *          | 6x     | EEPROM     | 60=读写测试                       |
 *          | 7x     | 传感器     | 70=VPOWER 71~76=轮速 77~78=压力   |
 *          | 8x     | 轮速值     | 80~83=4通道计算值                  |
 *          | 9x/Ax  | 阀PWM控制  | 90=全阀PWM扫描, 91=芯片+阀故障上报    |
 *          +--------+-----------+----------------------------------+
 *
 * @note    框架版本, 各测试函数体留空 (TODO), 后续逐步填充实现。
 *          
 */

#include "cmn.h"
#include "can.h"
#include "wheel_speed.h"
#include "adc.h"
#include "bts724g.h"
#include "spi_eeprom.h"
#include "eeprom.h"
#include "rte_psw.h"
#include "psw_data.h"
#include "gpio.h"
#include "RTE.h"
#include <string.h>

/* ======================================================================== */
/*  框架常量和类型                                                             */
/* ======================================================================== */

#define BSW_SEND_ID_BASE    0x1CF00300u
#define MAX_DEBUG_CNT       15u

typedef void (*DebugFunc)(void);

typedef struct
{
    uint8_t  debugIndx;           /* 测试编号 (CAN 激活用的开关值) */
    DebugFunc debugFunc;          /* 测试函数指针                   */
    uint8_t  sendIdCnt;           /* 结果上报 CAN ID 数量           */
    uint32_t sendIdList[15];      /* 结果上报 CAN ID 列表           */
} debug_list_t;

/* ======================================================================== */
/*  激活运行时状态                                                             */
/* ======================================================================== */

static uint8_t   g_debugOpenCnt;                  /* 已激活的测试用例数     */
static DebugFunc g_debugFuncList[MAX_DEBUG_CNT];  /* 已激活函数指针数组     */
static uint8_t   g_debugOpenIndx[MAX_DEBUG_CNT];  /* 已激活的测试编号(防重) */

/* ======================================================================== */
/*  CAN 上报包装                                                               */
/* ======================================================================== */

/**
 * @brief 通过 CAN0 发送测试结果帧
 * @param id       相对于 BSW_SEND_ID_BASE 的偏移 ID
 * @param data     数据指针
 * @param dataLen  有效数据长度 (≤8)
 */
static void canPSWDbgMsgSend(uint32_t id, const uint8_t *data, uint8_t dataLen)
{
    uint8_t sendData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    for (uint8_t i = 0u; i < dataLen; i++)
    {
        sendData[i] = data[i];
    }
    CANDbgMsgSend(BSW_SEND_ID_BASE + id, sendData);
    //Can0_SendMsg(BSW_SEND_ID_BASE + id, 0, sendData, dataLen);
}

/* ======================================================================== */
/*  测试用例函数                                                               */
/* ======================================================================== */


/** @brief 02 — UART 通信 */
void _02_UART_testCase(void)
{
    /* TODO: UART  */
}

/* ---- 1x 系统 ---- */

/** @brief 10 — 软件复位 */
void _10_softReset_testCase(void)
{
    static uint8_t rstFlag = 0;
	rstFlag++;
	if (rstFlag < 100)
	{
		TestMark_Ctrl(1);
	}
	else
	{
		system_reboot();
	}
}

void _11_softReset_testCase()
{
	canPSWDbgMsgSend(0x11, (uint8_t *)&resetReason, 4);
}

void _20_WD_testCase(void)
{
    static uint32_t cnt = 0u;
    cnt++;
    if (cnt > 100u)
    {
        g_skipSoftWD = true;  /* 停喂软狗, 等待 WDT 超时复位 */
    }
}

void _21_WD_testCase(void)
{
    static uint32_t cnt = 0u;
    cnt++;
    if (cnt > 100u)
    {
        g_skipHardWD = true;  /* 停喂硬狗, 等待外置硬狗超时 */
    }
}


/** @brief 30 — 10ms timer 节拍验证 (翻转 X1_9/P8.1 观察 10ms 周期) */
void _30_10MStimer_testCase(void)
{
    static uint32_t timerCnt = 0;
    timerCnt++;
    timerCnt = (timerCnt > 1) ? 0 : 1;
    TestMark_Ctrl(timerCnt);
}

void _40_GPO_testCase(void)
{
    static uint8_t cnt = 0;
    uint8_t msg[8] = {0};
    cnt++;
    if (cnt >= 200u)
    {
        cnt = 0u;
    }

    if (cnt < 100u)
    {
        if (cnt == 0u)
        {
            ASRFLowSideSw(1u);   /* P2.0 ON: 前桥ASR + 挂车ABS */
            ASRRLowSideSw(1u);/* P6.2 ON: 后桥ASR */
            TestMark_Ctrl(1);
        }
    }
    else
    {
        if (cnt == 100u)
        {
            ASRFLowSideSw(0u);   /* 全部关闭 */
            ASRRLowSideSw(0u);
            TestMark_Ctrl(0);
        }
    }

    msg[0] = ASRFLowSideSt();    /* P2.0 当前状态 */
    msg[1] = ASRRLowSideSt();    /* P6.2 当前状态 */
    canPSWDbgMsgSend(0x40u, msg, 2u);
}

/** @brief 41 — 接插件开关状态读取 (X3-4 挂车ABS / X1-6 ESC / X1-5 坡起) */
void _41_GPI_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)Xn_pin_StaGet(3u, 4u);   /* X3-4: 挂车ABS开关 */
    data[1] = (uint8_t)Xn_pin_StaGet(1u, 6u);   /* X1-6: ESC开关 */
    data[2] = (uint8_t)Xn_pin_StaGet(1u, 5u);   /* X1-5: 坡起开关 */
    canPSWDbgMsgSend(0x41u, data, 3u);
}
void _42_GPI_testCase()
{
 	uint8_t gpiMsg[8] = {0};
 	gpiMsg[0] = PSWErrRelay;       //继电器故障 0-正常 1-故障
 	canPSWDbgMsgSend(0x42, gpiMsg, 1);
}

/* ---- 6x EEPROM ---- */

/** @brief 60 — EEPROM 读写功能测试 */
/**
 * @brief 60 — EEPROM 512 字节读写测试 (TestMark_Ctrl 示波器测时 + 读回校验)
 *
 * @details 1000 拍周期:
 *          +----------+--------------------------------------------------+
 *          | 拍数     | 操作                                             |
 *          +----------+--------------------------------------------------+
 *          |   0~ 99  | 初始化: 清零读缓冲 + 填充递增模式写缓冲          |
 *          |     100  | 写: TestMark_Ctrl↑ → Eeprom_WriteBlock(512B) → ↓ |
 *          |          |   返回值 512=写后读回 3 次全对, 经 CAN 0x60 上报  |
 *          | 101~199  | 等待                                             |
 *          |     200  | 读: TestMark_Ctrl↑ → Eeprom_ReadBlock(512B) → ↓ + 校验 |
 *          | 201~999  | 每拍 CAN 0x66 发 8 字节读回数据 (共 64 帧)      |
 *          +----------+--------------------------------------------------+
 * 正确性校验: 写阶段接 WriteBlock 返回值, 读阶段逐字节比较; 递增模式 buf[i]=i
 *   能暴露地址错位/数据线故障。CAN 0x60 校验帧(4B): [stage, badCnt, badAddr_lo, badAddr_hi]
 *   (stage 1=写 badCnt 0=全对/255=失败; stage 2=读校验 badCnt=不一致数, badAddr=首错误地址)
 * @note    ⚠ 破坏性测试 — 写满 512 字节 (递增模式), 包含 BOOT/PSW/COM/ASW 全部区域
 */
void _60_EEPROM_testCase(void)
{
    static uint16_t flag  = 0u;
    static uint16_t sendp = 0u;
    static uint8_t  readBuf[EEPROM_TOTAL_SIZE];
    static uint8_t  writeBuf[EEPROM_TOTAL_SIZE];
    uint8_t  msg[4] = {0};
    uint16_t i;

    flag++;
    if (flag >= 1000u) { flag = 0u; }

    if (flag < 100u)
    {
        /* 初始化: 填充递增模式写缓冲, 清读缓冲 */
        if (flag == 0u)
        {
            for (i = 0u; i < EEPROM_TOTAL_SIZE; i++)
            {
                writeBuf[i] = (uint8_t)i;
            }
            memset(readBuf, 0x00, EEPROM_TOTAL_SIZE);
            sendp = 0u;
        }
        canPSWDbgMsgSend(0x60, (uint8_t *)&flag, 2);
    }
    else if (flag == 100u)
    {
        /* 写 512 字节(递增模式), TestMark_Ctrl 脉宽 = 写耗时; 返回 512 = 写后读回 3 次全对 */
        TestMark_Ctrl(1u);
        msg[0] = 1u;   /* stage 1 = 写 */
        msg[1] = (Eeprom_WriteBlock(0u, writeBuf, EEPROM_TOTAL_SIZE) == EEPROM_TOTAL_SIZE) ? 0u : 255u;
        TestMark_Ctrl(0u);
        msg[2] = 0u;
        msg[3] = 0u;
        canPSWDbgMsgSend(0x60u, msg, 4u);
    }
    else if (flag < 200u)
    {
        /* 等待 EEPROM 完成后稳定 */
    }
    else if (flag == 200u)
    {
        /* 读 512 字节 + 逐字节校验 */
        TestMark_Ctrl(1u);
        Eeprom_ReadBlock(0u, readBuf, EEPROM_TOTAL_SIZE);
        TestMark_Ctrl(0u);
        msg[0] = 2u;   /* stage 2 = 读校验 */
        msg[1] = 0u;
        msg[2] = 0u;
        msg[3] = 0u;
        for (i = 0u; i < EEPROM_TOTAL_SIZE; i++)
        {
            if (readBuf[i] != (uint8_t)i)
            {
                if (msg[1] == 0u) { msg[2] = (uint8_t)(i & 0xFFu); msg[3] = (uint8_t)(i >> 8); }
                if (msg[1] < 255u) { msg[1]++; }
            }
        }
        canPSWDbgMsgSend(0x60u, msg, 4u);
    }
    else
    {
        /* 每次发 8 字节, 用 CAN 0x66 上报读回数据 */
        if (sendp < EEPROM_TOTAL_SIZE)
        {
            canPSWDbgMsgSend(0x66u, &readBuf[sendp], 8u);
            sendp += 8u;
        }
    }
}

/* ---- 7x 传感器采集 (ADC) ---- */

/** @brief 70 — VPOWER 电压 + 报警 (小端字节序, 仅一路 VPOWER, vBat/vIgn 同源) */
void _70_ADC_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(PSWvIgn & 0xFFu);        /* vBat 低字节 (同 VPOWER) */
    data[1] = (uint8_t)(PSWvIgn >> 8);           /* vBat 高字节 */
    data[2] = 0u;                                 /* vBat警 无独立电池 */
    data[3] = 0u;
    data[4] = (uint8_t)(PSWvIgn & 0xFFu);        /* vIgn 低字节 (VPOWER) */
    data[5] = (uint8_t)(PSWvIgn >> 8);           /* vIgn 高字节 */
    data[6] = PSWErrVpwrLo;                      /* VPOWER 低压故障 */
    data[7] = PSWErrVpwrHi;                      /* VPOWER 高压故障 */
    canPSWDbgMsgSend(0x70u, data, 8u);
}

/** @brief 71 — 前后桥压力传感器 (小端字节序, 旧 DBC 格式) */
void _71_ADC_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(PSWBrkPreX2_7 & 0xFFu);        /* 前桥压力 低字节 */
    data[1] = (uint8_t)(PSWBrkPreX2_7 >> 8);           /* 前桥压力 高字节 */
    data[2] = PSWfErrPreX2_7;                          /* 前桥故障 */
    data[3] = 0u;
    data[4] = (uint8_t)(PSWBrkPreX3_10 & 0xFFu);       /* 后桥压力 低字节 */
    data[5] = (uint8_t)(PSWBrkPreX3_10 >> 8);          /* 后桥压力 高字节 */
    data[6] = PSWfErrPreX3_10;                         /* 后桥故障 */
    data[7] = 0u;
    canPSWDbgMsgSend(0x71u, data, 8u);
}

/** @brief 72 — 六通道轮速传感器故障 (旧 DBC 顺序: FL/FR/RL/RR/XL/XR) */
void _72_ADC_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = PSWErrWssOpenFL | (PSWErrWssShortFL << 1) | (PSWErrWssGapFL << 2);
    data[1] = PSWErrWssOpenFR | (PSWErrWssShortFR << 1) | (PSWErrWssGapFR << 2);
    data[2] = PSWErrWssOpenRL | (PSWErrWssShortRL << 1) | (PSWErrWssGapRL << 2);
    data[3] = PSWErrWssOpenRR | (PSWErrWssShortRR << 1) | (PSWErrWssGapRR << 2);
    data[4] = PSWErrWssOpenXL | (PSWErrWssShortXL << 1) | (PSWErrWssGapXL << 2);
    data[5] = PSWErrWssOpenXR | (PSWErrWssShortXR << 1) | (PSWErrWssGapXR << 2);
    canPSWDbgMsgSend(0x72u, data, 6u);
}


/** @brief 80 — 轮速值 FL+FR+RL+RR (小端字节序, 旧 DBC 格式) */
void _80_PWM_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(PSWWheelSpeedFL & 0xFFu); data[1] = (uint8_t)(PSWWheelSpeedFL >> 8);
    data[2] = (uint8_t)(PSWWheelSpeedFR & 0xFFu); data[3] = (uint8_t)(PSWWheelSpeedFR >> 8);
    data[4] = (uint8_t)(PSWWheelSpeedRL & 0xFFu); data[5] = (uint8_t)(PSWWheelSpeedRL >> 8);
    data[6] = (uint8_t)(PSWWheelSpeedRR & 0xFFu); data[7] = (uint8_t)(PSWWheelSpeedRR >> 8);
    canPSWDbgMsgSend(0x80u, data, 8u);
}

/** @brief 81 — 轮速值 XL+XR (小端字节序, 旧 DBC 格式) */
void _81_PWM_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(PSWWheelSpeedXL & 0xFFu); data[1] = (uint8_t)(PSWWheelSpeedXL >> 8);
    data[2] = (uint8_t)(PSWWheelSpeedXR & 0xFFu); data[3] = (uint8_t)(PSWWheelSpeedXR >> 8);
    canPSWDbgMsgSend(0x81u, data, 4u);
}

void _90_PWM_CMP_testCase(void)
{
    static uint16_t cnt = 0;
    RTEfValWssTestForbit = 1;   //测试过程中不再检阀
    cnt++;
        ASRFLowSideSw(1u);
        ASRRLowSideSw(1u);
    /* ---- Phase 0: 0% DC (全程关阀) ---- */
    if (cnt < 300u)
    {
        /* ABS 进气阀 (后段高 LOW→HIGH) */
        InValActFL(200u, 0u);   /* 前左进气 */
        InValActFR(200u, 0u);   /* 前右进气 */
        InValActRL(200u, 0u);   /* 后左进气 */
        InValActRR(200u, 0u);   /* 后右进气 */
        InValActXL(200u, 0u);   /* 辅左进气 */
        InValActXR(200u, 0u);   /* 辅右进气 */
        InValActTr(200u, 0u);   /* 挂车进气 */
        /* ABS 排气阀 (前段高 HIGH→LOW) */
        OutValActFL(200u, 0u);  /* 前左排气 */
        OutValActFR(200u, 0u);  /* 前右排气 */
        OutValActRL(200u, 0u);  /* 后左排气 */
        OutValActRR(200u, 0u);  /* 后右排气 */
        OutValActXL(200u, 0u);  /* 辅左排气 */
        OutValActXR(200u, 0u);  /* 辅右排气 */
        OutValActTr(2000u, 0u); /* 挂车排气 (0.01ms) */
        /* ASR 阀 (前段高) */
        ASRValActF(200u, 0u);   /* 前 ASR */
        ASRValActR(200u, 0u);   /* 后 ASR */
        ASRValActX(200u, 0u);   /* 挂车 ASR */
    }
    /* ---- Phase 1: 33% DC ---- */
    else if (cnt < 600u)
    {
        InValActFL(300u, 100u);
        InValActFR(300u, 100u);
        InValActRL(300u, 100u);
        InValActRR(300u, 100u);
        InValActXL(300u, 100u);
        InValActXR(300u, 100u);
        InValActTr(300u, 100u);
        OutValActFL(300u, 100u);
        OutValActFR(300u, 100u);
        OutValActRL(300u, 100u);
        OutValActRR(300u, 100u);
        OutValActXL(300u, 100u);
        OutValActXR(300u, 100u);
        OutValActTr(3000u, 1000u); /* 0.01ms */
        ASRValActF(300u, 100u);
        ASRValActR(300u, 100u);
        ASRValActX(300u, 100u);
    }
    /* ---- Phase 2: 75% DC ---- */
    else if (cnt < 900u)
    {
        InValActFL(400u, 300u);
        InValActFR(400u, 300u);
        InValActRL(400u, 300u);
        InValActRR(400u, 300u);
        InValActXL(400u, 300u);
        InValActXR(400u, 300u);
        InValActTr(400u, 300u);
        OutValActFL(400u, 300u);
        OutValActFR(400u, 300u);
        OutValActRL(400u, 300u);
        OutValActRR(400u, 300u);
        OutValActXL(400u, 300u);
        OutValActXR(400u, 300u);
        OutValActTr(4000u, 3000u); /* 0.01ms */
        ASRValActF(400u, 300u);
        ASRValActR(400u, 300u);
        ASRValActX(400u, 300u);
    }
    /* ---- Phase 3: 100% DC ---- */
    else
    {
        InValActFL(500u, 500u);
        InValActFR(500u, 500u);
        InValActRL(500u, 500u);
        InValActRR(500u, 500u);
        InValActXL(500u, 500u);
        InValActXR(500u, 500u);
        InValActTr(500u, 500u);
        OutValActFL(500u, 500u);
        OutValActFR(500u, 500u);
        OutValActRL(500u, 500u);
        OutValActRR(500u, 500u);
        OutValActXL(500u, 500u);
        OutValActXR(500u, 500u);
        OutValActTr(5000u, 5000u); /* 0.01ms */
        ASRValActF(500u, 500u);
        ASRValActR(500u, 500u);
        ASRValActX(500u, 500u);
    }
}




void _92_PWM_CMP_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = PSWErrInValOpenFL  | (PSWErrInValShortFL  << 1);
    data[1] = PSWErrOutValOpenFL | (PSWErrOutValShortFL << 1);
    data[2] = PSWErrInValOpenFR  | (PSWErrInValShortFR  << 1);
    data[3] = PSWErrOutValOpenFR | (PSWErrOutValShortFR << 1);
    data[4] = PSWErrInValOpenRL  | (PSWErrInValShortRL  << 1);
    data[5] = PSWErrOutValOpenRL | (PSWErrOutValShortRL << 1);
    data[6] = PSWErrInValOpenRR  | (PSWErrInValShortRR  << 1);
    data[7] = PSWErrOutValOpenRR | (PSWErrOutValShortRR << 1);
    canPSWDbgMsgSend(0x92u, data, 8u);
}


void _93_PWM_CMP_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = PSWErrInValOpenTr  | (PSWErrInValShortTr  << 1);
    data[1] = PSWErrOutValOpenTr | (PSWErrOutValShortTr << 1);
    data[2] = PSWErrInValOpenXL  | (PSWErrInValShortXL  << 1);
    data[3] = PSWErrOutValOpenXL | (PSWErrOutValShortXL << 1);
    data[4] = PSWErrInValOpenXR  | (PSWErrInValShortXR  << 1);
    data[5] = PSWErrOutValOpenXR | (PSWErrOutValShortXR << 1);
    canPSWDbgMsgSend(0x93u, data, 6u);
}

void _94_PWM_CMP_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = PSWErrASRValveOpenF  | (PSWErrASRValveShortF << 1);
    data[1] = PSWErrASRValveOpenR  | (PSWErrASRValveShortR << 1);
    data[2] = PSWErrASRValveOpenX  | (PSWErrASRValveShortX << 1);
    canPSWDbgMsgSend(0x94u, data, 3u);
}

void _A0_PWM_CMP_testCase(void)
{
    uint8_t faultMsg[8] = {0};
    faultMsg[0] = PSWErrOpenDrive724_U6  | (PSWErrShortDrive724_U6  << 4);
    faultMsg[1] = PSWErrOpenDrive724_U9  | (PSWErrShortDrive724_U9  << 4);
    faultMsg[2] = PSWErrOpenDrive724_U12 | (PSWErrShortDrive724_U12 << 4);
    faultMsg[3] = PSWErrOpenDrive724_U13 | (PSWErrShortDrive724_U13 << 4);
    faultMsg[4] = PSWErrOpenDrive724_U19 | (PSWErrShortDrive724_U19 << 4);
    faultMsg[5] = PSWErrDriveChip;
    canPSWDbgMsgSend(0xA1u, faultMsg, 6u);
}
/* ======================================================================== */
/*  测试用例注册表                                                             */
/* ======================================================================== */

static const debug_list_t g_debugCase_list[] =
{
    /* ---- 1x 系统 ---- */
    {0x10, _10_softReset_testCase,      1, {0x10}},
    {0x11, _11_softReset_testCase,      1, {0x11}},

    /* ---- 2x 看门狗 ---- */
    {0x20, _20_WD_testCase,             1, {0x20}},
    {0x21, _21_WD_testCase,             1, {0x21}},

    /* ---- 3x 定时器 ---- */
    {0x30, _30_10MStimer_testCase,      1, {0x30}},

    /* ---- 4x 电源/GPIO ---- */
    {0x40, _40_GPO_testCase,            1, {0x40}},
    {0x41, _41_GPI_testCase,            1, {0x41}},

    /* ---- 6x EEPROM ---- */
    {0x60, _60_EEPROM_testCase,         2, {0x60}},

    /* ---- 7x 传感器采集 ---- */
    {0x70, _70_ADC_testCase,            1, {0x70}},
    {0x71, _71_ADC_testCase,            1, {0x71}},
    {0x72, _72_ADC_testCase,            1, {0x72}},

    /* ---- 8x 轮速计算值 ---- */
    {0x80, _80_PWM_testCase,            1, {0x80}},
    {0x81, _81_PWM_testCase,            1, {0x81}},

    /* ---- 9x/Ax 阀故障 & 芯片诊断 ---- */
    {0x90, _90_PWM_CMP_testCase,        0, {}},
    {0x91, _90_PWM_CMP_testCase,        0, {}},
    {0x92, _92_PWM_CMP_testCase,        1, {0x92}},
    {0x93, _93_PWM_CMP_testCase,        1, {0x93}},
    {0x94, _94_PWM_CMP_testCase,        1, {0x94}},
    {0xA0, _A0_PWM_CMP_testCase,        1, {0xA0}},
};

static const uint8_t g_debug_case_num =
    (uint8_t)(sizeof(g_debugCase_list) / sizeof(g_debugCase_list[0]));

void PSWdebug(void)
{
    uint8_t i;

    /* ---- 注册新测试 ---- */
    if (RTEfDbgMsgSW3 != 0xFF)
    {
        if (g_debugOpenCnt < MAX_DEBUG_CNT)
        {
            /* 查重 */
            for (i = 0u; i < g_debugOpenCnt; i++)
            {
                if (g_debugOpenIndx[i] == RTEfDbgMsgSW3)
                {
                    break;
                }
            }
            if (i >= g_debugOpenCnt)
            {
                /* 匹配注册表 */
                for (uint8_t j = 0u; j < g_debug_case_num; j++)
                {
                    if (g_debugCase_list[j].debugIndx == RTEfDbgMsgSW3)
                    {
                        g_debugFuncList[g_debugOpenCnt] = g_debugCase_list[j].debugFunc;
                        g_debugOpenIndx[g_debugOpenCnt] = RTEfDbgMsgSW3;
                        g_debugOpenCnt++;
                        break;
                    }
                }
            }
        }
        RTEfDbgMsgSW3 = 0xFF;  /* 消费激活信号 */
    }

    /* ---- 执行已注册的测试 ---- */
    for (uint8_t j = 0u; j < g_debugOpenCnt; j++)
    {
        if (g_debugFuncList[j] != NULL)
        {
            g_debugFuncList[j]();
        }
    }
}
