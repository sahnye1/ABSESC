/**
 * @file    test_deal.h
 * @brief   测试/调试层入口 — 由 PSW_10ms_deal() 在固定位置调用
 *
 * @details 三个入口的调用**位置不能挪动** (有顺序依赖):
 *            TestDeal_Early() : ④      阀波形测试, 须早于 ValveDiag_Process
 *            TestDeal_Mid()   : ⑥      阀诊断之后 (阀诊断明细输出)
 *            TestDeal_Late()  : ⑧.5~⑧.6 须晚于 PSWDataToRte (CAN 上报用 RTE/PSW 最新值)
 *                              + ⑪     节拍验证
 *
 *          开关见 test_cfg.h; 实现见 test_deal.c。
 */

#ifndef TEST_DEAL_H
#define TEST_DEAL_H

void TestDeal_Early(void);
void TestDeal_Mid(void);
void TestDeal_Late(void);

#endif /* TEST_DEAL_H */
