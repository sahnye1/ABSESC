注意：每次更新lib包时，同步上传各层版本日期（20260811401）、修改人（**）、修改内容（1、*** 换行2、***）

一、驱动层：
版本日期：2026/8/17
修改人：王孟寒
修改内容：
1、继电器上电闭合修复：Gpio_Init() 中 P14.3 初始化为输出 HIGH，board_init() 调用 Gpio_RelayCtrl(1) 闭合继电器，解决 VPOWER 无电导致所有诊断失效的问题
2、硬件喂狗引脚补全：cmn.h 中 GPIO_WD_IN 从 P18.0 改为 P5.0（电路图核对），gpio.c 配置表加 P5.0 输出初始化，删除废弃的 WD_EN 定义
3、VPOWER 电压检测修复：sar1_adc 从 PASS0_SAR1 切回 PASS0_SAR0（老代码用 SAR0，SAR1 读数为 0），通道改到 CH6~CH8 避免与 sensor_diag 冲突，去掉重复的 Cy_Adc_Init
4、阀诊断逻辑重构：bts724g.c 废弃原 3 相状态机+空闲扫描+partner 守卫的复杂实现，改为老代码风格 4 步轮转（开路→短路→芯片开路→芯片短路），每步 40ms 测 1 阀/芯片，3 次上下计数消抖
5、开路检测极性修正：is_fault = !Bts724g_ReadST()（BTS724G ST 为 open-drain 低有效，ST=Low 判开路）
6、删除错误的 FAR 守卫：原「FAR=High 跳过开路检测」方向反了，导致接阀后无法恢复开路故障
7、ValvePwm_SetOnOff 加 SetCounter(period-1) 强制溢出生效：解决检阀 300μs 窗口内输出来不及变高导致阀无滴答声的问题
8、芯片检测防护：芯片级故障确认次数改为 5 次（CHIP_FAULT_CONFIRM_NEED，高于阀级 3 次），芯片短路检测加守卫（本芯片有阀开路则跳过，防阀开路导致 OUT 浮空误报芯片短路）
9、低边开关映射：bts724g_asr_lowside_enable 中 TRFIN/TRFOUT/RXI/RXO/LXO/LXI 加 P2.0（前 ASR 低边），FA_ASR/TR_ASR 走 P2.0，DA_ASR 走 P6.2
10、阀映射表 ST 脚修正：U6 ST12=P5.2/ST34=P5.1，U9 ST12=P6.0/ST34=P5.3，U12 ST12=P2.3/ST34=P2.2，U13 ST12=P0.1，U19 ST12=P14.2/ST34=P3.1（电路图核对）
11、控阀 PWM 接口拆分：新增 ValvePwm_SetDutyNoReset（智能衔接动态调参，不清 counter 无尾巴），rte_psw 控阀改用该接口，检阀保持 SetOnOff 立即归零
12、轮速传感器故障通道映射修正：CH0=P6.4→FL，CH1=P6.5→RL，CH2=P7.0→RR，CH3=P7.1→FR，CH4=P7.3→XL，CH5=P7.4→XR
13、SCC3000/ECU 类型桩函数：新增 SCC3000_calibration_req/SCC3000_UnCalibration_req/SCC3000_calibration_resp/SCC3000_Err/get_ECU_type 桩函数，解决 COM.lib 链接错误
14、CAN 接口对齐老代码：新增 can_info_cfg/can_init 兼容接口；CAN0 发送改为查发送白名单+固定 TX buffer 映射+固定 DLC=8+0xFF 填充；CAN1 发送改为无白名单轮转+固定 DLC=8
15、can0_bitrate_set 死循环修复：改为只更新位时序配置不立即 Init（对齐老代码行为，避免时钟未配置时 Cy_CANFD_Init 卡死）
16、中断槽冲突修复：Traveo II CM4 仅 8 个 CPU 中断线，轮速 6 通道合并到同一 CPUIntIdx6（对齐老代码），10ms 定时器改 CPUIntIdx3，消除与 CAN0/CAN1 的冲突
17、PSW_api.h 精简：CAN 部分只保留 can_info_cfg/can_init 高层接口，底层驱动接口归 can_driver.h，消除重复的 can_filter_t 定义
18、ERR_CODE 读写越界 bug 修复：maxSize 参数从 EEPROM_ERR_CODE_SIZE(116) 改为 EEPROM_ASW_SIZE(416)，修复 ERR_CODE 接口永远越界返回 0 的问题
19、已知遗留：CNT37 资源冲突（后桥左进阀 PWM LINE37 P21.5 与 RX 轮速捕获 TR_ONE_CNT_IN111 P12.1），待硬件改版解决

二、应用层：

三、通讯层：
