///////////////////////////////////////////////////////////////////////////////
// WASR_ControlConfig.h
// @brief  模块级控制参数 (编译期固定)。
///////////////////////////////////////////////////////////////////////////////
#ifndef WASR_CONTROLCONFIG_H_
#define WASR_CONTROLCONFIG_H_

// 转矩限幅 [0.1% 额定转矩] (驱动侧 CST 输出与负载侧目标转矩共用)
// 1000 = 100% 额定; 3000 = 300% (对拖/扫频常用)
#define WASR_TC_TORLIM                3000

// 负载侧 (对拖 CST) 目标转矩限幅 [0.1% 额定转矩]
// 6071h 数据范围 -3000~3000, 100.0% = 1 倍电机额定转矩。
// 负载模拟默认收到 100%: 超过额定会持续过载发热, 对拖台架没必要。
#define WASR_TC_LOADTORLIM            1000

// ---------------- 驱动器监视 (CoE SDO 周期读) ----------------
// 1 = 周期读两台伺服的惯量比/刚性等级, 驱动侧那台的值写到 Status.Driver_*
// 只读 (SDO upload) 不磨损驱动器 EEPROM, 也不影响实验流程
#define IR_MON_ENABLE                 1
// 读取周期 [ms]: 每台两个对象, 100 ms = 40 次/s, 远低于 SDO 邮箱能力
#define IR_MON_PERIOD_MS              100

// Servo0 = 维控 VD5L: 0x2003:01 负载惯量比 (单位 0.01, 100..10000)
#define IR_MON_S0_INDEX               0x2003
#define IR_MON_S0_SUBINDEX            0x01
#define IR_MON_S0_VAL_MIN             100
#define IR_MON_S0_VAL_MAX             10000
// Servo0 = 维控 VD5L: 0x2003:02 负载刚性等级 (0..31)
#define IR_MON_S0_RIG_INDEX           0x2003
#define IR_MON_S0_RIG_SUBINDEX        0x02
#define IR_MON_S0_RIG_VAL_MIN         0
#define IR_MON_S0_RIG_VAL_MAX         31

// Servo1 = 汇川 SV680N / SV630N: 0x2008:10 负载惯量比 H08.15 (0.01, 0..12000)
#define IR_MON_S1_INDEX               0x2008
#define IR_MON_S1_SUBINDEX            0x10
#define IR_MON_S1_VAL_MIN             0
#define IR_MON_S1_VAL_MAX             12000
// Servo1 = 汇川 SV680N / SV630N: 0x2009:02 刚性等级 H09.01 (0..41)
#define IR_MON_S1_RIG_INDEX           0x2009
#define IR_MON_S1_RIG_SUBINDEX        0x02
#define IR_MON_S1_RIG_VAL_MIN         0
#define IR_MON_S1_RIG_VAL_MAX         41

#endif // WASR_CONTROLCONFIG_H_
