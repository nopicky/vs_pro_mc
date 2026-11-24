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

// 转矩前馈 (60B2h Torque offset) 限幅 [0.1% 额定转矩]
// 为什么单独设、且比 WASR_TC_TORLIM 紧:
//   60B2 是"按惯量算出来的力"，标定时如果 J 估大或增益拉满，很容易算出 >100%，
//   而它在 CSP 下是叠在速度环输出上的 —— 没有位置环那种"错了会自己收敛"的兜底。
//   1000 = 100% 额定：350m/min 加速段理论需要约 118%，这里夹到 100% 是刻意的
//   （宁可前馈给不足、让位置环补一点，也不要靠前馈把电机长期顶在过载区）。
#define WASR_TC_TORFF_LIM             1000

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

// ---------------- 绝对位置自动清零 (维控 P10-06 / 0x200A:06 AbsEncRst) ----------------
// 为什么要有: 607Ah / 6064h 都是 int32, VD5E 的 23bit 编码器一圈 8388608 counts,
//   于是位置空间 = 2^31 / 8388608 = 正好 256 转 = 256 x 195.349 = 50009 mm 送纸。
//   30 m/min 连续送纸约 100 s 就用完, 越过之后 PosToLong 会把目标位置钳住,
//   表现为"电机一步不动 / 转矩 1% / 驱动器不报错"(2026-11-23 实机踩过)。
//   根治只能把驱动器的多圈计数清掉, 让 6064 回到 0 附近。
//
// 对象: P10-06 AbsEncRst, UINT16, 设定值 1 = 清多圈圈数 + 清当前位置 + 清编码器告警
//       (面板 / 上位机 / SDO 三种方式效果相同, 写完驱动器自动回 0, 且该值不存储,
//        所以不磨损 EEPROM, 可以按需反复清)
//
// ★ 只能在"未使能"时执行: 写 1 是"立即生效", 6064 瞬间从 2e9 跳到 0。
//   若驱动器还在 Operation Enabled, 607A 仍是旧的大位置, 它看到目标领先
//   2e9 counts(≈47km) 就会以最大加速度去追 —— 等于飞车。
//   未使能时送纸凸轮的冻结分支每拍把 posCmdOut 贴回 6064, 位置跳变被自动吸收,
//   使能后不存在残余偏差, 所以这个窗口是零冲击的。
#define ABS_ZERO_ENABLE               1

// 自动触发阈值 [指令单位]: 任一驱动轴 |6064| 超过它 → 挂"待清零"标志, 等停机窗口
// 1.6e9 ≈ 190 转 ≈ 37.1 m 送纸; 到跳闸线 2.1e9 还留 5e8 (≈11.6 m, 30m/min 下 23 s)
#define ABS_ZERO_POS_UNIT             1.6e9

// 安全条件需要连续满足的拍数 (Enable=0 且所有已挂载驱动轴都不在 OP)
// 125us 周期下 2000 拍 = 250 ms: 足够等 CiA402 从 0F 走到 06
// (Enable 刚清的那几十毫秒里 m_abOpDrive 仍为真, 不去抖就会清在使能态)
#define ABS_ZERO_SAFE_CYCLES          2000UL

// 写成功后再等多少拍才读 6064 验证 (125us x 2000 = 250 ms)
#define ABS_ZERO_SETTLE_CYCLES        2000UL

// 验证容差 [指令单位]: 清完 |6064| 应远小于它 (10000 counts ≈ 0.23 mm)
#define ABS_ZERO_VERIFY_TOL           10000L

// 失败后自动重试的冷却拍数 (125us x 240000 = 30 s), 防止位置仍超阈值时无限重发
#define ABS_ZERO_RETRY_COOL           240000UL

// P10-06 对象与命令值
#define ABS_ZERO_INDEX                0x200A
#define ABS_ZERO_SUBINDEX             0x06
#define ABS_ZERO_CMD_CLEAR            1

// 6064h 位置实际值 (验证清零是否真的生效)
#define ABS_ZERO_POS_INDEX            0x6064

#endif // WASR_CONTROLCONFIG_H_
