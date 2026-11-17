///////////////////////////////////////////////////////////////////////////////
// ServoPlatformConfig.h
// @brief  伺服/电机参数与驱动侧角色 (编译期宏), 构造时填入 ServoParams。
//   Servo0 = 维控 VD5L + 电机 (由 WASR_S0_MOTOR 编译期开关选择)
//            默认        = WD80M-07530S-A1  (750W, 17bit)
//            可切换为    = WD40M-01030S-D1* (100W, 23bit, 型号代码 D066)
//                          WE130M-10025S-A1*(1.0kW, 17bit, 型号代码 A091)
//   Servo1 = 汇川 驱动器 + 电机 (驱动器与电机成对, 由 WASR_S1_MOTOR 编译期开关选择)
//            默认        = SV630N + MS1H2-10C30CB-T33*R   (18bit, 非抱闸 T331R)
//            可切换为    = SV680N + MS1H1-75B30CB-A6/S63*R (26bit)
// 角色运行中可在 XAE 里改 Status.Driver_Servo_number (Enable = 0 时生效)。
//
// 齿轮比约定: GEAR_NUM / GEAR_DEN 直接填驱动器的 6091:01h / 6091:02h
// (VD5L: P00-17/P00-18, SV680N/SV630N: 6091.01h/02h; 6091:01h 为分子)。
//   每转指令单位数 = ENC_CNT_PER_REV × GEAR_DEN / GEAR_NUM
//   (即一个电机轴转对应的指令单位数, 6064h/607Ah/60FFh/606Ch 都是指令单位)
//   [rpm]          = (指令单位/s) × 60 / 每转指令单位数
// 改驱动器齿轮比后必须同步改这里, 否则 rpm 换算差倍数, 高速时 60FFh/606Ch
// 还可能溢出 32 位; 模块每拍还会把这里的分子/分母写进 6091, 两边必须一致。
// Servo0 的 WD40M 分支: 6091:01h = 编码器每转计数, 6091:02h = 10000
// → 1 电机转 = 10000 指令单位 (0.036 度/单位), 位置指令数值直接可读
// (例如 2500 = 1/4 转); WD80M / WE130M 分支按 1:1 (1 转 = 编码器每转计数)。
// Servo1 默认 (SV630N) 按 1:1, 1 转 = 262144 (6000 rpm 时 60FFh 2.62e7, 不溢出);
// 切到 SV680N 时用手册推荐的 8:1, 1 转 = 8388608。
///////////////////////////////////////////////////////////////////////////////
#ifndef SERVOPLATFORMCONFIG_H_
#define SERVOPLATFORMCONFIG_H_

// Servo0 电机型号选择: 0 = WD80M-07530S-A1, 1 = WD40M-01030S-D1* (D066),
//                       2 = WE130M-10025S-A1* (A091)
// 只换电机侧参数 (编码器位数/转子惯量/额定转矩/惯量比初值), 驱动器仍是 VD5L
// 0 = WD80M-07530S-A1 (750W, 17bit)
#define WASR_S0_MOTOR_WD80M_07530S_A1   0
// 1 = WD40M-01030S-D1* (100W, 23bit, 型号代码 D066)
#define WASR_S0_MOTOR_WD40M_01030S_D1   1
// 2 = WE130M-10025S-A1* (1.0kW, 17bit, 型号代码 A091)
#define WASR_S0_MOTOR_WE130M_10025S_A1  2
// Servo0 电机型号 (默认 WE130M-10025S-A1 (17bit); 也可在工程属性里 /D WASR_S0_MOTOR=1 覆盖)
#ifndef WASR_S0_MOTOR
#define WASR_S0_MOTOR                   WASR_S0_MOTOR_WE130M_10025S_A1
#endif
#if (WASR_S0_MOTOR != WASR_S0_MOTOR_WD80M_07530S_A1) && (WASR_S0_MOTOR != WASR_S0_MOTOR_WD40M_01030S_D1) && (WASR_S0_MOTOR != WASR_S0_MOTOR_WE130M_10025S_A1)
#error "WASR_S0_MOTOR must be WASR_S0_MOTOR_WD80M_07530S_A1 (0), WASR_S0_MOTOR_WD40M_01030S_D1 (1) or WASR_S0_MOTOR_WE130M_10025S_A1 (2)"
#endif

#if WASR_S0_MOTOR == WASR_S0_MOTOR_WD40M_01030S_D1
// Servo0: 维控 VD5L + WD40M-01030S-D1* (WD40M 系列, 型号代码 D066)
// 完整铭牌: 功率 100 W / 额定电压 220 V / 额定电流 1 A / 额定转速 3000 rpm
//   额定转矩 0.318 N·m / 最大转矩 0.954 N·m / 反电势系数 22 V/krpm
//   力矩系数 0.32 N·m/A / 转子惯量 0.066 kg·cm2 / 线电阻 18.8 Ω
//   线电感 12.2 mH / 最大电流(计算) 3 A / 最大转速 6000 rpm / 极数 10
//   编码器位数 23
// 转子惯量 0.066 kg·cm2 = 6.6e-6 kg·m2; 23bit → 8388608 inc/rev
// 注意: WASR_TC_TORLIM = 3000 (300%) 正好等于该电机最大转矩/最大电流
// 每转编码器计数
#define WASR_S0_ENC_CNT_PER_REV      8388608UL
// 6091:01h (P00-17) 分子 = 编码器每转计数
#define WASR_S0_GEAR_NUM             8388608UL
// 6091:02h (P00-18) 分母 = 10000 → 每转指令单位数 = 8388608 × 10000 / 8388608 = 10000
#define WASR_S0_GEAR_DEN             10000UL
// 电机转子惯量 [kg·m2]
#define WASR_S0_INERTIA_BASE         0.0000066f
// 电机额定转矩 [N·m]
#define WASR_S0_RATED_TORQUE         0.318f
#elif WASR_S0_MOTOR == WASR_S0_MOTOR_WE130M_10025S_A1
// Servo0: 维控 VD5L + WE130M-10025S-A1* (WE130M 系列, 型号代码 A091)
// 完整铭牌: 功率 1000 W / 额定电压 220 V / 额定电流 4 A / 额定转速 2500 rpm
//   额定转矩 4 N·m / 最大转矩 12 N·m / 反电势系数 72 V/krpm
//   力矩系数 1 N·m/A / 转子惯量 8.5 kg·cm2 / 线电阻 2.76 Ω
//   线电感 6.42 mH / 最大电流(计算) 12 A / 最大转速 5000 rpm / 极数 8
//   编码器位数 17
// 转子惯量 8.5 kg·cm2 = 8.5e-4 kg·m2; 17bit → 131072 inc/rev
// 注意: WASR_TC_TORLIM = 3000 (300%) 正好等于该电机最大转矩 12 N·m
// (最大电流 12 A 也正好 3 倍额定), 对拖/扫频要留裕量请把 TORLIM 降到 2500 以下
// 齿轮比按 1:1 (与 WD80M 同), 5000 rpm 时 60FFh ≈ 1.09e7, 不溢出 32 位
// 若沿用 WD40M 那套 "1 转 = 10000 指令单位" 约定, 把 GEAR_NUM 改成 131072、
// GEAR_DEN 改成 10000 (与驱动器 P00-17/P00-18 保持一致)
// 每转编码器计数
#define WASR_S0_ENC_CNT_PER_REV      131072UL
// 6091:01h (P00-17) 分子
#define WASR_S0_GEAR_NUM             1UL
// 6091:02h (P00-18) 分母
#define WASR_S0_GEAR_DEN             1UL
// 电机转子惯量 [kg·m2]
#define WASR_S0_INERTIA_BASE         0.00085f
// 电机额定转矩 [N·m]
#define WASR_S0_RATED_TORQUE         4.0f
#else
// Servo0: 维控 VD5L + WD80M-07530S-A1* (WD80M 系列), 17 位编码器, 转子惯量 1.48e-4 kg·m2
// 每转编码器计数
#define WASR_S0_ENC_CNT_PER_REV      131072UL
// 6091:01h (P00-17) 分子
#define WASR_S0_GEAR_NUM             1UL
// 6091:02h (P00-18) 分母
#define WASR_S0_GEAR_DEN             1UL
// 电机转子惯量 [kg·m2]
#define WASR_S0_INERTIA_BASE         0.000148f
// 电机额定转矩 [N·m]
#define WASR_S0_RATED_TORQUE         2.39f
#endif

// Servo1 配套选择 (编译期开关): 驱动器与电机成对, 一个组合一个选项
//   0 = SV680N + MS1H1-75B30CB-A6/S63*R (既有台架) 26bit / 0.68 kg·cm2 / 2.39 N·m
//   1 = SV630N + MS1H2-10C30CB-T33*R    (非抱闸)   18bit / 1.78 kg·cm2 / 3.18 N·m
// 换配套只改 WASR_S1_MOTOR 一处 (或工程属性 /D WASR_S1_MOTOR=0/1 覆盖默认值);
// Servo1 序号、驱动/负载角色、实验脚本都不变。改选项后必须把 Servo1 驱动器的
// 6091.01h/6091.02h 改成与该分支相同的值 (见上方齿轮比约定)。
// 0 = 汇川 SV680N + MS1H1-75B30CB-A6/S63*R (26bit)
#define WASR_S1_MOTOR_SV680N_MS1H1_75B30CB_A6       0
// 1 = 汇川 SV630N + MS1H2-10C30CB-T33*R (18bit, 非抱闸 T331R)
#define WASR_S1_MOTOR_SV630N_MS1H2_10C30CB_T33      1
// Servo1 配套 (默认 SV630N + MS1H2-10C30CB-T33: 1.0kW/18bit 台架;
// /D WASR_S1_MOTOR=0 可切回 SV680N + MS1H1-75B30CB-A6)
#ifndef WASR_S1_MOTOR
#define WASR_S1_MOTOR                   WASR_S1_MOTOR_SV630N_MS1H2_10C30CB_T33
#endif
#if (WASR_S1_MOTOR != WASR_S1_MOTOR_SV680N_MS1H1_75B30CB_A6) && (WASR_S1_MOTOR != WASR_S1_MOTOR_SV630N_MS1H2_10C30CB_T33)
#error "WASR_S1_MOTOR must be WASR_S1_MOTOR_SV680N_MS1H1_75B30CB_A6 (0) or WASR_S1_MOTOR_SV630N_MS1H2_10C30CB_T33 (1)"
#endif

#if WASR_S1_MOTOR == WASR_S1_MOTOR_SV630N_MS1H2_10C30CB_T33
// Servo1: 汇川 SV630N + 电机 MS1H2-10C30CB-T33*R (非抱闸 T331R)
// MS1H2 = 低惯量、中容量系列, 机座 100 mm, 1.0 kW 伺服电机
// 完整铭牌: 额定功率 1.0 kW / 额定电压 220 V / 额定频率 250 Hz
//   额定转矩 3.18 N·m / 最大转矩 9.54 N·m / 转矩系数 0.54 N·m/A
//   额定电流 6.4 A / 最大电流 23 A / 额定转速 3000 rpm / 最高转速 6000 rpm
//   转子惯量 1.78±10% kg·cm2 (非抱闸; 抱闸 T334R 为 2.6±10% → 2.6e-4 kg·m2)
// 编码器 18 位 (SV630N 配 T 系列总线电机, 2^18 = 262144 inc/rev, H00.00=14101)
// 注意: WASR_TC_TORLIM = 3000 (300%) 正好等于该电机最大转矩 9.54 N·m
// (最大电流 23 A 约 3.6 倍额定), 对拖/扫频要留裕量请把 TORLIM 降到 2500 以下
// 每转编码器计数
#define WASR_S1_ENC_CNT_PER_REV      262144UL
// 6091.01h 分子 (1:1, 即 SV630N 出厂值)
#define WASR_S1_GEAR_NUM             1UL
// 6091.02h 分母
#define WASR_S1_GEAR_DEN             1UL
// 电机转子惯量 [kg·m2]
#define WASR_S1_INERTIA_BASE         0.000178f
// 电机额定转矩 [N·m]
#define WASR_S1_RATED_TORQUE         3.18f
#else
// Servo1: 汇川 SV680N + MS1H1-75B30CB-A6/S63*R (26bit 选项), 26 位编码器,
// 转子惯量 0.68e-4 kg·m2 (非抱闸; 抱闸 0.71 kg·cm2 → 7.1e-5)
// 每转编码器计数
#define WASR_S1_ENC_CNT_PER_REV      67108864UL
// 6091.01h 分子 (手册推荐 8:1)
#define WASR_S1_GEAR_NUM             8UL
// 6091.02h 分母
#define WASR_S1_GEAR_DEN             1UL
// 电机转子惯量 [kg·m2]
#define WASR_S1_INERTIA_BASE         0.000068f
// 电机额定转矩 [N·m]
#define WASR_S1_RATED_TORQUE         2.39f
#endif

// 驱动侧伺服初值: 0 = Servo0 驱动, 1 = Servo1 驱动
// 默认 0 = Servo0 (VD5L 17bit) 作为驱动侧 (单轴送纸凸轮首测)
#ifndef WASR_EXPERIMENT_DRIVE_SIDE_SERVO
#define WASR_EXPERIMENT_DRIVE_SIDE_SERVO       0
#endif
#if (WASR_EXPERIMENT_DRIVE_SIDE_SERVO != 0) && (WASR_EXPERIMENT_DRIVE_SIDE_SERVO != 1)
#error "WASR_EXPERIMENT_DRIVE_SIDE_SERVO must be 0 or 1 (DriveSide servo index)"
#endif

#endif // SERVOPLATFORMCONFIG_H_
