///////////////////////////////////////////////////////////////////////////////
// MotorSpec.h
// @brief  电机规格表 (替代原来 ServoPlatformConfig.h 的编译期宏树)。
//
// 设计意图:
//   一台机的差异只在「电机参数」和「哪个槽位接什么」，全是数据、没有逻辑，
//   所以用查表而不是 #if 分支: 加一个新电机只加一行表项, 不动任何逻辑代码。
//
// 每转指令单位数 = EncCntPerRev × GearDen / GearNum
//   (GearNum / GearDen 直接就是驱动器 6091:01h / 6091:02h, 模块每拍下发,
//    两边必须一致, 否则 rpm 换算差倍数, 高速时 60FFh/606Ch 还会溢出 32 位)
//   [rpm] = (指令单位/s) × 60 / 每转指令单位数
//
// 新增电机步骤:
//   1) 在表里加一项 (MotorSpec.cpp), 并在下面加一个编号宏;
//   2) 在站点表 (SiteConfig.cpp) 里把对应槽位的 ucMotor 指向它。
///////////////////////////////////////////////////////////////////////////////
#ifndef MOTORSPEC_H_
#define MOTORSPEC_H_

/* 电机编号 (站点表里填这个) */
#define MOTOR_WD80M_17B      0   /* 维控 WD80M-07530S-A1  750W  17bit */
#define MOTOR_WD40M_23B      1   /* 维控 WD40M-01030S-D1*  100W  23bit (1转=10000单位) */
#define MOTOR_WE130M_17B     2   /* 维控 WE130M-10025S-A1* 1.0kW 17bit */
#define MOTOR_SV630N_18B     3   /* 汇川 MS1H2-10C30CB-T33* 1.0kW 18bit */
#define MOTOR_SV680N_26B     4   /* 汇川 MS1H1-75B30CB-A6/S63*      26bit (6091 = 8:1) */
#define MOTOR_WE130_23B      5   /* 维控 VD5E 驱动轴 23bit 绝对值 (1转=8388608单位, 6091=1:1) */
#define MOTOR_SPEC_COUNT     6

/* 电机规格 */
typedef struct _MOTOR_SPEC_T
{
    const char*  pcName;          /* 铭牌简称 (日志/调试用) */
    unsigned int ulEncCntPerRev;  /* 编码器每转计数 */
    unsigned int ulGearNum;       /* 6091:01h 分子 */
    unsigned int ulGearDen;       /* 6091:02h 分母 */
    float        fInertiaBase;    /* 转子惯量 [kg·m2] */
    float        fRatedTorque;    /* 额定转矩 [N·m] */
} MOTOR_SPEC_T;

/* 电机规格表 (下标 = MOTOR_xxx 编号) */
extern const MOTOR_SPEC_T g_aMotorSpec[MOTOR_SPEC_COUNT];

/* 电机规格 (越界回退到 MOTOR_WE130M_17B)
   @param ucMotor MOTOR_xxx 编号
   @return 规格指针 (不为 NULL) */
const MOTOR_SPEC_T* MotorSpecGet(unsigned char ucMotor);

/* 每转指令单位数 (已处理 0 值与非法组合)
   @param ucMotor MOTOR_xxx 编号
   @return 每转指令单位数 (>= 1) */
double MotorSpecUnitsPerRev(unsigned char ucMotor);

#endif // MOTORSPEC_H_
