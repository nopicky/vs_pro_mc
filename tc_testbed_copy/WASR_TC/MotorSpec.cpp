///////////////////////////////////////////////////////////////////////////////
// MotorSpec.cpp
// @brief  电机规格表 (数值来源: 各电机铭牌, 见 MotorSpec.h 说明)
#include "TcPch.h"
#pragma hdrstop

#include "MotorSpec.h"

// 电机规格表 (下标 = MOTOR_xxx 编号)
// 每转指令单位数 = ulEncCntPerRev × ulGearDen / ulGearNum
const MOTOR_SPEC_T g_aMotorSpec[MOTOR_SPEC_COUNT] =
{
    // 0 = 维控 WD80M-07530S-A1: 750W / 3000rpm / 2.39 N·m / 1.48e-4 kg·m2 / 17bit
    { "WD80M-07530S-A1",   131072UL,    1UL,        1UL,
      0.000148f,  2.39f },
    // 1 = 维控 WD40M-01030S-D1*: 100W / 3000rpm / 0.318 N·m / 6.6e-6 kg·m2 / 23bit
    //     沿用 "1 电机转 = 10000 指令单位" 约定 (6091:01h=8388608, 02h=10000)
    { "WD40M-01030S-D1",   8388608UL,   8388608UL,  10000UL,
      0.0000066f, 0.318f },
    // 2 = 维控 WE130M-10025S-A1*: 1.0kW / 2500rpm / 4.0 N·m / 8.5e-4 kg·m2 / 17bit
    { "WE130M-10025S-A1",  131072UL,    1UL,        1UL,
      0.00085f,   4.0f  },
    // 3 = 汇川 MS1H2-10C30CB-T33*: 1.0kW / 3000rpm / 3.18 N·m / 1.78e-4 kg·m2 / 18bit
    { "MS1H2-10C30CB-T33", 262144UL,    1UL,        1UL,
      0.000178f,  3.18f },
    // 4 = 汇川 MS1H1-75B30CB-A6/S63*: 750W / 3000rpm / 2.39 N·m / 6.8e-5 kg·m2 / 26bit
    //     手册推荐 6091 = 8:1 → 1 电机转 = 8388608 指令单位
    { "MS1H1-75B30CB-A6",  67108864UL,  8UL,        1UL,
      0.000068f,  2.39f },
    // 5 = 维控 VD5E 驱动轴 23bit 绝对值编码器 (A 方案: 直接用编码器原始分辨率)
    //     1 电机转 = 8388608 指令单位 (6091 = 1:1), 与 17bit 的 131072 同一套口径
    //     换算: 8388608 / 195.349 = 42946 单位/mm (原 17bit 是 671 单位/mm)
    //     TODO(待补): 惯量/额定转矩现场测电机铭牌后回填, 目前只影响转矩扫频与惯量折算,
    //                 不影响送纸凸轮位置/速度前馈换算 (那些只看 ulEncCntPerRev)
    { "WE130-23B",         8388608UL,   1UL,        1UL,
      0.00085f,   4.0f  }
};

///////////////////////////////////////////////////////////////////////////////
const MOTOR_SPEC_T* MotorSpecGet(unsigned char ucMotor)
{
    if (ucMotor >= MOTOR_SPEC_COUNT) {
        ucMotor = MOTOR_WE130M_17B;
    }
    return &g_aMotorSpec[ucMotor];
}

///////////////////////////////////////////////////////////////////////////////
double MotorSpecUnitsPerRev(unsigned char ucMotor)
{
    const MOTOR_SPEC_T* pSpec = MotorSpecGet(ucMotor);
    unsigned int encCnt  = pSpec->ulEncCntPerRev;
    unsigned int gearNum = pSpec->ulGearNum;
    unsigned int gearDen = pSpec->ulGearDen;

    // 0 值非法: 按 1:1 兜底, 避免除零把整条换算链变成 inf/NaN
    if (encCnt == 0) {
        encCnt = 131072UL;
    }
    if (gearNum == 0) {
        gearNum = 1UL;
    }
    if (gearDen == 0) {
        gearDen = 1UL;
    }

    double units = (double)encCnt * (double)gearDen / (double)gearNum;
    if (units < 1.0) {
        units = 1.0;
    }
    return units;
}
