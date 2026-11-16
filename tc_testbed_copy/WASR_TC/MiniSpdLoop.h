///////////////////////////////////////////////////////////////////////////////
// MiniSpdLoop.h
// @brief  上位机速度环 (PI + 转矩滤波), 供转矩扫频实验保持基础速度。
//
// 伺服在 CST(10) 转矩模式下没有速度环, 由本模块输出转矩指令。
// 参数为编译期宏, 改参数需重新编译。
//
// 用法:
//   MiniSpdLoop loop;
//   loop.Init(dtSec, 总惯量, 额定转矩, 转矩限幅);
//   float torCmd = loop.Step(spdRef, spdFb);   // [0.1% 额定转矩]
//
// 单位: 速度 [rpm], 转矩 [0.1% 额定转矩], 增益 [Hz], 时间常数 [ms]。
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <stdint.h>

// 比例增益 [Hz] (对应伺服 Pn100)
#define MINISPD_KV_HZ        25.0f
// 积分时间常数 [ms] (对应伺服 Pn101)
#define MINISPD_TI_MS        23.0f
// 转矩指令滤波时间常数 [ms] (对应伺服 Pn102, 0 = 直通)
#define MINISPD_TF_MS        0.95f
// 比例项指令权重 [-] (1.0 = 纯 PI, 0.0 = 比例项只见反馈)
#define MINISPD_PDFF         1.0f

#define MINISPD_TWO_PI       6.283185307f

struct MiniSpdLoop
{
    // 控制周期 [s]
    float dtSec;
    // 机械折算增益 [0.1% / rpm] = 2π·Kv·InerConv
    float kvMech;
    // 积分时间常数 [s]
    float tiSec;
    // 转矩滤波时间常数 [s] (0 = 直通)
    float tfSec;
    // 转矩限幅 [0.1% 额定转矩]
    float torLim;
    // 比例项指令权重 [-]
    float pdfRate;
    // 积分器 [0.1% 额定转矩]
    float integral;
    // 转矩滤波状态 [0.1% 额定转矩]
    float filt;
    // 上拍滤波前输出 [0.1% 额定转矩]
    float outPreFilt;
    // 本拍输出 [0.1% 额定转矩]
    float out;
    // 上拍是否限幅 (1 = 限幅中)
    uint8_t clamped;

    /**
     * 初始化
     * @param dt          控制周期 [s]
     * @param inertia     总惯量 (转子惯量 × (1 + 惯量比)) [kg·m^2]
     * @param ratedTorque 电机额定转矩 [N·m] (<= 0 时按 1 处理)
     * @param lim         转矩限幅 [0.1% 额定转矩]
     */
    void Init(float dt, float inertia, float ratedTorque, float lim)
    {
        // 惯量下限保护
        float j = (inertia > 1e-12f) ? inertia : 1e-12f;
        // 惯量折算: 转矩[0.1%] = 1000·J·dω/dt / (60·Tr)
        float inerConv = (ratedTorque > 1e-12f)
                       ? (1000.0f * MINISPD_TWO_PI / (60.0f * ratedTorque)) * j
                       : j;
        kvMech = MINISPD_TWO_PI * MINISPD_KV_HZ * inerConv;

        dtSec        = dt;
        tiSec        = MINISPD_TI_MS * 0.001f;
        tfSec        = MINISPD_TF_MS * 0.001f;
        torLim       = (lim > 0.0f) ? lim : 0.0f;
        pdfRate      = MINISPD_PDFF;

        Reset();
    }

    /**
     * 复位 (进入实验或驱动器重新使能时调用)
     */
    void Reset()
    {
        integral   = 0.0f;
        filt       = 0.0f;
        outPreFilt = 0.0f;
        out        = 0.0f;
        clamped    = 0;
    }

    /**
     * 跑一拍
     * @param spdRef 速度指令 [rpm]
     * @param spdFb  速度反馈 [rpm]
     * @return 转矩指令 [0.1% 额定转矩] (已滤波并限幅)
     */
    float Step(float spdRef, float spdFb)
    {
        // 比例项误差 (pdfRate 加权指令)
        float errP = pdfRate * spdRef - spdFb;
        // 比例项输出
        float pOut = errP * kvMech;
        // 积分项误差
        float errI = spdRef - spdFb;

        // 抗积分饱和: 已限幅且误差继续推向限幅时不积分
        uint8_t allowInt = 1;
        if (clamped) {
            if ((outPreFilt > 0.0f && errI > 0.0f) ||
                (outPreFilt < 0.0f && errI < 0.0f)) {
                allowInt = 0;
            }
        }
        if (allowInt && tiSec > 1e-12f) {
            integral += errI * kvMech * dtSec / tiSec;
        }
        // 积分器限幅
        if (integral >  torLim) integral =  torLim;
        if (integral < -torLim) integral = -torLim;

        // 滤波前输出
        float raw = pOut + integral;
        outPreFilt = raw;

        // 滤波前限幅
        float lim = raw;
        if (lim >  torLim) lim =  torLim;
        if (lim < -torLim) lim = -torLim;
        clamped = (lim != raw) ? 1 : 0;

        // 一阶滤波
        if (tfSec > 1e-9f) {
            float alpha = dtSec / (tfSec + dtSec);
            if (alpha > 1.0f) alpha = 1.0f;
            filt += (lim - filt) * alpha;
        } else {
            filt = lim;
        }

        out = filt;
        return out;
    }
};
