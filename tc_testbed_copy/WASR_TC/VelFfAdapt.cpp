///////////////////////////////////////////////////////////////////////////////
// VelFfAdapt.cpp
// LMS 自动学速度前馈增益的实现。纯数学，不依赖 TwinCAT/PDO/全局变量，
// 所有输入经 tick() 参数传入 —— 可以脱离控制器用历史 CSV 直接回归测试。
///////////////////////////////////////////////////////////////////////////////
#include "TcPch.h"
#pragma hdrstop

#include "VelFfAdapt.h"

#include <cmath>

CVelFfAdapt::CVelFfAdapt()
    : m_bEnable(false)
    , m_eState(VELFF_ADAPT_OFF)
    , m_nPrevCycles(-1)
    , m_fSumErrMm(0.0)
    , m_fSumVelMmS(0.0)
    , m_nSyncSamples(0)
    , m_nSyncSeen(0)
    , m_fTheta(1.0)
    , m_fGainOut(1.0)
    , m_fLastResidMmS(0.0)
    , m_nFreezeCnt(0)
    , m_nCyclesLearned(0)
{
    /* 默认值：全部来自机台实测（2026-11-25，见 FeedConfig.h 与「前馈调参阶段报告.md」） */
    init(NULL);
}

void CVelFfAdapt::init(const VELFF_ADAPT_CFG_T* pCfg)
{
    if (pCfg == NULL) {
        m_cfg.nLearnAxis        = 0;
        m_cfg.fKv               = 47.8;
        m_cfg.fMu               = 0.1;
        m_cfg.fLeak             = 0.999;
        m_cfg.fGainMin          = 0.5;
        m_cfg.fGainMax          = 1.5;
        m_cfg.fMaxStepPerCycle  = 0.05;
        m_cfg.fSyncVelMinMmS    = 1000.0;
        m_cfg.fSyncAccMaxMmS2   = 5000.0;
        m_cfg.nSyncSkipSamples  = 60;
        m_cfg.nMinSyncSamples   = 30;
        m_cfg.fErrMaxMm         = 50.0;
        m_cfg.nFreezeCycles     = 3;
        m_cfg.fFreezeDelta      = 0.002;
    } else {
        m_cfg = *pCfg;
    }
    /* 参数合法性兜底：非法就退回默认，宁可不学也不拿野参数学 */
    if (m_cfg.fKv <= 0.0)              { m_cfg.fKv = 47.8; }
    if (m_cfg.fMu <= 0.0 || m_cfg.fMu > 1.0) { m_cfg.fMu = 0.1; }
    if (m_cfg.fLeak <= 0.0 || m_cfg.fLeak > 1.0) { m_cfg.fLeak = 0.999; }
    if (m_cfg.fGainMin <= 0.0)         { m_cfg.fGainMin = 0.5; }
    if (m_cfg.fGainMax <= m_cfg.fGainMin) { m_cfg.fGainMax = m_cfg.fGainMin + 0.1; }
    if (m_cfg.fMaxStepPerCycle <= 0.0) { m_cfg.fMaxStepPerCycle = 0.05; }
    if (m_cfg.nMinSyncSamples < 1)     { m_cfg.nMinSyncSamples = 30; }
}

double CVelFfAdapt::clampD(double v, double lo, double hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void CVelFfAdapt::reset(double fGainSeed)
{
    /* reset 即视为"开始一次新的学习"：同时打开使能。
       ★ 2026-11-26 现场坑：reset 只置了 LEARNING，m_bEnable 还是构造时的 false，
         tick() 第一行就 return —— 表现为"状态=1 但增益永远不动"。 */
    m_bEnable  = true;
    /* 种子增益夹进合法区间；theta = 1/g（g 是 theta 的倒数关系，见头文件原理段） */
    m_fGainOut = clampD(fGainSeed, m_cfg.fGainMin, m_cfg.fGainMax);
    m_fTheta   = clampD(1.0 / m_fGainOut,
                        1.0 / m_cfg.fGainMax, 1.0 / m_cfg.fGainMin);
    m_eState        = VELFF_ADAPT_LEARNING;
    m_nPrevCycles   = -1;               /* 等 tick 对齐当前圈数，防误触发一次假更新 */
    m_fLastResidMmS = 0.0;
    m_nFreezeCnt    = 0;
    m_nCyclesLearned = 0;
    clearCycleAccum();
}

void CVelFfAdapt::setEnable(bool bEnable)
{
    m_bEnable = bEnable;
    if (!bEnable) {
        m_eState = VELFF_ADAPT_OFF;
    }
}

void CVelFfAdapt::clearCycleAccum()
{
    m_fSumErrMm    = 0.0;
    m_fSumVelMmS   = 0.0;
    m_nSyncSamples = 0;
    m_nSyncSeen    = 0;
}

void CVelFfAdapt::tick(bool bCamRun, int nDoneCycles,
                       double fErrMm, double fVelMmS, double fAccMmS2)
{
    if (!m_bEnable) {
        return;
    }

    /* 重新启动（圈数被清 0）：本圈累计作废，等下一次圈末再学 */
    if (nDoneCycles < m_nPrevCycles) {
        m_nPrevCycles = nDoneCycles;
        clearCycleAccum();
    }
    /* reset 后第一拍：对齐圈数，不触发更新 */
    if (m_nPrevCycles < 0) {
        m_nPrevCycles = nDoneCycles;
        clearCycleAccum();
    }

    /* 圈末：用刚走完的这圈的同步段均值做一次 LMS 更新 */
    if (nDoneCycles > m_nPrevCycles) {
        m_nPrevCycles = nDoneCycles;
        if (m_eState == VELFF_ADAPT_LEARNING) {
            updateCycle();
        }
        clearCycleAccum();
        return;                         /* 圈末这一拍不再计入下一圈 */
    }

    /* 非凸轮运行段（停等/ARMED/STOPPING/冻结）：不学，但保持窗口计数清零，
       否则跨段的假同步拍会污染下一圈 */
    if (!bCamRun) {
        m_nSyncSeen = 0;
        return;
    }

    /* ---- 同步段检测：运动学量，无噪声（曲率×v^2 在匀速/停等段恒为 0）---- */
    const bool bInSync = (fVelMmS >= m_cfg.fSyncVelMinMmS) &&
                         (fabs(fAccMmS2) <= m_cfg.fSyncAccMaxMmS2);
    if (!bInSync) {
        m_nSyncSeen = 0;
        return;
    }

    m_nSyncSeen++;
    /* 跳过镇定段：进同步后的过冲回落是瞬态误差，不是稳态样本 */
    if (m_nSyncSeen <= m_cfg.nSyncSkipSamples) {
        return;
    }
    /* 野点剔除：误差超出上限 = 跳闸前兆或采集异常，这一拍不学 */
    if (fabs(fErrMm) > m_cfg.fErrMaxMm) {
        return;
    }
    m_fSumErrMm  += fErrMm;
    m_fSumVelMmS += fVelMmS;
    m_nSyncSamples++;
}

void CVelFfAdapt::updateCycle()
{
    /* 样本不足（圈被截断/同步段太短）：这圈不学、不计圈 */
    if (m_nSyncSamples < m_cfg.nMinSyncSamples) {
        return;
    }

    const double vBar = m_fSumVelMmS / m_nSyncSamples;  /* 本圈同步段速度均值 */
    const double eBar = m_fSumErrMm  / m_nSyncSamples;  /* 本圈同步段误差均值 */

    /* 观测：前馈实际兑现的速度 y = v - e*Kv（e>0 = 实际落后 = 兑现不足） */
    const double y    = vBar - m_cfg.fKv * eBar;
    /* 预测：theta * v * g（按当前估计参数应兑现的速度） */
    const double pred = m_fTheta * vBar * m_fGainOut;
    const double res  = y - pred;
    /* 归一化 LMS：除以 v^2 后步长与线速度无关（290/350 不用改 mu） */
    const double denom = vBar * vBar + 1.0;
    const double dTheta = m_cfg.fMu * res * vBar / denom;

    /* 泄漏 + 更新 + theta 限幅（红线 1、2） */
    m_fTheta = m_cfg.fLeak * m_fTheta + dTheta;
    m_fTheta = clampD(m_fTheta,
                      1.0 / m_cfg.fGainMax, 1.0 / m_cfg.fGainMin);

    /* 输出增益 = 1/theta，每圈步长限幅（一圈野数据最多挪 5%） */
    const double gTarget = 1.0 / m_fTheta;
    double       gStep   = gTarget - m_fGainOut;
    gStep = clampD(gStep, -m_cfg.fMaxStepPerCycle, m_cfg.fMaxStepPerCycle);
    const double gNew = clampD(m_fGainOut + gStep,
                               m_cfg.fGainMin, m_cfg.fGainMax);

    /* 冻结判据（红线 3）：连续 N 圈步长足够小 → 认为收敛，停止追噪声 */
    if (fabs(gNew - m_fGainOut) < m_cfg.fFreezeDelta) {
        if (++m_nFreezeCnt >= m_cfg.nFreezeCycles) {
            m_eState = VELFF_ADAPT_FROZEN;
        }
    } else {
        m_nFreezeCnt = 0;
    }

    m_fGainOut     = gNew;
    m_fLastResidMmS = res;
    m_nCyclesLearned++;
}
