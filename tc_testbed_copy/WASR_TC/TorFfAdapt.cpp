///////////////////////////////////////////////////////////////////////////////
// TorFfAdapt.cpp
// 转矩前馈增益 LMS 学习器（方案 A：分时 + 残差扣除法）。
// 纯数学，不依赖 TwinCAT/PDO/全局变量，可用历史 CSV 离线回归测试。
///////////////////////////////////////////////////////////////////////////////
#include "TcPch.h"
#pragma hdrstop

#include "TorFfAdapt.h"

#include <cmath>

CTorFfAdapt::CTorFfAdapt()
    : m_bEnable(false)
    , m_eState(TORFF_ADAPT_OFF)
    , m_nPrevCycles(-1)
    , m_fSumPhi2(0.0)
    , m_fSumPhiE(0.0)
    , m_nAccSamples(0)
    , m_nAccSeen(0)
    , m_bCycleSaturated(false)
    , m_fTheta(1.0)
    , m_fGainOut(0.7)
    , m_fLastResidMm(0.0)
    , m_nFreezeCnt(0)
    , m_nCyclesLearned(0)
{
    init(NULL);
}

void CTorFfAdapt::init(const TORFF_ADAPT_CFG_T* pCfg)
{
    if (pCfg == NULL) {
        m_cfg.nLearnAxis        = 0;
        m_cfg.fMu               = 0.1;
        m_cfg.fLeak             = 0.999;
        m_cfg.fGainMin          = 0.2;
        m_cfg.fGainMax          = 1.2;
        m_cfg.fMaxStepPerCycle  = 0.05;
        m_cfg.fKacc             = 2.0e-4;    /* 机台标定（2026-11-26，见头文件注释） */
        m_cfg.fAccMinMmS2       = 10000.0;
        m_cfg.nAccSkipSamples   = 20;
        m_cfg.nMinAccSamples    = 40;
        m_cfg.fErrMaxMm         = 50.0;
        m_cfg.nFreezeCycles     = 3;
        m_cfg.fFreezeDelta      = 0.002;
    } else {
        m_cfg = *pCfg;
    }
    /* 参数合法性兜底 */
    if (m_cfg.fMu <= 0.0 || m_cfg.fMu > 1.0) { m_cfg.fMu = 0.1; }
    if (m_cfg.fLeak <= 0.0 || m_cfg.fLeak > 1.0) { m_cfg.fLeak = 0.999; }
    if (m_cfg.fGainMin <= 0.0)         { m_cfg.fGainMin = 0.2; }
    if (m_cfg.fGainMax <= m_cfg.fGainMin) { m_cfg.fGainMax = m_cfg.fGainMin + 0.1; }
    if (m_cfg.fMaxStepPerCycle <= 0.0) { m_cfg.fMaxStepPerCycle = 0.05; }
    if (m_cfg.fKacc <= 0.0)            { m_cfg.fKacc = 2.0e-4; }
    if (m_cfg.fAccMinMmS2 <= 0.0)      { m_cfg.fAccMinMmS2 = 10000.0; }
    if (m_cfg.nMinAccSamples < 1)      { m_cfg.nMinAccSamples = 40; }
}

double CTorFfAdapt::clampD(double v, double lo, double hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void CTorFfAdapt::reset(double fGainSeed)
{
    /* reset 即"开始一次新学习"：同时打开使能（与 VelFfAdapt 同款约定） */
    m_bEnable  = true;
    m_fGainOut = clampD(fGainSeed, m_cfg.fGainMin, m_cfg.fGainMax);
    m_fTheta   = clampD(1.0 / m_fGainOut,
                        1.0 / m_cfg.fGainMax, 1.0 / m_cfg.fGainMin);
    m_eState         = TORFF_ADAPT_LEARNING;
    m_nPrevCycles    = -1;
    m_fLastResidMm   = 0.0;
    m_nFreezeCnt     = 0;
    m_nCyclesLearned = 0;
    clearCycleAccum();
}

void CTorFfAdapt::setEnable(bool bEnable)
{
    m_bEnable = bEnable;
    if (!bEnable) {
        m_eState = TORFF_ADAPT_OFF;
    }
}

void CTorFfAdapt::clearCycleAccum()
{
    m_fSumPhi2        = 0.0;
    m_fSumPhiE        = 0.0;
    m_nAccSamples     = 0;
    m_nAccSeen        = 0;
    m_bCycleSaturated = false;
}

void CTorFfAdapt::tick(bool bCamRun, int nDoneCycles,
                       double fErrMm, double fVelPartMm,
                       double fAccMmS2, bool bSaturated)
{
    if (!m_bEnable) {
        return;
    }

    /* 重新启动（圈数被清 0）：本圈累计作废 */
    if (nDoneCycles < m_nPrevCycles) {
        m_nPrevCycles = nDoneCycles;
        clearCycleAccum();
    }
    /* reset 后第一拍：对齐圈数，不触发更新 */
    if (m_nPrevCycles < 0) {
        m_nPrevCycles = nDoneCycles;
        clearCycleAccum();
    }

    /* 圈末：用刚走完的这圈做一次批式 LMS 更新 */
    if (nDoneCycles > m_nPrevCycles) {
        m_nPrevCycles = nDoneCycles;
        if (m_eState == TORFF_ADAPT_LEARNING) {
            updateCycle();
        }
        clearCycleAccum();
        return;
    }

    if (!bCamRun) {
        m_nAccSeen = 0;
        return;
    }

    /* 特有红线 4：本圈 60B2 顶过限幅 → 整圈作废。
       顶限幅时误差不再随 TF 变化，学了就是"误以为 TF 没用"无限上调 */
    if (bSaturated) {
        m_bCycleSaturated = true;
    }

    /* ---- 加减速段检测：运动学量（凸轮解析加速度），无噪声 ---- */
    if (fabs(fAccMmS2) < m_cfg.fAccMinMmS2) {
        m_nAccSeen = 0;
        return;
    }
    m_nAccSeen++;
    /* 跳过段首过渡（进加速/进减速那几拍误差还是上一段的尾巴） */
    if (m_nAccSeen <= m_cfg.nAccSkipSamples) {
        return;
    }

    /* 残差扣除：e_res = e − 速度项该占的部分 */
    const double fResMm = fErrMm - fVelPartMm;
    if (fabs(fResMm) > m_cfg.fErrMaxMm) {
        return;
    }

    /* φ = α·Kacc（带符号：加速+、减速−；e_res 减速段也是负的，
       φ·e_res 两段同号累加，激励覆盖翻倍） */
    const double fPhi = fAccMmS2 * m_cfg.fKacc;
    m_fSumPhi2 += fPhi * fPhi;
    m_fSumPhiE += fPhi * fResMm;
    m_nAccSamples++;
}

void CTorFfAdapt::updateCycle()
{
    /* 顶过限幅 / 样本不足：这圈不学、不计圈 */
    if (m_bCycleSaturated || m_nAccSamples < m_cfg.nMinAccSamples) {
        return;
    }

    /* 批式 LMS：
       模型 e_res = (1−θ·TF)·φ → y = φ − e_res = θ·TF·φ
       残差 res = Σφ·y − θ̂·TF·Σφ² = (A1 − A2) − θ̂·TF·A1
       （θ̂、TF 圈内不变，求和可直接搬出来） */
    const double fA1  = m_fSumPhi2;
    const double fA2  = m_fSumPhiE;
    const double fRes = (fA1 - fA2) - m_fTheta * m_fGainOut * fA1;
    const double dTheta = m_cfg.fMu * fRes / (fA1 + 1.0);

    /* 泄漏 + 更新 + theta 限幅 */
    m_fTheta = m_cfg.fLeak * m_fTheta + dTheta;
    m_fTheta = clampD(m_fTheta,
                      1.0 / m_cfg.fGainMax, 1.0 / m_cfg.fGainMin);

    /* 输出 TF = 1/θ̂，每圈步长限幅 */
    const double gTarget = 1.0 / m_fTheta;
    double       gStep   = gTarget - m_fGainOut;
    gStep = clampD(gStep, -m_cfg.fMaxStepPerCycle, m_cfg.fMaxStepPerCycle);
    const double gNew = clampD(m_fGainOut + gStep,
                               m_cfg.fGainMin, m_cfg.fGainMax);

    /* 冻结判据 */
    if (fabs(gNew - m_fGainOut) < m_cfg.fFreezeDelta) {
        if (++m_nFreezeCnt >= m_cfg.nFreezeCycles) {
            m_eState = TORFF_ADAPT_FROZEN;
        }
    } else {
        m_nFreezeCnt = 0;
    }

    m_fGainOut      = gNew;
    m_fLastResidMm  = fRes;
    m_nCyclesLearned++;
}
