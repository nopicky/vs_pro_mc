///////////////////////////////////////////////////////////////////////////////
// MasterEnc.cpp
// 真实主轴编码器 → 凸轮周期相位。纯数学，不依赖 TwinCAT/PDO/全局变量。
///////////////////////////////////////////////////////////////////////////////
#include "TcPch.h"
#pragma hdrstop

#include "MasterEnc.h"

#include <cmath>
#include <stdlib.h>

CMasterEnc::CMasterEnc()
    : m_bValid(false)
    , m_nPrevPos(0)
    , m_fPhaseMm(0.0)
    , m_fVelMmS(0.0)
    , m_nBadSamples(0)
{
    init(NULL);
}

void CMasterEnc::init(const MASTER_ENC_CFG_T* pCfg)
{
    if (pCfg == NULL) {
        m_cfg.fCountsPerRev   = 8388608.0;   /* VD5E 23bit */
        m_cfg.fMmPerRev       = 237.75;      /* 22:27 齿比折算（FAE 2026-11-26） */
        m_cfg.fCycleLenMm     = 1188.78;     /* 凸轮周期行程 = fMasterCircum */
        m_cfg.fMaxDeltaCounts = 8388608L;    /* 丢帧阈值：1 整转/拍（正常 ~3万 counts/拍） */
    } else {
        m_cfg = *pCfg;
    }
    /* 参数合法性兜底：非法就退回默认，宁可按默认换算也不能除零 */
    if (m_cfg.fCountsPerRev <= 0.0)  { m_cfg.fCountsPerRev   = 8388608.0; }
    if (m_cfg.fMmPerRev <= 0.0)      { m_cfg.fMmPerRev       = 237.75; }
    if (m_cfg.fCycleLenMm <= 0.0)    { m_cfg.fCycleLenMm     = 1188.78; }
    if (m_cfg.fMaxDeltaCounts <= 0)  { m_cfg.fMaxDeltaCounts = 8388608L; }
}

double CMasterEnc::tick(long nPos6064, double fDtSec)
{
    /* 首拍：只记录基准，不产生虚假速度/相位跳变 */
    if (!m_bValid) {
        m_nPrevPos = nPos6064;
        m_bValid   = true;
        m_fVelMmS  = 0.0;
        return m_fPhaseMm;
    }

    /* int32 安全差分：6064 是 int32 累积值，跨 2^31 回绕时
       "无符号减再转回有符号"得到的增量依然正确（主轴永不清零的依据） */
    const unsigned long du = (unsigned long)nPos6064 - (unsigned long)m_nPrevPos;
    const long d = (long)du;
    m_nPrevPos = nPos6064;

    /* 丢帧/异常剔除：单拍增量超阈值 = 采到坏样本。
       该拍冻结相位（保持上一拍值），计数供诊断 —— 不让坏样本污染凸轮 */
    if (labs(d) > m_cfg.fMaxDeltaCounts) {
        m_nBadSamples++;
        return m_fPhaseMm;
    }

    /* 累计增量 + 自己绕到 [0, fCycleLenMm)。
       负增量（主轴倒转）也允许：相位反向绕，由下游凸轮解缠的
       "负差分按 0 处理"做最后防线（凸轮单向送纸） */
    const double fMmPerCount = m_cfg.fMmPerRev / m_cfg.fCountsPerRev;
    const double dMm = (double)d * fMmPerCount;
    m_fPhaseMm += dMm;
    while (m_fPhaseMm >= m_cfg.fCycleLenMm) { m_fPhaseMm -= m_cfg.fCycleLenMm; }
    while (m_fPhaseMm < 0.0)                { m_fPhaseMm += m_cfg.fCycleLenMm; }

    /* 速度回显（不参与控制；dt 异常时保持上一次的值） */
    if (fDtSec > 1e-9) {
        m_fVelMmS = dMm / fDtSec;
    }
    return m_fPhaseMm;
}
