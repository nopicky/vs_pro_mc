#include "TcPch.h"
#pragma hdrstop

#include "FeedSequencer.h"
#include <cmath>
#include <string.h>

/* 全局工艺参数实例（默认值）
   存取者：CFeedSequencer::init 读取快照；调试期经 ADS 在线修改后重新 init 生效
   注意：afSyncLen/afOvertravel 单位 mm；fStartPhaseComp 叠加在整圈对齐相位上 */
FEED_CONFIG_T g_cfgFeed = {
    1,                             /* nAxisCount：单轴 VD5L 先行 */
    195.349,                       /* fFeedRollerCircum */
    1188.78,                       /* fMasterCircum */
    90.0,                          /* fRollerPitch */
    195.349,                       /* fMmPerMotorRev */
    300.0,                         /* fAccelX：FAE 柔和版，威科达更短 */
    120.0,                         /* fAccelY：威科达参考 100 */
    300.0,                         /* fDecelX */
    120.0,                         /* fDecelY：威科达参考 25 */
    {430.0, 340.0, 250.0, 160.0},  /* afSyncLen：A1~A4 同步段长 */
    {10.0, 0.0, 0.0, 0.0},         /* afOvertravel：轴1 +10mm（科威达工艺） */
    0.0,                           /* fStartPhaseComp */
    350.0,                         /* fMaxLineSpeedMMin */
    true,                          /* bContinuous：默认连续循环（Start 撤销后当前循环走完即停） */
    false,                         /* bLoadSideEnable：默认不接负载伺服 */
    0.0                            /* fLeadTimeMs：超前补偿关闭（等效 T≈20ms，实测后从 10~25 试） */
};

/* 走完凸轮后保持终值的周期数（125us×8000 ≈ 1s），之后回 IDLE 等下一次 Start */
static const int c_nStopHoldTicks = 8000;

CFeedSequencer::CFeedSequencer() {

    m_eState = FEED_IDLE;
    m_bStart = false;
    m_fTriggerPhase = 0.0;
    m_fCamOrigin = 0.0;
    m_nStopTicks = 0;
    m_nAxisCount = 0;
    m_fMaxCamLen = 0.0;
    m_fPrevPhase = 0.0;
    m_bPrevValid = false;
    for (int i = 0; i < 4; i++) {
        m_afPosBase[i] = 0.0;
    }
    memset(&m_cfg, 0, sizeof(m_cfg));
    memset(&m_mon, 0, sizeof(m_mon));
}

bool CFeedSequencer::init(const FEED_CONFIG_T* pCfg) {

    if (pCfg == 0) {
        return false;
    }
    /* 参数校验：任一非法拒绝使能 */
    if (pCfg->fMasterCircum <= 0.0 || pCfg->fFeedRollerCircum <= 0.0) {
        return false;
    }
    if (pCfg->fMmPerMotorRev <= 0.0) {
        return false;
    }
    if (pCfg->nAxisCount < 1 || pCfg->nAxisCount > 4) {
        return false;
    }
    /* 只构建启用轴的凸轮，并记录最大凸轮总长（CAM_RUN 结束判据） */
    m_fMaxCamLen = 0.0;
    for (int i = 0; i < pCfg->nAxisCount; i++) {
        if (pCfg->afSyncLen[i] <= 0.0) {
            return false;
        }
        if (!m_aFeeder[i].build(pCfg->afSyncLen[i], pCfg->afOvertravel[i],
                                pCfg->fAccelX, pCfg->fAccelY,
                                pCfg->fDecelX, pCfg->fDecelY)) {
            return false;
        }
        if (m_aFeeder[i].getLength() > m_fMaxCamLen) {
            m_fMaxCamLen = m_aFeeder[i].getLength();
        }
    }
    m_nAxisCount = pCfg->nAxisCount;
    m_cfg = *pCfg;   /* 参数快照 */
    /* 状态与位移基准复位：init 即新会话（挂载层会重新采样指令原点），
       否则重新使能后旧累计位移叠加到新原点，607A 指令瞬间跳变（飞车风险） */
    m_bStart = false;
    m_nStopTicks = 0;
    m_fTriggerPhase = 0.0;
    m_fCamOrigin = 0.0;
    m_fPrevPhase = 0.0;
    m_bPrevValid = false;
    for (int i = 0; i < 4; i++) {
        m_afPosBase[i] = 0.0;
    }
    m_eState = FEED_IDLE;
    return true;
}

void CFeedSequencer::setStart(bool bStart) {

    m_bStart = bStart;
}

void CFeedSequencer::tick(double fMasterPhase, double fDtSec, const double* afActPosMm,
                          double afCmdPosMm[4]) {

    /* 主轴速度：相位差分（供超前补偿用；反向不计，v1 只做正向送纸）
       仅在上一拍有效时差分，避免 init 后首拍产生巨大虚假速度 */
    double fMasterVel = 0.0;
    if (m_bPrevValid && fDtSec > 1e-9) {
        fMasterVel = (fMasterPhase - m_fPrevPhase) / fDtSec;
        if (fMasterVel < 0.0) {
            fMasterVel = 0.0;
        }
    }
    m_fPrevPhase = fMasterPhase;
    m_bPrevValid = true;

    double fCamX = fMasterPhase - m_fCamOrigin;

    switch (m_eState) {
        case FEED_IDLE: {
            if (m_bStart) {
                m_eState = FEED_ARMED;
            }
        } break;

        case FEED_ARMED: {
            /* 整圈对齐：启动相位 = 主轴下一整圈 + 相位补偿（对齐 FAE 工程02） */
            double fCirc = m_cfg.fMasterCircum;
            double fTurns = floor(fMasterPhase / fCirc) + 1.0;
            m_fTriggerPhase = fTurns * fCirc + m_cfg.fStartPhaseComp;
            m_fCamOrigin = m_fTriggerPhase;
            m_eState = FEED_WAIT_PHASE;
        } break;

        case FEED_WAIT_PHASE: {
            /* 等相位期间撤销 Start: 直接回待命 (连续模式间隙停车, 不再续送下一张) */
            if (!m_bStart) {
                m_eState = FEED_IDLE;
            } else if (fMasterPhase >= m_fTriggerPhase) {
                m_eState = FEED_CAM_RUN;
            }
        } break;

        case FEED_CAM_RUN: {
            /* 以启用轴中最长凸轮走完为结束标志；各轴 evalPos 越界自动钳位 */
            if (fCamX >= m_fMaxCamLen) {
                /* 循环结束：各轴终值固化为位移基准（送纸辊单向连续旋转，指令不跳变） */
                for (int i = 0; i < m_nAxisCount; i++) {
                    m_afPosBase[i] += m_aFeeder[i].evalPos(m_aFeeder[i].getLength());
                }
                if (m_cfg.bContinuous && m_bStart) {
                    /* 连续模式：不停车，立即等主轴下一整圈自动续送（真实送纸节拍） */
                    m_eState = FEED_ARMED;
                } else {
                    m_eState = FEED_STOPPING;
                    m_nStopTicks = 0;
                }
            }
        } break;

        case FEED_STOPPING: {
            m_nStopTicks++;
            if (m_nStopTicks >= c_nStopHoldTicks) {
                /* 单次模式：位移基准已在 CAM_RUN 结束时累加，这里只回待命 */
                m_bStart = false;
                m_eState = FEED_IDLE;
            }
        } break;

        default: {
            m_eState = FEED_IDLE;   /* 非法状态防御 */
        } break;
    }

    /* 启用轴位置指令 = 位移基准 + 凸轮求值 + 超前补偿 + 补偿钩子（v1 恒 0）；未启用轴置 0
       IDLE 下凸轮相位按 0 处理（输出 = 位移基准，保持循环终点位置） */
    double fCamXEff = (m_eState == FEED_CAM_RUN || m_eState == FEED_STOPPING) ? fCamX : 0.0;
    double fLeadSec = m_cfg.fLeadTimeMs * 0.001;    /* ms → s */
    for (int i = 0; i < 4; i++) {
        if (i < m_nAxisCount) {
            /* 超前补偿（替代驱动器速度前馈）：指令前移「轴速度 × T」
               轴速度 = 速比×主轴速度，故前移量 = evalVel(x)·v_master·T（一阶泰勒，等价相位提前）
               凸轮两端速比为 0 → 起停无跳变；T=0 时与原实现完全一致 */
            double fLead = m_aFeeder[i].evalVel(fCamXEff) * fMasterVel * fLeadSec;
            afCmdPosMm[i] = m_afPosBase[i] + m_aFeeder[i].evalPos(fCamXEff) + fLead
                            + compensationHook(i);
        } else {
            afCmdPosMm[i] = 0.0;
        }
    }

    /* 监视量登记 */
    m_mon.nState = (int)m_eState;
    m_mon.fMasterPhase = fMasterPhase;
    m_mon.fCamX = fCamX;
    for (int i = 0; i < 4; i++) {
        m_mon.afCmdPosMm[i] = afCmdPosMm[i];
        if (afActPosMm != 0 && i < m_nAxisCount) {
            /* 对齐 FAE 显示习惯：跟随偏差 = 指令 - 实际 - 启动相位补偿 */
            m_mon.afFollowErr[i] = afCmdPosMm[i] - afActPosMm[i] - m_cfg.fStartPhaseComp;
        } else {
            m_mon.afFollowErr[i] = 0.0;
        }
    }
}

void CFeedSequencer::getMonitor(FEED_MONITOR_T* pOut) const {

    if (pOut != 0) {
        *pOut = m_mon;
    }
}

double CFeedSequencer::compensationHook(int nAxisIdx) const {

    (void)nAxisIdx;
    return 0.0;   /* v1：补偿预留插座，DOB/AI 后续在此叠加 */
}
