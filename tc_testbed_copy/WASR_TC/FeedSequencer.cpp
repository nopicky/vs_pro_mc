#include "FeedSequencer.h"
#include <cmath>
#include <string.h>

/* 全局工艺参数实例（默认值）
   存取者：CFeedSequencer::init 读取快照；调试期经 ADS 在线修改后重新 init 生效
   注意：afSyncLen/afOvertravel 单位 mm；fStartPhaseComp 叠加在整圈对齐相位上 */
FEED_CONFIG_T g_cfgFeed = {
    195.349,                       /* fFeedRollerCircum */
    1188.78,                       /* fMasterCircum */
    90.0,                          /* fRollerPitch */
    {430.0, 340.0, 250.0, 160.0},  /* afSyncLen：A1~A4 同步段长 */
    {10.0, 0.0, 0.0, 0.0},         /* afOvertravel：轴1 +10mm（科威达工艺） */
    0.0,                           /* fStartPhaseComp */
    17,                            /* nEncoderBits */
    350.0,                         /* fMaxLineSpeedMMin */
    false                          /* bLoadSideEnable：默认不接负载伺服 */
};

/* 走完凸轮后保持终值的周期数（125us×8000 ≈ 1s），之后回 IDLE 等下一次 Start */
static const int c_nStopHoldTicks = 8000;

CFeedSequencer::CFeedSequencer() {

    m_eState = FEED_IDLE;
    m_bStart = false;
    m_fTriggerPhase = 0.0;
    m_fCamOrigin = 0.0;
    m_nStopTicks = 0;
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
    if (pCfg->nEncoderBits != 17 && pCfg->nEncoderBits != 23) {
        return false;
    }
    for (int i = 0; i < 4; i++) {
        if (pCfg->afSyncLen[i] <= 0.0) {
            return false;
        }
        if (!m_aFeeder[i].build(pCfg->afSyncLen[i], pCfg->afOvertravel[i])) {
            return false;
        }
    }
    m_cfg = *pCfg;   /* 参数快照 */
    m_eState = FEED_IDLE;
    return true;
}

void CFeedSequencer::setStart(bool bStart) {

    m_bStart = bStart;
}

void CFeedSequencer::tick(double fMasterPhase, double fDtSec, const double* afActPosMm,
                          double afCmdPosMm[4]) {

    (void)fDtSec;   /* v1 未用（固定周期），预留变速周期 */
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
            if (fMasterPhase >= m_fTriggerPhase) {
                m_eState = FEED_CAM_RUN;
            }
        } break;

        case FEED_CAM_RUN: {
            /* 以总长最大的轴（轴1）走完为结束标志；各轴 evalPos 越界自动钳位 */
            if (fCamX >= m_aFeeder[0].getLength()) {
                m_eState = FEED_STOPPING;
                m_nStopTicks = 0;
            }
        } break;

        case FEED_STOPPING: {
            m_nStopTicks++;
            if (m_nStopTicks >= c_nStopHoldTicks) {
                m_bStart = false;
                m_eState = FEED_IDLE;
            }
        } break;

        default: {
            m_eState = FEED_IDLE;   /* 非法状态防御 */
        } break;
    }

    /* 4 轴位置指令：凸轮求值 + 补偿钩子（v1 恒 0） */
    for (int i = 0; i < 4; i++) {
        afCmdPosMm[i] = m_aFeeder[i].evalPos(fCamX) + compensationHook(i);
    }

    /* 监视量登记 */
    m_mon.nState = (int)m_eState;
    m_mon.fMasterPhase = fMasterPhase;
    m_mon.fCamX = fCamX;
    for (int i = 0; i < 4; i++) {
        m_mon.afCmdPosMm[i] = afCmdPosMm[i];
        if (afActPosMm != 0) {
            /* 对齐 FAE 显示习惯：跟随偏差 = 指令 - 实际 - 启动相位补偿 */
            m_mon.afFollowErr[i] = afCmdPosMm[i] - afActPosMm[i] - m_cfg.fStartPhaseComp;
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
