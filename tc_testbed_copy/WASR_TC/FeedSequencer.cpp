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
    400.0,                         /* fMaxLineSpeedMMin */
    true,                          /* bContinuous：默认连续循环（Start 撤销后当前循环走完即停） */
    false,                         /* bLoadSideEnable：默认不接负载伺服 */
    0.0,                           /* fLeadTimeMs：已改用驱动器 60B1h 前馈，指令超前归 0
                                      (要回到老方案: 这里填 20.3, fVelFfGain 填 0) */
    0.0,                           /* fLeadAccMs2：加速度前馈默认关（扫参起点 0.05~0.2） */
    1.0,                           /* fVelFfGain：速度前馈满量（0=关闭） */
    0.0,                           /* fVelFfAccGainMs：加速度前馈项默认关（方案B，扫参 3~15） */
    200.0,                         /* fMaxFollowErrMm：跟随偏差跳闸阈值（≈1/4 辊周长），0=关闭 */
    /* ---- 方案 C：转矩前馈 ---- */
    0.0,                           /* fTorFfGain：转矩前馈默认关。
                                      先用 bLiveTuning=1 + Status.TorFfGain 在线扫，
                                      定了再把这里改成最终值当上电默认——
                                      别一上电就带转矩前馈，惯量标错会直接冲击机械 */
    {2.72, 2.73, 2.65, 2.74}       /* fInertiaRatio：4 台 VD5E 实测 2003-01 / 100
                                      (2026-11-24, 对拖机械连好、3E 不使能状态测) */
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
    m_fPhaseUnwrapped = 0.0;
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
    m_fPhaseUnwrapped = 0.0;
    for (int i = 0; i < 4; i++) {
        m_afPosBase[i] = 0.0;
    }
    m_eState = FEED_IDLE;
    return true;
}

void CFeedSequencer::setStart(bool bStart) {

    m_bStart = bStart;
}

/* 在线调参：刷 m_cfg 快照里的补偿参数（不必重新 init / 重编译）
   传负值表示"这一项不动"：上位机只改一个旋钮时不用把其它项也背一遍
   fLeadTimeMs 限速 0.05ms/拍：位置指令每拍最多跳 v×0.05ms（5840mm/s 时约 0.29mm），
   即使手滑输个大值也是斜坡过去，不会一步跳过去打死机械 */
void CFeedSequencer::setLiveTuning(double fLeadTimeMs, double fLeadAccMs2,
                                   double fVelFfAccGainMs) {

    if (fLeadTimeMs >= 0.0) {
        double fStep = fLeadTimeMs - m_cfg.fLeadTimeMs;
        if (fStep >  0.05) { fStep =  0.05; }
        if (fStep < -0.05) { fStep = -0.05; }
        m_cfg.fLeadTimeMs += fStep;
    }
    if (fLeadAccMs2 >= 0.0) {
        m_cfg.fLeadAccMs2 = fLeadAccMs2;
    }
    if (fVelFfAccGainMs >= 0.0) {
        m_cfg.fVelFfAccGainMs = fVelFfAccGainMs;
    }
}

void CFeedSequencer::tick(double fMasterPhase, double fDtSec, const double* afActPosMm,
                          double afCmdPosMm[4]) {

    /* ---- 主轴相位解缠: 外部相位 -> 内部单调累加相位 ----
       虚拟主轴是无界累加的, 现场主轴相位却来自滚筒编码器, 取值在 [0, 周长) 内循环。
       下游(整圈对齐 floor(相位/周长)、凸轮相位差、超前补偿)都要求单调相位,
       所以统一在这里解缠一次, 调用方给哪种都行:
         - 编码器值: 跨圈那一拍差分是 -周长, 补一个周长回来
         - 虚拟主轴: 差分恒为正, 解缠结果与原值一致 (差异仅为浮点累加误差)
       反向不支持(凸轮单向送纸): 负差分按 0 处理, 相位冻结,
       此时指令与实际会拉开, 由跟随偏差暴露成故障, 不会反向冲纸 */
    double fMasterVel = 0.0;
    if (!m_bPrevValid) {
        /* 首拍: 没有历史可比, 直接以输入值起算, 不产生虚假速度 */
        m_fPhaseUnwrapped = fMasterPhase;
        m_bPrevValid = true;
    } else {
        double fDelta = fMasterPhase - m_fPrevPhase;
        double fHalfCirc = 0.5 * m_cfg.fMasterCircum;
        if (fDelta < -fHalfCirc) {
            fDelta += m_cfg.fMasterCircum;   /* 正转跨圈: 相位回绕 */
        } else if (fDelta > fHalfCirc) {
            fDelta -= m_cfg.fMasterCircum;   /* 反转跨圈(下面按 0 处理) */
        }
        if (fDelta < 0.0) {
            fDelta = 0.0;
        }
        if (fDtSec > 1e-9) {
            fMasterVel = fDelta / fDtSec;
            /* 上限: 1.5 × 配置线速度上限。只挡坏样本(相位跳变/丢帧),
               正常加速有 50% 余量, 不会削掉真实速度 */
            double fVelMax = m_cfg.fMaxLineSpeedMMin * 1000.0 / 60.0 * 1.5;
            if (fVelMax > 0.0 && fMasterVel > fVelMax) {
                fMasterVel = fVelMax;
            }
        }
        m_fPhaseUnwrapped += fDelta;
    }
    m_fPrevPhase = fMasterPhase;

    /* 以下全部用解缠后的单调相位 */
    const double fPhase = m_fPhaseUnwrapped;
    double fCamX = fPhase - m_fCamOrigin;

    switch (m_eState) {
        case FEED_IDLE: {
            if (m_bStart) {
                m_eState = FEED_ARMED;
            }
        } break;

        case FEED_ARMED: {
            /* 整圈对齐：启动相位 = 主轴下一整圈 + 相位补偿（对齐 FAE 工程02） */
            double fCirc = m_cfg.fMasterCircum;
            double fTurns = floor(fPhase / fCirc) + 1.0;
            m_fTriggerPhase = fTurns * fCirc + m_cfg.fStartPhaseComp;
            m_fCamOrigin = m_fTriggerPhase;
            m_eState = FEED_WAIT_PHASE;
        } break;

        case FEED_WAIT_PHASE: {
            /* 等相位期间撤销 Start: 直接回待命 (连续模式间隙停车, 不再续送下一张) */
            if (!m_bStart) {
                m_eState = FEED_IDLE;
            } else if (fPhase >= m_fTriggerPhase) {
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
       注意：只有 CAM_RUN 才叠加凸轮求值。STOPPING/IDLE 必须按 camX=0 处理，
       因为凸轮终值已在 CAM_RUN 结束时累加进 m_afPosBase——若 STOPPING 仍叠加
       evalPos(越界钳位=终值)，指令会多算一个「整段位移」(+680mm)，回 IDLE 时
       再减去，形成「先正冲、后倒退」的两次大跳变（实测 ±450k 指令单位） */
    double fCamXEff = (m_eState == FEED_CAM_RUN) ? fCamX : 0.0;
    double fLeadSec = m_cfg.fLeadTimeMs * 0.001;    /* ms → s */
    double afIdealMm[4] = {0.0, 0.0, 0.0, 0.0};
    double afVelFf[4]   = {0.0, 0.0, 0.0, 0.0};
    double afAccMm[4]   = {0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < 4; i++) {
        if (i < m_nAxisCount) {
            /* 速比一次求值, 位置前馈(指令超前)与速度前馈共用, 避免两处口径漂移 */
            double fRatio = m_aFeeder[i].evalVel(fCamXEff);
            /* 凸轮理论位置（不含超前）：工艺期望的实际位置 */
            double fIdeal = m_afPosBase[i] + m_aFeeder[i].evalPos(fCamXEff);
            /* 超前补偿（替代驱动器速度前馈）：指令前移「轴速度 × T」
               轴速度 = 速比×主轴速度，故前移量 = evalVel(x)·v_master·T（一阶泰勒，等价相位提前）
               凸轮两端速比为 0 → 起停无跳变；T=0 时与原实现完全一致 */
            double fLead = fRatio * fMasterVel * fLeadSec;
            /* 二阶项：前移量 = T·v + k·a（a = 凸轮曲率 × 主轴速度²）
               实测（350m/min, T=20.3ms）加速段残余 +9.2mm（实际落后）、
               减速段残余 -6.8mm（实际超前）——两者都说明「轴跟不上速度变化」，
               故加速(a>0)要多给超前、减速(a<0)要少给，符号为 **加**。
               同步段曲率为 0 → 该项不影响稳态段，只修加减速过渡。
               k 的量纲是 ms²，起点建议 30~60（k=50 时在 350 档约补 9mm） */
            /* 从轴加速度 [mm/s²]：位置超前二阶项、速度前馈加速度项(方案B)、
               转矩前馈(方案C) 三者共用 —— 既然都在这条链上，就只求一次。
               evalAcc 是凸轮查表二阶导，开销小；无条件算比"按需判断"更好维护：
               以后再加一个用加速度的补偿，不会因为忘了改这个 if 而静默算出 0。 */
            double fAccAxis = m_aFeeder[i].evalAcc(fCamXEff) * fMasterVel * fMasterVel;
            if (m_cfg.fLeadAccMs2 > 0.0) {
                fLead += fAccAxis * (m_cfg.fLeadAccMs2 * 1.0e-6);
            }
            afIdealMm[i] = fIdeal;
            afCmdPosMm[i] = fIdeal + fLead + compensationHook(i);
            /* 速度前馈 = 轴速度 [mm/s]，只表示"这一刻该跑多快"，与 T 无关；
               非 CAM_RUN 强制 0 (残留的速度偏置会让电机自己跑)
               加速度项(方案B)：+ Ka·a，把加速段"该多跑的那点速度"提前给出，
               位置环不用攒误差去追 → 消掉进同步段的过冲。
               量纲: a[mm/s²] × Ka[ms] × 1e-3 = mm/s */
            afVelFf[i] = (m_eState == FEED_CAM_RUN)
                        ? (fRatio * fMasterVel + m_cfg.fVelFfAccGainMs * 1.0e-3 * fAccAxis)
                        : 0.0;
            /* 加速度只上报运动学值，不做电机折算 —— 停/非 RUN 一律 0，
               否则挂载层会在起停段按凸轮起点曲率算出一份假转矩前馈 */
            afAccMm[i] = (m_eState == FEED_CAM_RUN) ? fAccAxis : 0.0;
        } else {
            afCmdPosMm[i] = 0.0;
        }
    }

    /* 监视量登记 */
    m_mon.nState = (int)m_eState;
    m_mon.fMasterPhase = fPhase;   /* 解缠后的单调相位（Scope 上连续，不会每圈跳回 0） */
    m_mon.fCamX = fCamX;
    for (int i = 0; i < 4; i++) {
        m_mon.afCmdPosMm[i] = afCmdPosMm[i];
        m_mon.afVelFfMm[i]  = afVelFf[i];
        m_mon.afAccAxisMm[i]= afAccMm[i];
        if (afActPosMm != 0 && i < m_nAxisCount) {
            /* 对齐 FAE 显示习惯：跟随偏差 = 指令 - 实际 - 启动相位补偿 */
            m_mon.afFollowErr[i] = afCmdPosMm[i] - afActPosMm[i] - m_cfg.fStartPhaseComp;
            /* 理想偏差 = 凸轮理论位置 - 实际（正 = 实际落后于曲线） */
            m_mon.afIdealErr[i] = afIdealMm[i] - afActPosMm[i];
        } else {
            m_mon.afFollowErr[i] = 0.0;
            m_mon.afIdealErr[i] = 0.0;
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
