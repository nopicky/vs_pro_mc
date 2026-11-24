#ifndef FEED_SEQUENCER_H__20261117
#define FEED_SEQUENCER_H__20261117

#include "FeedConfig.h"
#include "CamProfile.h"

/* 送纸工艺状态（复刻 FAE 状态机 Num 0~6，启动逻辑对齐 FAE 工程02） */
typedef enum tagFEED_STATE_E {
    FEED_IDLE = 0,      /* 等 Start（上位机变量） */
    FEED_ARMED,         /* 锁定启动相位：主轴下一整圈 + 相位补偿 */
    FEED_WAIT_PHASE,    /* 等主轴走到启动相位 */
    FEED_CAM_RUN,       /* 4 轴凸轮输出中 */
    FEED_STOPPING       /* 走完保持终值，延时后回 IDLE */
} FEED_STATE_E;

/* 监视量：供 Scope 录波与上位机显示 */
typedef struct tagFEED_MONITOR_T {
    int    nState;          /* 当前状态 FEED_STATE_E */
    double fMasterPhase;    /* 主轴相位 mm */
    double fCamX;           /* 凸轮内部相位 mm（主轴相位 - 启动原点） */
    double afCmdPosMm[4];   /* 4 轴位置指令 mm（含超前补偿） */
    double afFollowErr[4];  /* 4 轴跟随偏差 mm（指令-实际，对齐 FAE 习惯扣除启动相位补偿） */
    double afIdealErr[4];   /* 4 轴相对凸轮理论曲线的偏差 mm（理想-实际，正=实际落后）
                               —— 评价超前补偿/前馈效果看这个，不是 afFollowErr */
    double afVelFfMm[4];    /* 4 轴速度前馈 mm/s（凸轮速比 × 主轴速度；非 CAM_RUN 恒 0）
                               —— 挂载层乘 mm→指令单位换算后写 60B1h */
} FEED_MONITOR_T;

/* 送纸工艺状态机：只依赖主轴相位输入（虚拟主轴 / 现场编码器均可）与 CCamFeeder，
   不碰 PDO —— 换主轴来源不动这一层 */
class CFeedSequencer {
public:
    CFeedSequencer();

    /* 按配置构建 4 个凸轮并做全参数校验，任一非法返回 false（禁止使能）
       配置做快照，运行期改 g_cfgFeed 不影响已 init 的实例 */
    bool init(const FEED_CONFIG_T* pCfg);

    /* 上位机启动信号（ADS/全局变量轮询写入） */
    void setStart(bool bStart);

    /* 在线调参：把可运行期修改的补偿参数刷进 m_cfg 快照（不必重新 init）
       每拍调用一次即可；传负值 = 该项保持不变。
       ★ 只收这三项，是因为序列器内部是从 m_cfg 快照读它们的：
           fLeadTimeMs       —— 位置超前（带 0.05ms/拍限速，防止指令阶跃）
           fLeadAccMs2       —— 位置二阶超前
           fVelFfAccGainMs   —— 速度前馈加速度项 Ka（方案 B）
         fVelFfGain / fMaxFollowErrMm 由挂载层直接读 g_cfgFeed，不进快照，不用刷。
       ★ 凸轮几何参数与 fStartPhaseComp 只能走 init，这里不碰。 */
    void setLiveTuning(double fLeadTimeMs, double fLeadAccMs2, double fVelFfAccGainMs);

    /* 每周期（125us）：推进状态机，输出 4 轴位置指令 mm
       fMasterPhase：主轴相位 mm。**无界累加值或 [0,周长) 的编码器值都可以**，
                     内部按差分解缠成单调相位（跨圈/反向都不影响对齐与凸轮）
       afActPosMm：4 轴实际位置反馈（用于跟随偏差监视），传 0 = 不监视 */
    void tick(double fMasterPhase, double fDtSec, const double* afActPosMm,
              double afCmdPosMm[4]);

    void getMonitor(FEED_MONITOR_T* pOut) const;

private:
    /* 补偿钩子：v1 恒 0；后续 DOB/AI 补偿在此叠加到位置指令，不动状态机 */
    double compensationHook(int nAxisIdx) const;

    CCamFeeder    m_aFeeder[4];     /* 4 轴凸轮 */
    FEED_CONFIG_T m_cfg;            /* 参数快照（init 时拷贝） */
    int           m_nAxisCount;     /* 启用轴数（init 快照，1~4） */
    double        m_fMaxCamLen;     /* 启用轴中最大凸轮总长 mm（CAM_RUN 结束判据） */
    double        m_afPosBase[4];   /* 各轴工艺位移基准 mm：每次循环结束累加（送纸辊单向连续，指令不跳变） */
    FEED_MONITOR_T m_mon;           /* 监视量 */
    FEED_STATE_E  m_eState;         /* 当前状态 */
    bool          m_bStart;         /* 启动信号 */
    double        m_fTriggerPhase;  /* 锁定的启动相位 mm（整圈对齐+补偿） */
    double        m_fCamOrigin;     /* 凸轮相位原点 mm（= m_fTriggerPhase） */
    int           m_nStopTicks;     /* 停止保持计数 */
    double        m_fPrevPhase;     /* 上一拍外部主轴相位 mm（差分求速度 + 回绕修正） */
    bool          m_bPrevValid;     /* 上一拍相位有效标志（init 后首拍不差分） */
    double        m_fPhaseUnwrapped;/* 解缠后的内部主轴相位 mm（单调累加，下游统一用它） */
};

#endif /* FEED_SEQUENCER_H__20261117 */
