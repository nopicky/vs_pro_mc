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
    double afCmdPosMm[4];   /* 4 轴位置指令 mm */
    double afFollowErr[4];  /* 4 轴跟随偏差 mm（指令-实际，对齐 FAE 习惯扣除启动相位补偿） */
} FEED_MONITOR_T;

/* 送纸工艺状态机：只依赖 CVirtualMaster 相位输入与 CCamFeeder，不碰 PDO */
class CFeedSequencer {
public:
    CFeedSequencer();

    /* 按配置构建 4 个凸轮并做全参数校验，任一非法返回 false（禁止使能）
       配置做快照，运行期改 g_cfgFeed 不影响已 init 的实例 */
    bool init(const FEED_CONFIG_T* pCfg);

    /* 上位机启动信号（ADS/全局变量轮询写入） */
    void setStart(bool bStart);

    /* 每周期（125us）：推进状态机，输出 4 轴位置指令 mm
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
};

#endif /* FEED_SEQUENCER_H__20261117 */
