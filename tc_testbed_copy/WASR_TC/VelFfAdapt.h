#ifndef VEL_FF_ADAPT_H__20261126
#define VEL_FF_ADAPT_H__20261126

///////////////////////////////////////////////////////////////////////////////
// VelFfAdapt.h
// @brief  LMS 自动学「速度前馈增益 fVelFfGain」—— 换平台/换驱动器后不用再手扫增益。
//
// 原理（设计文档「前馈自适应方案设计.md」第二部分，已实测验证 R2=0.9997）：
//   同步段（匀速段）稳态误差满足一元一次方程：
//       FollowErr = (1 - a*g) * v / Kv
//   其中 g = 我们设的增益(已知)、v = 同步段速度(已知)、e = 实测误差，
//        a = 「前馈有效系数」= 驱动器实际兑现前馈的比例（唯一未知，慢变标量）。
//   所以每个凸轮周期拿同步段误差均值反解一次 a（LMS），输出 g = 1/a 即可。
//
// 为什么只学 fVelFfGain、不学 fTorFfGain：
//   速度前馈的判据是「同步段稳态滞后」—— 干净、周期平均后信噪比高；
//   转矩前馈的判据是「进同步的瞬态过冲」—— 噪声大、一改凸轮曲线判据就失效，
//   且撞 60B2 限幅后梯度方向会反。那个保持手动，正好当「拧回来」的旋钮。
//
// 耦合关系（低耦合是本类的设计目标）：
//   - 输入：每拍三个 mm 系监视量（误差/速度前馈/加速度）+ 圈数 + 是否在跑凸轮
//     —— 全部来自 FEED_MONITOR_T 与 CFeedSequencer::getDoneCycles()，本类不依赖
//     它们的具体实现，喂什么学什么（可离线用 CSV 回放任一历史数据验证）。
//   - 输出：一个增益值。谁把它写进 g_cfgFeed.fVelFfGain 由挂载层（WASR_TC）决定，
//     本类不碰全局、不碰 PDO、不碰 Status。
//
// 三条保命红线（设计文档第 7 节，不满足就会「学飞」）：
//   1) 限幅：增益夹在 [fGainMin, fGainMax]，theta 夹在对应倒数区间；
//   2) 泄漏：theta 每次更新乘 fLeak(0.999)，噪声下不无界漂移；
//   3) 冻结：连续 nFreezeCycles 圈增益步长 < fFreezeDelta 就停止更新（只查表执行），
//      收敛后不再追噪声。重新学习 = 由外部调 reset()（如开关重新置位）。
///////////////////////////////////////////////////////////////////////////////

/* LMS 学习参数（init 时可整体覆盖；构造时先填好一组实测过的默认值） */
typedef struct tagVELFF_ADAPT_CFG_T {
    int    nLearnAxis;          /* 用哪根轴的误差学（默认 0：轴0 同步段最长，信噪比最好） */
    double fKv;                 /* 位置环等效增益 [1/s]：把误差 mm 折算成欠供速度 mm/s
                                   机台实测 47.8 / 台架实测 52.4，取 47.8。
                                   ★ 影响的是 theta 的绝对值，增益收敛方向不受它影响，
                                     差 10% 只让收敛点多偏约 1%，不是敏感参数 */
    double fMu;                 /* LMS 学习步长（0.05~0.2，默认 0.1）
                                   0.1 时约 10~20 圈收敛（对拖 0.24s/圈 ≈ 3~5s） */
    double fLeak;               /* 泄漏系数（默认 0.999）：theta 每圈乘它，防噪声漂移 */
    double fGainMin;            /* 输出增益下限（默认 0.5） */
    double fGainMax;            /* 输出增益上限（默认 1.5） */
    double fMaxStepPerCycle;    /* 每圈增益最大步长（默认 0.05）：一圈野数据最多挪 5% */
    /* ---- 同步段检测（只在同步段学，加减速/停等段一律冻结不学） ---- */
    double fSyncVelMinMmS;      /* 速度下限 mm/s（默认 1000 ≈ 60m/min，排除停等段） */
    double fSyncAccMaxMmS2;     /* |加速度|上限 mm/s^2（默认 5000，排除加减速段；
                                   290m/min 加速段约 39000，裕量充足） */
    int    nSyncSkipSamples;    /* 进同步后跳过的拍数（默认 60 @0.5ms = 30ms 镇定时间；
                                   过冲回落段误差还是瞬态的，不能当稳态样本） */
    int    nMinSyncSamples;     /* 一圈同步段最少有效拍数，不足则该圈不学（默认 30） */
    double fErrMaxMm;           /* 单拍误差上限 mm（默认 50）：超出视为跳闸前兆/野点，不计入 */
    /* ---- 收敛冻结 ---- */
    int    nFreezeCycles;       /* 连续 N 圈步长小于阈值即冻结（默认 3） */
    double fFreezeDelta;        /* 冻结阈值（默认 0.002 = 增益变化 <0.2%/圈）
                                   ★ 别填 0.02：归一化 LMS 每圈步长只有 ~0.6% 量级，
                                     0.02 会在第 3 圈就误判"收敛"，实测会冻在半路
                                     （离线仿真验证：1.0 种子 14 圈到 1.046、
                                      0.8 种子 30 圈到 1.045、1.2 种子 19 圈到 1.068） */
} VELFF_ADAPT_CFG_T;

/* 学习状态（回显用） */
typedef enum tagVELFF_ADAPT_STATE_E {
    VELFF_ADAPT_OFF = 0,        /* 未使能（手调模式） */
    VELFF_ADAPT_LEARNING = 1,   /* 学习中（每圈更新一次增益） */
    VELFF_ADAPT_FROZEN = 2      /* 已收敛冻结（停止更新，保持当前增益执行） */
} VELFF_ADAPT_STATE_E;

class CVelFfAdapt {
public:
    CVelFfAdapt();

    /* 覆盖学习参数；传 NULL = 保持构造时的默认值。
       只改参数、不动学习状态；要换学习起点请用 reset() */
    void init(const VELFF_ADAPT_CFG_T* pCfg);

    /* 重置学习状态并以 fGainSeed 为起点（theta = 1/fGainSeed，输出 = fGainSeed）。
       reset 即视为"开始一次新的学习"，会同时把使能打开（m_bEnable = true），
       状态 = LEARNING。典型调用时机：使能开关上升沿 */
    void reset(double fGainSeed);

    /* 使能开关：false 时 tick() 直接返回（输出保持 reset 时的值，状态 = OFF） */
    void setEnable(bool bEnable);

    /* 每拍喂数据（mm 系）。圈末检测：nDoneCycles 比上一拍多 1 即做一次 LMS 更新。
       bCamRun=false（停等/未使能/冻结）时不累积样本。
       nDoneCycles 变小（重新启动）时自动清本圈累计，不误触发一次假更新 */
    void tick(bool bCamRun, int nDoneCycles,
              double fErrMm, double fVelMmS, double fAccMmS2);

    /* 当前输出增益（LEARNING/FROZEN 期间有效；OFF 时保持 reset 的种子值） */
    double getGain() const { return m_fGainOut; }

    /* 状态回显（0/1/2 见 VELFF_ADAPT_STATE_E） */
    int getState() const { return (int)m_eState; }

    /* 配置里的学习轴（挂载层拿它决定喂哪根轴的数据） */
    int getLearnAxis() const { return m_cfg.nLearnAxis; }

    /* 位置环等效增益（转矩 LMS 的残差扣除要用：fVelPart = (1−θ̂·g)·v/Kv） */
    double getKv() const { return m_cfg.fKv; }

    /* 诊断回显（可选，调试时看收敛过程） */
    double getTheta() const { return m_fTheta; }            /* 当前估计的有效系数 a */
    int    getCyclesLearned() const { return m_nCyclesLearned; } /* 已更新圈数 */
    double getLastResidMmS() const { return m_fLastResidMmS; }   /* 上圈 LMS 残差 mm/s */

private:
    /* 一圈结束时的 LMS 更新（样本不足/样本全被剔时不更新、不计圈） */
    void updateCycle();

    /* 清本圈同步段累计（不动学习参数与状态） */
    void clearCycleAccum();

    static double clampD(double v, double lo, double hi);

    VELFF_ADAPT_CFG_T   m_cfg;              /* 学习参数快照 */
    bool                m_bEnable;          /* 使能 */
    VELFF_ADAPT_STATE_E m_eState;           /* 学习状态 */
    int                 m_nPrevCycles;      /* 上一拍圈数（圈末检测；reset 后 = -1 表示待对齐） */
    /* ---- 本圈同步段累计 ---- */
    double              m_fSumErrMm;        /* 误差累计 mm */
    double              m_fSumVelMmS;       /* 速度累计 mm/s */
    int                 m_nSyncSamples;     /* 有效样本拍数 */
    int                 m_nSyncSeen;        /* 连续处于同步窗口的拍数（用于跳过镇定段） */
    /* ---- 学习状态量 ---- */
    double              m_fTheta;           /* 待估参数 a（前馈有效系数） */
    double              m_fGainOut;         /* 当前输出增益 = 1/theta（限幅后） */
    double              m_fLastResidMmS;    /* 上圈 LMS 残差 mm/s（诊断） */
    int                 m_nFreezeCnt;       /* 连续小步长圈数 */
    int                 m_nCyclesLearned;   /* 累计完成更新的圈数 */
};

#endif /* VEL_FF_ADAPT_H__20261126 */
