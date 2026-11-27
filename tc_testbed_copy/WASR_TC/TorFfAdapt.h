#ifndef TOR_FF_ADAPT_H__20261127
#define TOR_FF_ADAPT_H__20261127

///////////////////////////////////////////////////////////////////////////////
// TorFfAdapt.h
// @brief  LMS 自动学「转矩前馈增益 fTorFfGain」—— 方案 A：分时学习 + 残差扣除法
//         （设计文档《前馈自适应方案设计.md》§17）。
//
// 原理：
//   速度前馈收敛后（θ̂ = 前馈有效系数 a 已知），加速段的跟随误差可以拆成两份：
//       e(t) = (1 − a·g)·v(t)/Kv  +  (1 − b·TF)·α(t)·Kacc
//              ── 速度项 ──            ── 加速度项 ──
//   第一项是"速度前馈没兑现的部分"（已由速度 LMS 收敛时学到），从实测误差里扣掉，
//   剩下的残差 e_res 就是"转矩前馈没补够的力"造成的滞后。
//   对 e_res = (1 − b·TF)·φ（φ = α·Kacc）做标量 LMS，收敛后 TF = 1/b。
//
// 为什么收敛点不怕 Kacc 标错：
//   残差为零时 Kacc 被约掉（θ̂·TF·φ = φ 与 Kacc 无关），Kacc 只影响收敛路径
//   和步长，不影响终点。标定值来自机台实测（g=1.0/TF=0：加速完成滞后 12.77mm，
//   速度项 ~4.65mm，α≈39000mm/s² → Kacc ≈ 2.0e-4 s²）。
//
// 为什么判据不能用过冲峰值/加速段均值（§17 结论）：
//   过冲是瞬态峰值，噪声大、不可微；段均值把减速段符号相反的量平均没了。
//   残差扣除法用全部加减速段样本，φ 带符号（加速+、减速−），φ·e_res 同号累加，
//   信噪比和激励覆盖都是最好的。
//
// 三条保命红线（与 VelFfAdapt 同款）：
//   1) 限幅：TF 夹在 [fGainMin, fGainMax]（默认 0.2~1.2）；
//   2) 泄漏：θ̂ 每圈乘 fLeak(0.999)；
//   3) 冻结：连续 nFreezeCycles 圈步长 < fFreezeDelta 停止更新。
//   ★ 第 4 条特有红线：本圈 60B2 顶过限幅（±1000）→ 整圈不学。
//     顶限幅时误差不再随 TF 变化，任何学习律都会误以为"TF 没用"而无限上调。
//
// 前置条件（由调用方 WASR_TC 把门）：
//   速度 LMS 已收敛（VelFfAdaptState=2）或速度增益已手动定好（bAutoVelFfGain=0）。
//   学习中的速度增益会让残差扣除失准——θ̂ 在变，扣出来的残差是脏的。
//
// 低耦合：纯数学，不依赖 TwinCAT/PDO/全局变量。θ̂/g/Kv 等速度侧参数由调用方
//   每拍换算成 fVelPartMm 传进来，本模块不认识 VelFfAdapt。
///////////////////////////////////////////////////////////////////////////////

/* 转矩前馈学习参数（init 时可整体覆盖；构造时先填好一组实测过的默认值） */
typedef struct tagTORFF_ADAPT_CFG_T {
    int    nLearnAxis;          /* 用哪根轴（默认 0，与速度 LMS 同轴，残差扣除才对得上） */
    double fMu;                 /* 学习步长（默认 0.1） */
    double fLeak;               /* 泄漏系数（默认 0.999） */
    double fGainMin;            /* TF 下限（默认 0.2：再小说明转矩前馈在这工况帮倒忙） */
    double fGainMax;            /* TF 上限（默认 1.2：实测 1.0 就撞限幅，1.2 已留诊断余量） */
    double fMaxStepPerCycle;    /* 每圈最大步长（默认 0.05） */
    double fKacc;               /* 加速度→位置滞后折算常数 [s^2]（机台标定 2.0e-4，
                                   换平台要重标；★ 只影响收敛路径，不影响终点） */
    /* ---- 加减速段检测（只在 |α| 够大的段学，同步/停等段不学） ---- */
    double fAccMinMmS2;         /* |α| 下限（默认 10000；290m/min 加减速约 ±39000） */
    int    nAccSkipSamples;     /* 进加减速段后跳过的拍数（默认 20：跳过段首的过渡） */
    int    nMinAccSamples;      /* 一圈加减速段最少有效拍数（默认 40） */
    double fErrMaxMm;           /* 残差上限 mm（默认 50，超出视为野点） */
    /* ---- 收敛冻结 ---- */
    int    nFreezeCycles;       /* 默认 3 */
    double fFreezeDelta;        /* 默认 0.002（与速度 LMS 同口径，别填 0.02） */
} TORFF_ADAPT_CFG_T;

/* 学习状态（与 VelFfAdapt 同编码） */
typedef enum tagTORFF_ADAPT_STATE_E {
    TORFF_ADAPT_OFF = 0,        /* 未使能（手调模式） */
    TORFF_ADAPT_LEARNING = 1,
    TORFF_ADAPT_FROZEN = 2      /* 已收敛冻结 */
} TORFF_ADAPT_STATE_E;

class CTorFfAdapt {
public:
    CTorFfAdapt();

    /* 覆盖学习参数；传 NULL = 保持构造时的默认值 */
    void init(const TORFF_ADAPT_CFG_T* pCfg);

    /* 重置学习并以 fGainSeed 为起点；reset 即"开始一次新学习"，同时打开使能 */
    void reset(double fGainSeed);

    /* 使能开关：false 时 tick() 直接返回，状态归 OFF */
    void setEnable(bool bEnable);

    /* 每拍喂数据：
       bCamRun    凸轮运行中
       nDoneCycles 完成圈数（圈末检测用）
       fErrMm     原始跟随偏差（mm，符号同 FollowErr：正=实际落后）
       fVelPartMm 速度项该占的部分 = (1−θ̂·g)·v/Kv（调用方按速度 LMS 当前参数算，
                  本模块不认识 VelFfAdapt——换实现不用改这里）
       fAccMmS2   凸轮运动学加速度（mm/s²，带符号：加速+、减速−）
       bSaturated 本拍 60B2 已顶限幅（本圈顶过一次 → 整圈不学，特有红线 4） */
    void tick(bool bCamRun, int nDoneCycles,
              double fErrMm, double fVelPartMm, double fAccMmS2, bool bSaturated);

    double getGain() const { return m_fGainOut; }
    int    getState() const { return (int)m_eState; }
    int    getLearnAxis() const { return m_cfg.nLearnAxis; }
    double getTheta() const { return m_fTheta; }              /* 当前估计的 b */
    int    getCyclesLearned() const { return m_nCyclesLearned; }
    double getLastResidMm() const { return m_fLastResidMm; }  /* 上圈残差（诊断） */

private:
    void updateCycle();
    void clearCycleAccum();
    static double clampD(double v, double lo, double hi);

    TORFF_ADAPT_CFG_T   m_cfg;
    bool                m_bEnable;
    TORFF_ADAPT_STATE_E m_eState;
    int                 m_nPrevCycles;
    /* ---- 本圈加减速段累计（批式 LMS：Σφ² 与 Σφ·e_res） ---- */
    double              m_fSumPhi2;         /* Σφ² */
    double              m_fSumPhiE;         /* Σφ·e_res */
    int                 m_nAccSamples;
    int                 m_nAccSeen;         /* 连续处于加减速窗口的拍数（跳段首过渡用） */
    bool                m_bCycleSaturated;  /* 本圈 60B2 顶过限幅（整圈作废） */
    /* ---- 学习状态量 ---- */
    double              m_fTheta;           /* 待估参数 b（转矩前馈有效系数） */
    double              m_fGainOut;         /* 当前输出 TF = 1/θ̂（限幅后） */
    double              m_fLastResidMm;     /* 上圈 LMS 残差（诊断） */
    int                 m_nFreezeCnt;
    int                 m_nCyclesLearned;
};

#endif /* TOR_FF_ADAPT_H__20261127 */
