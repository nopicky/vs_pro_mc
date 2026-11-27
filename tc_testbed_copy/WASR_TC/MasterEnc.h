#ifndef MASTER_ENC_H__20261126
#define MASTER_ENC_H__20261126

///////////////////////////////////////////////////////////////////////////////
// MasterEnc.h
// @brief  真实主轴编码器 → 凸轮周期相位（方案见《主轴编码器方案大纲.md》§4）。
//
// 做什么：
//   每拍喂一次 6064h（int32 累积位置），输出 [0, fCycleLenMm) 的周期相位 mm。
//   主轴 VD5E 电机自带 23bit 编码器（8388608 counts/转），
//   经 22:27 齿比后电机 1 转 = 237.75mm 线长，5 转 = 1 个凸轮周期（1188.78mm）。
//
// 为什么用"累计增量 + 自己绕"，而不是"单圈取模"（raw % countsPerRev）：
//   1 个凸轮周期 = 5 编码器转，按单圈取模会让相位每 237.75mm 绕回一次，
//   凸轮一圈被启停 5 次。累计增量写法不依赖"比值是整数"，
//   以后换测量轮/齿比只改两个常数。
//
// 鲁棒性（三条，全部来自现场坑）：
//   1) int32 回绕：6064 是 int32，256 转（≈45.7s @400m/min）就跨一次 2^31。
//      差分用"无符号减再转回有符号"，跨回绕时增量依然正确 —— 主轴永不清零。
//   2) 丢帧/异常：单拍增量超过 fMaxDeltaCounts（默认 1 整转，正常工况下
//      单拍只有 ~3 万 counts）判定为坏样本，该拍冻结相位并计数，不污染凸轮。
//   3) 首拍对齐：第一拍只记录基准，不产生虚假速度/相位跳变。
//
// 低耦合：纯数学，不依赖 TwinCAT/PDO/全局变量。
//   输入只有 6064 与 dt，输出相位与诊断量 —— 可用历史 CSV 离线回归测试。
///////////////////////////////////////////////////////////////////////////////

/* 主轴编码器换算参数（init 时可整体覆盖；构造时先填好默认值） */
typedef struct tagMASTER_ENC_CFG_T {
    double fCountsPerRev;    /* 编码器每转计数（VD5E 23bit = 8388608） */
    double fMmPerRev;        /* 电机每转对应的主轴线长 mm（22:27 → 237.75） */
    double fCycleLenMm;      /* 凸轮周期对应的主轴行程 mm（1188.78 = fMasterCircum） */
    long   fMaxDeltaCounts;  /* 单拍增量上限 counts（丢帧剔除阈值，默认 1 整转） */
} MASTER_ENC_CFG_T;

class CMasterEnc {
public:
    CMasterEnc();

    /* 覆盖换算参数；传 NULL = 保持构造时的默认值 */
    void init(const MASTER_ENC_CFG_T* pCfg);

    /* 每拍喂 6064，返回 [0, fCycleLenMm) 的周期相位 mm。
       fDtSec 只用于速度回显，相位与 dt 无关（增量制） */
    double tick(long nPos6064, double fDtSec);

    /* 状态回显（不参与控制） */
    double getPhaseMm()    const { return m_fPhaseMm; }
    double getVelMmS()     const { return m_fVelMmS; }
    long   getPos6064()    const { return m_nPrevPos; }
    unsigned long getBadSamples() const { return m_nBadSamples; }

private:
    MASTER_ENC_CFG_T m_cfg;          /* 换算参数快照 */
    bool             m_bValid;       /* 首拍基准已对齐 */
    long             m_nPrevPos;     /* 上一拍 6064 */
    double           m_fPhaseMm;     /* 周期相位 [0, fCycleLenMm) */
    double           m_fVelMmS;      /* 差分速度（显示用，未滤波） */
    unsigned long    m_nBadSamples;  /* 被剔除的异常拍计数 */
};

#endif /* MASTER_ENC_H__20261126 */
