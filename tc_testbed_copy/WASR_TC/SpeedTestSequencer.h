///////////////////////////////////////////////////////////////////////////////
// SpeedTestSequencer.h
// @brief  实验序列器: 按 ExperimentType_T 生成速度指令 / 转矩指令。
//
// 实验 (激励参数见 TestScriptConfig.h, 频点表见 TestScanTables.h):
//   Manual            不启动实验, 速度指令由 PLC 直接给; SigOffset 是负载侧基准转矩。
//   StepResponse      SigOffset (中心) ± SigAmp 的阶跃, 时序 STEP_TEST_*。
//   WaveResponse      SigOffset (基础速度) + SigAmp × 波形, 频率 SigFreq;
//                     0 Hz 只给基础速度, 运行中可改。
//   VelocitySweep     SigOffset + SigAmp × 正弦, 频点 SweepFreqTable, SigFreq 回读当前频点。
//   TorqueSweep       上位机速度环稳住 SigOffset, SigAmp [0.1% 额定] 扫频正弦叠加在
//                     转矩输出上, SigFreq 回读当前频点。
//   LoadDisturbScan   驱动侧速度载波 (LOADSCAN_TEST_*) + 负载侧转矩
//                     SigOffset + 正弦 (频率/幅度见 LoadScanFreqTable 与 LOADSCAN_TEST_AMP_*),
//                     SigFreq / SigAmp 回读当前段。
//   PosLoopWave       伺服工作在 CSP(8): 目标位置 = SigOffset (原点) + SigAmp × 波形,
//                     频率 SigFreq; 全程指令单位, 0 Hz 冻结相位。
//   VelocityClosedLoopPRBS  速度闭环 PRBS: 先到 SigOffset (基础速度), 静置后
//                     叠加 PRBS (±SigAmp, 位速率 = SigFreq), 只输出激励序列,
//                     不对频响做任何在线计算 (离线用录波数据做 FFT)。
//   FeedCamTest       送纸凸轮: 伺服 CSP(8), 虚拟主轴相位驱动 4 条五次曲线凸轮,
//                     SigOffset = 主轴线速度 [m/min], SigFreq 回读凸轮相位 [mm],
//                     启停用 **bFeedStart** (1=送纸/续送, 0=停止), SigAmp 不用。
//                     轴号≠槽位号: 对拖时轴0 在 Servo1, 对应关系见 Status.AxisNSlot。
//
// 注入信号统一口径: SigFreq [Hz] / SigAmp / SigOffset, 单位随实验:
//   速度实验 (Step/Wave/Sweep/TorqueSweep): SigAmp [rpm], SigOffset [rpm]
//   转矩扫频:  SigAmp [0.1% 额定转矩], SigOffset 是基础速度 [rpm]
//   负载扫描:  SigOffset [0.1% 额定转矩] 是负载侧基准转矩, SigAmp 回读 [% 负载额定]
//   位置环波形: SigAmp [指令单位], SigOffset [指令单位] (原点)
//   PRBS:      SigAmp [rpm], SigOffset [rpm], SigFreq [Hz] 是 PRBS 位速率
//   送纸凸轮:  SigOffset [m/min] 线速度, 启停走独立位 bFeedStart
//   SigFreq 在扫频类实验里是回读 (序列器写), 其余实验是用户输入。
//
// 驱动侧模式 driveMode: 0 = CSV(9) 伺服自带速度环 (其余实验与 Manual),
//                       1 = CST(10) + 上位机速度环 (仅转矩扫频及其停机斜坡),
//                       2 = CSP(8) 位置指令 (位置环波形 / 送纸凸轮)。
// 停机: 实验结束或 Enable = 0 时按 TEST_RAMP_RATE 减速到 0 再禁用驱动器
//       (位置环波形为目标回基准位置, 保持 POSWAVE_TEST_HOLD_DUR 后禁用;
//        送纸凸轮为原地保持当前指令, 不回原点)。
//
// 用法 (每周期, 在 CiA402_Update 之前):
//   SpeedTestSequencer seq;
//   seq.Init(dtSec, 总惯量, 额定转矩, 转矩限幅);
//   seq.Update(m_Status.ExperimentType, &m_Status.Enable, &spdCmd,
//              spdFb, driveReady,
//              &m_Status.SigFreq, &m_Status.SigAmp, &m_Status.SigOffset,
//              &m_Status.bFeedStart, m_afActPosUnits);
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <math.h>

// 实验类型枚举 (ExperimentType_T)
#include "WASR_TCServices.h"
// 实验激励参数宏
#include "TestScriptConfig.h"
// 频点表
#include "TestScanTables.h"
// 转矩扫频用的上位机速度环
#include "MiniSpdLoop.h"
// 送纸凸轮实验 (FeedCamTest): 工艺状态机 / 虚拟主轴 / 负载谱
#include "FeedSequencer.h"
#include "VirtualMaster.h"
#include "LoadProfiler.h"
// 站点表: 送纸轴数上限 (SITE_AXIS_MAX)
#include "SiteConfig.h"

struct SpeedTestSequencer
{
    enum Mode {
        IDLE = 0,           // 空闲, 等待 Enable 上升沿
        SWEEP_TEST,         // 速度扫频
        TOR_SWEEP_TEST,     // 转矩扫频
        WAVE_TEST,          // 定频波形
        STEP_TEST,          // 阶跃
        LOAD_SCAN_TEST,     // 负载侧扰动转矩扫描
        POS_WAVE_TEST,      // 位置环定频波形 (CSP)
        PRBS_TEST,          // 速度闭环 PRBS + 增量频响
        FEED_CAM_TEST,      // 送纸凸轮: 虚拟主轴 + 4 轴五次曲线 (CSP)
        STOPPING,           // 减速到 0 / 幅值淡出, 准备禁用
        DONE                // 实验结束, 等待 Enable 释放
    };

    enum DriveMode {
        DRIVE_CSV = 0,      // 9  周期同步速度 (伺服自带速度环)
        DRIVE_CST = 1,      // 10 周期同步转矩 (上位机速度环)
        DRIVE_CSP = 2       // 8  周期同步位置 (本模块给目标位置)
    };

    enum SweepSub {
        SWEEP_ACTIVE = 0,   // 正弦输出中
        SWEEP_FINISHING,    // 完成当前整周期
        SWEEP_GAP,          // 频点间基础速度保持
        SWEEP_STANDBY,      // 启动: 0 rpm 保持
        SWEEP_RAMP_UP       // 启动: 匀加速到基础速度
    };

    enum WaveSub {
        WAVE_STANDBY = 0,   // 启动: 0 rpm 保持
        WAVE_RAMP_UP,       // 启动: 匀加速到基础速度
        WAVE_ACTIVE         // 基础速度 + 波形 (0 Hz 只给基础速度)
    };

    enum TorSweepSub {
        TOR_SWEEP_STANDBY = 0,  // 启动: 0 速保持 → 匀加速 → 基础速度保持
        TOR_SWEEP_MEASURE,      // 测量段 (仅一次): 测速度环平均转矩
        TOR_SWEEP_SETTLE,       // 静置段: 过渡到平均转矩
        TOR_SWEEP_ACTIVE,       // 震荡段: 平均转矩 + 扫频正弦
        TOR_SWEEP_FINISHING,    // 完成当前整周期
        TOR_SWEEP_RECOVER       // 恢复段: 交还速度环输出
    };

    enum LoadScanSub {
        LOAD_SCAN_STANDBY = 0,  // 启动: 0 速保持
        LOAD_SCAN_RAMP,         // 启动: 匀加速到载波中心速度
        LOAD_SCAN_GAP,          // 间隔段: 仅载波, 不注入扰动
        LOAD_SCAN_ACTIVE        // 当前 (频率, 幅度) 段: 载波 + 扰动
    };

    enum PosWaveSub {
        POS_WAVE_STANDBY = 0,   // 启动: 先只给原点并保持
        POS_WAVE_ACTIVE         // 原点 + 波形 (0 Hz 冻结相位)
    };

    enum PrbsSub {
        PRBS_STANDBY = 0,       // 启动: 0 rpm 保持
        PRBS_RAMP_UP,           // 启动: 匀加速到基础速度
        PRBS_SETTLE,            // 基础速度保持 (等速度环进稳态)
        PRBS_ACTIVE             // 基础速度 + PRBS (±幅值)
    };

    // Init 写入
    float dtSec;                  // 控制周期 [s]
    float torLim;                 // 转矩限幅 [0.1% 额定]

    // 公共状态
    Mode    mode;
    uint8_t prevEnable;
    float   elapsed;         // 当前相位计时 [s]

    // 速度扫频
    SweepSub sweepSub;
    uint8_t  sweepFreqIdx;          // SweepFreqTable 下标
    float    sweepSegElapsed;       // 当前频点计时 [s]
    int      sweepPrevCycle;        // 上一整周期序号
    uint16_t sweepCompletedCycles;  // 当前频点已完成整周期数
    float    sweepSubElapsed;       // GAP / FINISHING 计时 [s]

    // 阶跃
    uint8_t  stepPhase;             // 0=0 rpm, 1=base+Amp, 2=base-Amp

    // 定频波形
    WaveSub  waveSub;
    uint8_t  waveType;              // 1=正弦 2=三角 3=梯形 (宏固定)
    float    waveSubElapsed;        // 当前子相位计时 [s]
    float    wavePhase;             // 波形相位累积 [rad]

    // 位置环波形 (CSP)
    PosWaveSub posWaveSub;
    uint8_t    posWaveType;         // 1=正弦 2=三角 3=梯形 (宏固定)
    float      posWaveSubElapsed;   // 启动保持计时 [s]
    float      posWavePhase;        // 波形相位累积 [rad]
    // 已完成的完整周期数 (相位每回绕一次加 1, 用于 POSWAVE_TEST_CYCLES 自动结束)
    uint32_t   posWaveCycles;
    // 波形中心 [指令单位]: 宏给定的绝对位置, 运行中不变
    LONG       posCenter;
    // 停机保持计时 [s] (目标回基准位置后计时, 到 POSWAVE_TEST_HOLD_DUR 禁用)
    float      posStopElapsed;
    // 本拍各轴目标位置 [指令单位] (driveMode = DRIVE_CSP 时写入各轴 607Ah)
    // 用 double 累加: 绝对位置可达 2^31, float 尾数只有 24 位, 大位置下会丢小增量
    // [0] 同时供位置环波形等单轴实验使用; 送纸凸轮用到 [0..3]
    double     posCmdOut[SITE_AXIS_MAX];

    // 转矩扫频
    TorSweepSub torSweepSub;
    uint8_t     torSweepStartPhase;     // 启动段子相位: 0=0 速, 1=匀加速, 2=基础速度保持
    uint8_t     torSweepFreqIdx;        // TorSweepFreqTable 下标
    float       torSweepSegElapsed;     // 当前频点计时 [s]
    int         torSweepPrevCycle;      // 上一整周期序号
    uint16_t    torSweepCompletedCycles;// 当前频点已完成整周期数
    float       torSweepSubElapsed;     // 各子相位计时 [s]
    float       torSweepBlend;          // 速度环输出权重 [-] (1=纯速度环, 0=纯平均转矩)
    float       torSweepSine;           // 本拍扫频正弦 [0.1% 额定转矩]
    float       torSweepBaseTorque;     // 测量段平均转矩 [0.1% 额定转矩]
    float       torSweepMeasAccum;      // 测量段累加值 [0.1% 额定转矩]
    uint32_t    torSweepMeasCount;      // 测量段采样数

    // 负载侧扰动扫描
    uint16_t    loadScanFreqIdx;        // LoadScanFreqTable 下标
    uint8_t     loadScanAmpIdx;         // 幅度步序号
    uint8_t     loadScanAmpSteps;       // 幅度步数 (LOADSCAN_TEST_AMP_STEPS)
    uint8_t     loadScanStarted;        // 1 = 起始驻留已结束
    uint8_t     loadScanGapNext;        // GAP 后去向: 0=同频下一幅度, 1=下一频率
    LoadScanSub loadScanSub;
    float       loadScanSubElapsed;     // GAP / ACTIVE 计时 [s]
    float       loadScanSegElapsed;     // 当前段计时 [s]
    int         loadScanPrevCycle;      // 上一整周期序号
    uint16_t    loadScanCycles;         // 当前段已完成整周期数
    float       driveWavePhase;         // 载波相位 [rad]
    float       distPhase;              // 扰动相位 [rad] (每段从 0 起)
    float       loadTorInj;             // 负载侧注入转矩 [0.1% 额定], 其余实验为 0

    // 速度闭环 PRBS
    PrbsSub  prbsSub;
    float    prbsSubElapsed;        // 启动/静置计时 [s]
    float    prbsSegElapsed;        // PRBS 段计时 [s]
    uint16_t prbsReg;               // LFSR 状态 (0 会卡死, 用全 1 初始化)
    float    prbsAcc;               // 当前 bit 已计时 [s]
    float    prbsBit;               // 当前 bit 输出 [-1, +1]

    // 送纸凸轮实验 (FEED_CAM_TEST): 虚拟主轴 + 凸轮状态机
    // 轴数由站点表 (Status.SiteProfile) 决定: 1~4 个驱动轴各跑一条凸轮,
    // 每轴位置指令进 posCmdOut[0..3], 由挂载层按槽位下发
    CFeedSequencer feedSeq;         // 送纸工艺状态机 (含 4 轴凸轮)
    CVirtualMaster feedMaster;      // 虚拟主轴
    CLoadProfiler  feedLoad;        // 负载谱 (v1 占位, bLoadSideEnable=false 不用)
    // mm→指令单位 换算系数, 每轴一份 (轴电机不同则不同, 0 = 未挂载)
    double feedMmToUnit[SITE_AXIS_MAX];
    // 进实验时各轴 6064h 实际位置 [指令单位] (指令原点, 防跳变)
    double feedActPos0[SITE_AXIS_MAX];
    double feedCmdMm[SITE_AXIS_MAX];// 本拍各轴位置指令 [mm]

    // 停机
    float    lastSpdCmd;            // 上一拍速度指令 [rpm]
    uint8_t  stopClearFreq;         // 1 = 停机完成时把 SigFreq 清 0
    uint8_t  stopFromFeedCam;       // 1 = 本次停机来自送纸凸轮 (保持指令位置, 不回原点)

    // 上位机速度环 (仅 driveMode = DRIVE_CST 时运行)
    MiniSpdLoop spdLoop;
    // 驱动侧模式: DRIVE_CSV(9) / DRIVE_CST(10) / DRIVE_CSP(8)
    uint8_t  driveMode;
    // 本拍驱动侧转矩指令 [0.1% 额定转矩] (已限幅, driveMode = DRIVE_CST 时写入 6071h)
    float    torCmdOut;

    /**
     * 初始化
     * @param dt          控制周期 [s]
     * @param inertia     总惯量 (转子惯量 × (1 + 惯量比)) [kg·m^2]
     * @param ratedTorque 电机额定转矩 [N·m]
     * @param lim         转矩限幅 [0.1% 额定转矩]
     */
    void Init(float dt, float inertia, float ratedTorque, float lim);

    /**
     * 写速度指令并记录 (停机斜坡从当前指令开始)
     * @param spdCmd 速度指令指针
     * @param v      速度指令 [rpm]
     */
    void SetSpdCmd(float* spdCmd, float v);

    /**
     * 复位上位机速度环
     */
    void ResetSpdLoop();

    /**
     * 本拍驱动侧转矩指令
     * @return torCmdOut [0.1% 额定转矩]
     */
    float GetTorCmd() const;

    /**
     * 每周期更新 (在 CiA402_Update 之前调用)
     * @param experiment  m_Status.ExperimentType [in]
     * @param enable      m_Status.Enable [in/out]
     * @param spdCmd      速度指令 [in/out]; 转矩扫频中它是速度环参考速度
     * @param spdFb       驱动侧速度反馈 [rpm] (606Ch), 供上位机速度环
     * @param driveReady  驱动侧已 Operation Enabled = 1 (0 = 速度环不积分)
     * @param sigFreq     m_Status.SigFreq [in/out] [Hz]: 波形/PRBS 实验为输入; 扫频/
     *                    转矩扫频/负载扫描回读当前频率 (间隔段 0)
     * @param sigAmp      m_Status.SigAmp [in/out]: 幅值 (单位随实验, 见文件头);
     *                    负载扫描回读当前幅度 [% 负载额定]; 位置环波形超
     *                    POSWAVE_TEST_AMP_MAX 时夹紧并回读
     * @param sigOffset   m_Status.SigOffset [in/out]: 偏置 (单位随实验, 见文件头)
     * @param feedStart   m_Status.bFeedStart [in]: 送纸启动位 (1=送纸/续送, 0=停止)。
     *                    只在送纸凸轮实验里用; 原来是借 SigAmp 的上升沿当触发, 已废弃
     * @param afActPosUnits 各驱动轴 6064h 实际位置 [指令单位], 长度 SITE_AXIS_MAX
     *                    (送纸凸轮: 指令原点/跟随监视; 其余实验只用 [0])
     */
    void Update(ExperimentType_T experiment,
                uint8_t* enable, float* spdCmd, float spdFb,
                uint8_t driveReady,
                float* sigFreq, float* sigAmp, float* sigOffset,
                uint8_t* feedStart,
                const double* afActPosUnits);

};
