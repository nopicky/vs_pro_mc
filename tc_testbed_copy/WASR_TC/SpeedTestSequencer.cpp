///////////////////////////////////////////////////////////////////////////////
// SpeedTestSequencer.cpp
// @brief  实验序列器实现 (声明见 SpeedTestSequencer.h)
///////////////////////////////////////////////////////////////////////////////

#include "TcPch.h"
#pragma hdrstop

#include "SpeedTestSequencer.h"

// 三角波: 输出 [-1, 1], 周期 2π
static float WaveTriangle(float phase)
{
    // 相位回绕到 [0, 2π)
    float p = fmodf(phase, 6.283185307f);
    if (p < 0.0f) {
        p += 6.283185307f;
    }
    const float halfPi = 1.570796327f;
    if (p < halfPi) {
        // 上升沿 0 -> +1
        return p / halfPi;
    }
    if (p < 3.0f * halfPi) {
        // 下降沿 +1 -> -1
        return 2.0f - p / halfPi;
    }
    // 上升沿 -1 -> 0
    return p / halfPi - 4.0f;
}

// 梯形波: 输出 [-1, 1], 周期 2π, 斜坡占 WAVE_TEST_TRAP_SLOPE 个周期
static float WaveTrapezoid(float phase)
{
    // 相位回绕到 [0, 2π)
    float p = fmodf(phase, 6.283185307f);
    if (p < 0.0f) {
        p += 6.283185307f;
    }
    const float period = 6.283185307f;
    // 单个斜坡时长 [rad]
    const float slope  = WAVE_TEST_TRAP_SLOPE * period;
    // 半周期 [rad]
    const float half   = period * 0.5f;
    if (p < slope) {
        // 上升沿 -1 -> +1
        return -1.0f + 2.0f * (p / slope);
    }
    if (p < half) {
        // 顶部平顶 +1
        return 1.0f;
    }
    if (p < half + slope) {
        // 下降沿 +1 -> -1
        return 1.0f - 2.0f * ((p - half) / slope);
    }
    // 底部平顶 -1
    return -1.0f;
}

// 位置波形梯形波的上升段比例 (宏 POSWAVE_TEST_TRAP_RISE), 夹到 [0, 1]
//   1 → 没有平顶, 退化为三角波; 0 → 上升段瞬时, 退化为阶跃
static float PosTrapRise(void)
{
    // 上升段占半周期的比例 [-]
    float r = POSWAVE_TEST_TRAP_RISE;
    if (r < 0.0f) {
        r = 0.0f;
    }
    if (r > 1.0f) {
        r = 1.0f;
    }
    return r;
}

// 梯形波 (位置波形): 输出 [-1, 1], 周期 2π, 上升段占半周期的比例为 rise
// @param phase 相位 [rad]
// @param rise  上升段占半周期的比例 [0, 1]
static float WaveTrapezoidRise(float phase, float rise)
{
    // 相位回绕到 [0, 2π)
    float p = fmodf(phase, 6.283185307f);
    if (p < 0.0f) {
        p += 6.283185307f;
    }
    // 半周期 [rad]
    const float half = 3.141592654f;
    // 上升段时长 [rad]
    const float ramp = rise * half;
    if (p < half) {
        // 前半周期: 上升段 -1 → +1, 之后平顶 +1 (ramp = 0 时直接 +1)
        if (ramp <= 0.0f) {
            return 1.0f;
        }
        if (p < ramp) {
            return -1.0f + 2.0f * (p / ramp);
        }
        return 1.0f;
    }
    // 后半周期: 下降段 +1 → -1, 之后平顶 -1 (ramp = 0 时直接 -1)
    if (ramp <= 0.0f) {
        return -1.0f;
    }
    if (p < half + ramp) {
        return 1.0f - 2.0f * ((p - half) / ramp);
    }
    return -1.0f;
}

// 波形值 [-1, 1]: 1=正弦 2=三角 3=梯形 (其它按正弦)
// 梯形用位置波形的上升段比例 POSWAVE_TEST_TRAP_RISE (见 TestScriptConfig.h)
static float WaveValue(uint8_t type, float phase)
{
    switch (type) {
    case 2:
        return WaveTriangle(phase);
    case 3:
        return WaveTrapezoidRise(phase, PosTrapRise());
    default:
        return sinf(phase);
    }
}

// PRBS 移位一拍 (Galois 型 LFSR), 并更新当前 bit 输出 ±1
// @param reg LFSR 状态 (必须非 0)
// @param bit 输出: 本拍 bit 值 [-1, +1]
static void PrbsShift(uint16_t* reg, float* bit)
{
    // 移出位 = 反馈
    uint16_t lsb = (uint16_t)(*reg & 1u);
    *reg = (uint16_t)(*reg >> 1);
    if (lsb != 0) {
        *reg = (uint16_t)(*reg ^ (uint16_t)PRBS_TEST_LFSR_MASK);
    }
    // 逻辑 0/1 映射到 -1/+1 (均值为 0, 不抬基础速度)
    *bit = (lsb != 0) ? -1.0f : 1.0f;
}

///////////////////////////////////////////////////////////////////////////////
// 初始化 (控制周期/机械参数/转矩限幅)
// @param dt          控制周期 [s]
// @param inertia     总惯量 (电机转子惯量, 已含惯量比) [kg·m^2]
// @param ratedTorque 电机额定转矩 [N·m]
// @param lim         转矩限幅 [0.1% 额定转矩]
///////////////////////////////////////////////////////////////////////////////
void SpeedTestSequencer::Init(float dt, float inertia, float ratedTorque, float lim)
{
    dtSec        = dt;
    torLim       = lim;
    mode         = IDLE;
    prevEnable   = 0;
    elapsed      = 0.0f;

    sweepSub             = SWEEP_ACTIVE;
    sweepFreqIdx         = 0;
    sweepSegElapsed      = 0.0f;
    sweepPrevCycle       = 0;
    sweepCompletedCycles = 0;
    sweepSubElapsed      = 0.0f;

    stepPhase = 0;

    waveSub        = WAVE_STANDBY;
    waveType       = WAVE_TEST_SIGNAL_TYPE;
    waveSubElapsed = 0.0f;
    wavePhase      = 0.0f;
    // SigFreq/SigAmp/SigOffset 初值由模块按 ExperimentType 写入, 这里不改

    posWaveSub        = POS_WAVE_STANDBY;
    posWaveType       = POSWAVE_TEST_SIGNAL_TYPE;
    posWaveSubElapsed = 0.0f;
    posWavePhase      = 0.0f;
    posWaveCycles     = 0;
    posCenter         = 0;
    posStopElapsed    = 0.0f;
    posCmdOut         = 0.0;

    // 送纸凸轮: 换算未挂载 (=0) 时位置指令保持指令原点, 电机不动 (安全)
    feedMmToUnit   = 0.0;
    feedActPos0    = 0.0;
    feedPrevSigAmp = 0.0f;
    for (int i = 0; i < 4; i++) {
        feedCmdMm[i] = 0.0;
    }

    torSweepSub             = TOR_SWEEP_STANDBY;
    torSweepStartPhase      = 0;
    torSweepFreqIdx         = 0;
    torSweepSegElapsed      = 0.0f;
    torSweepPrevCycle       = 0;
    torSweepCompletedCycles = 0;
    torSweepSubElapsed      = 0.0f;
    torSweepBlend           = 1.0f;
    torSweepSine            = 0.0f;
    torSweepBaseTorque      = 0.0f;
    torSweepMeasAccum       = 0.0f;
    torSweepMeasCount       = 0;

    loadScanFreqIdx     = 0;
    loadScanAmpIdx      = 0;
    loadScanAmpSteps    = 0;
    loadScanStarted     = 0;
    loadScanGapNext     = 0;
    loadScanSub         = LOAD_SCAN_STANDBY;
    loadScanSubElapsed  = 0.0f;
    loadScanSegElapsed  = 0.0f;
    loadScanPrevCycle   = 0;
    loadScanCycles      = 0;
    driveWavePhase      = 0.0f;
    distPhase           = 0.0f;
    loadTorInj          = 0.0f;

    prbsSub        = PRBS_STANDBY;
    prbsSubElapsed = 0.0f;
    prbsSegElapsed = 0.0f;
    // 全 1 初值: LFSR 全 0 会永久卡死
    prbsReg        = (uint16_t)((1u << PRBS_TEST_LFSR_BITS) - 1u);
    prbsAcc        = 0.0f;
    prbsBit        = 1.0f;

    lastSpdCmd    = 0.0f;
    stopClearFreq = 0;

    // 驱动侧默认 CSV, 只有转矩扫频置 CST, 位置环波形置 CSP
    driveMode = DRIVE_CSV;
    torCmdOut = 0.0f;

    // 转矩扫频速度环 (参数见 MiniSpdLoop.h)
    spdLoop.Init(dt, inertia, ratedTorque, torLim);
}

///////////////////////////////////////////////////////////////////////////////
// 写速度指令并记录 (停机斜坡从当前指令开始)
// @param spdCmd 速度指令指针
// @param v      速度指令 [rpm]
///////////////////////////////////////////////////////////////////////////////
void SpeedTestSequencer::SetSpdCmd(float* spdCmd, float v)
{
    *spdCmd    = v;
    lastSpdCmd = v;
}

///////////////////////////////////////////////////////////////////////////////
// 复位上位机速度环
///////////////////////////////////////////////////////////////////////////////
void SpeedTestSequencer::ResetSpdLoop()
{
    spdLoop.Reset();
}

///////////////////////////////////////////////////////////////////////////////
// 本拍驱动侧转矩指令
// @return 转矩指令 [0.1% 额定转矩]
///////////////////////////////////////////////////////////////////////////////
float SpeedTestSequencer::GetTorCmd() const
{
    return torCmdOut;
}

///////////////////////////////////////////////////////////////////////////////
// 每周期更新 (在 CiA402_Update 之前调用), 参数说明见 SpeedTestSequencer.h
///////////////////////////////////////////////////////////////////////////////
void SpeedTestSequencer::Update(ExperimentType_T experiment,
            uint8_t* enable, float* spdCmd, float spdFb,
            uint8_t driveReady,
            float* sigFreq, float* sigAmp, float* sigOffset,
            double actPosUnits)
{
    // 外部 Enable 快照 (下面可能改写 *enable)
    uint8_t extEnable = *enable;

    uint8_t rising = (prevEnable == 0 && extEnable != 0);
    prevEnable = extEnable;

    // 负载侧基准转矩 [0.1% 额定]: Manual / 负载扫描用 SigOffset, 其余实验为 0
    float loadBase = 0.0f;
    if (experiment == ExperimentType_Manual ||
        experiment == ExperimentType_LoadDisturbScan) {
        loadBase = (sigOffset != NULL) ? *sigOffset : 0.0f;
    }

    // 每拍默认值: 注入/转矩指令/扫频分量全部清零, 只有对应实验会写入
    torCmdOut     = 0.0f;
    torSweepSine  = 0.0f;
    torSweepBlend = 1.0f;
    // 位置指令默认停在原点: 位置实验之外不用它, 各位置分支会按需覆盖
    posCmdOut     = (double)posCenter;
    // 负载侧注入转矩默认只有基准值; 仅 LOAD_SCAN_TEST 按 (频率,幅度) 段再叠加
    loadTorInj    = loadBase;
    // 只有 LOAD_SCAN_TEST 把 SigAmp 当回读; 波形/位置实验中是用户输入, 不清零
    if (sigAmp != NULL && mode == LOAD_SCAN_TEST) {
        *sigAmp = 0.0f;
    }

    // Enable = 0: 不立即停机, 先按 TEST_RAMP_RATE 减速到 0 再禁用
    if (extEnable == 0 && mode != IDLE && mode != DONE && mode != STOPPING) {
        // 保持使能, 斜坡减速
        *enable = 1;
        // 位置环波形: 停机段幅值淡出计时从 0 起
        posStopElapsed = 0.0f;
        // 频率回读只在扫频/转矩扫频/负载扫描中清零 (波形实验是用户输入)
        stopClearFreq = (mode == SWEEP_TEST || mode == TOR_SWEEP_TEST ||
                         mode == LOAD_SCAN_TEST) ? 1 : 0;
        if (stopClearFreq && sigFreq != NULL) {
            *sigFreq = 0;
        }
        mode = STOPPING;
        // 落入下面的 STOPPING, 指令保持连续
    }

    // Enable 上升沿: 按 ExperimentType 进入实验 (STOPPING 中忽略, 先停完)
    if (rising && mode != STOPPING) {
        // 默认交还伺服速度环, 转矩扫频改 CST, 位置环波形改 CSP
        driveMode = DRIVE_CSV;

        switch (experiment) {

        case ExperimentType_StepResponse: {
            // 阶跃: 中心 SigOffset ± SigAmp (每周期取, 运行中可改)
            mode      = STEP_TEST;
            stepPhase = 0;
            elapsed   = 0.0f;
            break;
        }

        case ExperimentType_VelocitySweep: {
            // 速度扫频: 0 rpm 保持后开始扫频
            mode                 = SWEEP_TEST;
            sweepSub             = SWEEP_STANDBY;
            sweepFreqIdx         = 0;
            sweepSegElapsed      = 0.0f;
            sweepPrevCycle       = 0;
            sweepCompletedCycles = 0;
            sweepSubElapsed      = 0.0f;
            break;
        }

        case ExperimentType_TorqueSweep: {
            // 转矩扫频: 上位机速度环稳住 SigOffset, 伺服走 CST
            mode                    = TOR_SWEEP_TEST;
            torSweepSub             = TOR_SWEEP_STANDBY;
            torSweepStartPhase      = 0;
            torSweepFreqIdx         = 0;
            torSweepSegElapsed      = 0.0f;
            torSweepPrevCycle       = 0;
            torSweepCompletedCycles = 0;
            torSweepSubElapsed      = 0.0f;
            torSweepBlend           = 1.0f;
            torSweepSine            = 0.0f;
            torSweepBaseTorque      = 0.0f;
            torSweepMeasAccum       = 0.0f;
            torSweepMeasCount       = 0;
            if (sigFreq != NULL) {
                *sigFreq = 0;
            }
            // 速度环从零状态起步
            ResetSpdLoop();
            driveMode = DRIVE_CST;
            break;
        }

        case ExperimentType_LoadDisturbScan: {
            // 负载扰动扫描: 驱动侧载波 + 负载侧 (频率 × 幅度) 二维扰动
            mode               = LOAD_SCAN_TEST;
            loadScanFreqIdx    = 0;
            loadScanAmpIdx     = 0;
            loadScanAmpSteps   = (uint8_t)LOADSCAN_TEST_AMP_STEPS;
            if (loadScanAmpSteps < 1) {
                loadScanAmpSteps = 1;
            }
            loadScanStarted    = 0;
            loadScanGapNext    = 0;
            loadScanSub        = LOAD_SCAN_STANDBY;
            loadScanSubElapsed = 0.0f;
            loadScanSegElapsed = 0.0f;
            loadScanPrevCycle  = 0;
            loadScanCycles     = 0;
            driveWavePhase     = 0.0f;
            distPhase          = 0.0f;
            loadTorInj         = loadBase;
            if (sigFreq != NULL) {
                *sigFreq = 0;
            }
            // 幅度回读清零
            if (sigAmp != NULL) {
                *sigAmp = 0.0f;
            }
            break;
        }

        case ExperimentType_WaveResponse: {
            // 定频波形: SigOffset + SigAmp × 波形, 频率/幅值每周期读取
            mode           = WAVE_TEST;
            waveType       = WAVE_TEST_SIGNAL_TYPE;
            waveSub        = WAVE_STANDBY;
            waveSubElapsed = 0.0f;
            wavePhase      = 0.0f;
            break;
        }

        case ExperimentType_PosLoopWave: {
            // 位置环波形: 伺服走 CSP(8), 原点/幅值/频率每周期从 Sig* 读取
            mode              = POS_WAVE_TEST;
            posWaveType       = POSWAVE_TEST_SIGNAL_TYPE;
            posWaveSub        = POS_WAVE_STANDBY;
            posWaveSubElapsed = 0.0f;
            posWavePhase      = 0.0f;
            posWaveCycles     = 0;
            posStopElapsed    = 0.0f;
            if (sigOffset != NULL) {
                posCenter = (LONG)(*sigOffset);
            }
            posCmdOut = (double)posCenter;
            // 位置上不使用速度指令 (CSP 下 60FFh 写 0)
            SetSpdCmd(spdCmd, 0.0f);
            driveMode = DRIVE_CSP;
            break;
        }

        case ExperimentType_VelocityClosedLoopPRBS: {
            // 速度闭环 PRBS: 0 速保持 → 基础速度 → 静置 → PRBS + 增量频响
            mode           = PRBS_TEST;
            prbsSub        = PRBS_STANDBY;
            prbsSubElapsed = 0.0f;
            prbsSegElapsed = 0.0f;
            // 全 1 初值: LFSR 全 0 会永久卡死
            prbsReg        = (uint16_t)((1u << PRBS_TEST_LFSR_BITS) - 1u);
            prbsAcc        = 0.0f;
            prbsBit        = 1.0f;
            // SigFreq 是用户输入 (PRBS 位速率), 保持不动
            break;
        }

        case ExperimentType_FeedCamTest: {
            // 送纸凸轮: 伺服走 CSP(8); 虚拟主轴 + 4 轴五次曲线 (v1 仅轴0 下发)
            // 启动逻辑整圈对齐 (对齐 FAE 工程02); SigOffset = 线速度 [m/min],
            // SigAmp = Start 触发 (0→非0 上升沿送一次)
            if (!feedSeq.init(&g_cfgFeed)) {
                // 参数非法: 不启动实验, 检查 g_cfgFeed (nAxisCount / afSyncLen 等)
                break;
            }
            mode = FEED_CAM_TEST;
            feedMaster.setVelocity(((sigOffset != NULL) ? (double)*sigOffset : 0.0) * 1000.0 / 60.0);
            feedMaster.start();
            // 指令原点 = 当前实际位置 (6064h): 凸轮位置作为相对偏移叠加, 防使能跳变;
            // posCenter 同步为该原点 (STOPPING 公共段复用: 停机回原点保持后禁用)
            feedActPos0    = actPosUnits;
            posCenter      = (LONG)actPosUnits;
            feedPrevSigAmp = 0.0f;
            posCmdOut      = actPosUnits;
            SetSpdCmd(spdCmd, 0.0f);
            driveMode      = DRIVE_CSP;
            break;
        }

        case ExperimentType_Manual:
        default:
            // 其它取值不启动实验, 保持 IDLE
            break;
        }
    }

    // 按当前模式推进
    switch (mode) {

    case IDLE:
        return;

    case SWEEP_TEST: {
        float freq      = SweepFreqTable[sweepFreqIdx];
        // 基础速度与幅值 (每周期取 SigOffset / SigAmp, 运行中可改)
        float baseSpd   = (sigOffset != NULL) ? *sigOffset : 0.0f;
        float ampl      = (sigAmp != NULL) ? *sigAmp : 0.0f;
        float twoPiFreq = 6.283185307f * freq;

        switch (sweepSub) {

        case SWEEP_STANDBY: {
            // 0 rpm 保持
            if (sigFreq != NULL) {
                *sigFreq = 0;
            }
            SetSpdCmd(spdCmd, 0.0f);
            sweepSubElapsed += dtSec;
            if (sweepSubElapsed >= SWEEP_TEST_STANDBY_DUR) {
                sweepSub        = SWEEP_RAMP_UP;
                sweepSubElapsed = 0.0f;
            }
            return;
        }

        case SWEEP_RAMP_UP: {
            // 匀加速到基础速度
            *sigFreq = 0;
            float rampDir = (baseSpd >= 0.0f) ? 1.0f : -1.0f;
            float rampSpd = rampDir * TEST_RAMP_RATE * sweepSubElapsed;
            // 斜坡时长 [s]
            float rampDur = fabsf(baseSpd) / TEST_RAMP_RATE;

            if ((baseSpd >= 0.0f && rampSpd >= baseSpd) ||
                (baseSpd < 0.0f && rampSpd <= baseSpd)) {
                rampSpd = baseSpd;
            }
            SetSpdCmd(spdCmd, rampSpd);
            sweepSubElapsed += dtSec;

            // 到位后间隔 GAP 再开始首频点
            if (sweepSubElapsed >= rampDur + SWEEP_TEST_GAP_DUR) {
                sweepSub             = SWEEP_ACTIVE;
                sweepSegElapsed      = 0.0f;
                sweepPrevCycle       = 0;
                sweepCompletedCycles = 0;
                sweepSubElapsed      = 0.0f;
            }
            return;
        }

        case SWEEP_ACTIVE: {
            // 基础速度 + 正弦
            *sigFreq = freq;
            SetSpdCmd(spdCmd, baseSpd + ampl * sinf(twoPiFreq * sweepSegElapsed));
            sweepSegElapsed += dtSec;

            // 整周期边界
            int curCycle = (int)(sweepSegElapsed * freq);
            if (curCycle > sweepPrevCycle) {
                sweepCompletedCycles += (uint16_t)(curCycle - sweepPrevCycle);
                sweepPrevCycle = curCycle;
                if (sweepSegElapsed >= SWEEP_TEST_MIN_DUR &&
                    sweepCompletedCycles >= SWEEP_TEST_MIN_CYCLES) {
                    sweepSub        = SWEEP_GAP;
                    sweepSubElapsed = 0.0f;
                    return;
                }
            }

            // 条件在中途满足: 等当前整周期结束
            if (sweepSegElapsed >= SWEEP_TEST_MIN_DUR &&
                sweepCompletedCycles >= SWEEP_TEST_MIN_CYCLES) {
                sweepSub        = SWEEP_FINISHING;
                sweepSubElapsed = 0.0f;
            }
            return;
        }

        case SWEEP_FINISHING: {
            // 继续正弦直到整周期结束
            *sigFreq = freq;
            SetSpdCmd(spdCmd, baseSpd + ampl * sinf(twoPiFreq * sweepSegElapsed));
            sweepSegElapsed += dtSec;
            sweepSubElapsed += dtSec;

            int curCycle = (int)(sweepSegElapsed * freq);
            if (curCycle > sweepPrevCycle) {
                sweepCompletedCycles += (uint16_t)(curCycle - sweepPrevCycle);
                sweepPrevCycle = curCycle;
                sweepSub        = SWEEP_GAP;
                sweepSubElapsed = 0.0f;
                return;
            }

            // 超时保护: 一个周期 + 0.5 s
            float period = 1.0f / freq;
            if (sweepSubElapsed >= (period + 0.5f)) {
                sweepSub        = SWEEP_GAP;
                sweepSubElapsed = 0.0f;
            }
            return;
        }

        case SWEEP_GAP: {
            // 频点间: 只给基础速度
            *sigFreq = 0;
            SetSpdCmd(spdCmd, baseSpd);
            sweepSubElapsed += dtSec;
            if (sweepSubElapsed >= SWEEP_TEST_GAP_DUR) {
                sweepFreqIdx++;
                if (sweepFreqIdx >= SWEEP_TEST_NUM_FREQS) {
                    // 全部频点完成: 斜坡停机
                    *sigFreq = 0;
                    stopClearFreq  = 1;
                    mode           = STOPPING;
                    return;
                }
                sweepSub             = SWEEP_ACTIVE;
                sweepSegElapsed      = 0.0f;
                sweepPrevCycle       = 0;
                sweepCompletedCycles = 0;
                sweepSubElapsed      = 0.0f;
            }
            return;
        }
        }
        return;
    }

    case TOR_SWEEP_TEST: {
        // 输出 = blend·速度环 + (1-blend)·平均转矩 + 正弦
        // 基础速度 [rpm] 与正弦幅值 [0.1% 额定转矩] (每周期取, 运行中可改)
        float base  = (sigOffset != NULL) ? *sigOffset : 0.0f;
        float amp   = (sigAmp != NULL) ? *sigAmp : 0.0f;
        // 参考速度
        float ref   = base;
        // 速度环输出权重
        float blend = 1.0f;
        // 本拍扫频正弦 [0.1% 额定转矩]
        float sine  = 0.0f;

        switch (torSweepSub) {

        case TOR_SWEEP_STANDBY: {
            // 启动: 0 速保持 → 匀加速 → 基础速度保持
            *sigFreq = 0;
            if (torSweepStartPhase == 0) {
                ref = 0.0f;
            } else if (torSweepStartPhase == 1) {
                // 匀加速
                ref = base * (torSweepSubElapsed / TOR_SWEEP_TEST_RAMP_DUR);
            }
            break;
        }

        case TOR_SWEEP_MEASURE: {
            // 测量段: 速度环输出直通, 累加后求平均
            *sigFreq = 0;
            break;
        }

        case TOR_SWEEP_SETTLE: {
            // 静置段: 前 SETTLE_RAMP 比例内从速度环过渡到平均转矩
            *sigFreq = 0;
            // 过渡时长 [s]
            float rampDur = TOR_SWEEP_TEST_SETTLE_DUR * TOR_SWEEP_TEST_SETTLE_RAMP;
            blend = 0.0f;
            if (rampDur > 0.0f && torSweepSubElapsed < rampDur) {
                // 1 -> 0
                blend = 1.0f - torSweepSubElapsed / rampDur;
            }
            break;
        }

        case TOR_SWEEP_ACTIVE:
        case TOR_SWEEP_FINISHING: {
            // 震荡段: 平均转矩 + 扫频正弦
            blend = 0.0f;
            // 当前频点 [Hz]
            float freq = TorSweepFreqTable[torSweepFreqIdx];
            sine = amp * sinf(6.283185307f * freq * torSweepSegElapsed);
            *sigFreq = freq;
            break;
        }

        case TOR_SWEEP_RECOVER: {
            // 恢复段: 前 RECOVER_RAMP 比例内从平均转矩交还速度环
            *sigFreq = 0;
            // 过渡时长 [s]
            float rampDur = TOR_SWEEP_TEST_RECOVER_DUR * TOR_SWEEP_TEST_RECOVER_RAMP;
            blend = 1.0f;
            if (rampDur > 0.0f && torSweepSubElapsed < rampDur) {
                // 0 -> 1
                blend = torSweepSubElapsed / rampDur;
            }
            break;
        }
        }

        SetSpdCmd(spdCmd, ref);

        // 速度环只在驱动侧使能时积分 (未使能时冻结)
        float loopOut = 0.0f;
        if (driveReady) {
            loopOut = spdLoop.Step(ref, spdFb);
        }

        // 测量段: 累加速度环输出
        if (torSweepSub == TOR_SWEEP_MEASURE) {
            torSweepMeasAccum += loopOut;
            torSweepMeasCount++;
        }

        // 合成 + 限幅
        float torCmd = blend * loopOut + (1.0f - blend) * torSweepBaseTorque + sine;
        if (torCmd >  torLim) torCmd =  torLim;
        if (torCmd < -torLim) torCmd = -torLim;
        torCmdOut     = torCmd;
        torSweepBlend = blend;
        torSweepSine  = sine;

        // 时序推进
        switch (torSweepSub) {

        case TOR_SWEEP_STANDBY:
            torSweepSubElapsed += dtSec;
            if (torSweepStartPhase == 0 &&
                torSweepSubElapsed >= TOR_SWEEP_TEST_ZERO_DUR) {
                torSweepStartPhase = 1;
                torSweepSubElapsed = 0.0f;
            } else if (torSweepStartPhase == 1 &&
                       torSweepSubElapsed >= TOR_SWEEP_TEST_RAMP_DUR) {
                torSweepStartPhase = 2;
                torSweepSubElapsed = 0.0f;
            } else if (torSweepStartPhase == 2 &&
                       torSweepSubElapsed >= TOR_SWEEP_TEST_STANDBY_DUR) {
                // 进入首频点测量段
                torSweepSub        = TOR_SWEEP_MEASURE;
                torSweepSubElapsed = 0.0f;
                torSweepMeasAccum  = 0.0f;
                torSweepMeasCount  = 0;
                torSweepBaseTorque = 0.0f;
            }
            return;

        case TOR_SWEEP_MEASURE:
            torSweepSubElapsed += dtSec;
            if (torSweepSubElapsed >= TOR_SWEEP_TEST_MEASURE_DUR) {
                if (torSweepMeasCount > 0) {
                    torSweepBaseTorque = torSweepMeasAccum / (float)torSweepMeasCount;
                }
                // 进入静置段
                torSweepSub        = TOR_SWEEP_SETTLE;
                torSweepSubElapsed = 0.0f;
            }
            return;

        case TOR_SWEEP_SETTLE:
            torSweepSubElapsed += dtSec;
            if (torSweepSubElapsed >= TOR_SWEEP_TEST_SETTLE_DUR) {
                // 进入震荡段
                torSweepSub             = TOR_SWEEP_ACTIVE;
                torSweepSegElapsed      = 0.0f;
                torSweepPrevCycle       = 0;
                torSweepCompletedCycles = 0;
                torSweepSubElapsed      = 0.0f;
            }
            return;

        case TOR_SWEEP_ACTIVE: {
            // 当前频点 [Hz]
            float freq = TorSweepFreqTable[torSweepFreqIdx];
            torSweepSegElapsed += dtSec;

            // 整周期边界检测 (f*t 整数穿越)
            int curCycle = (int)(torSweepSegElapsed * freq);
            if (curCycle > torSweepPrevCycle) {
                torSweepCompletedCycles += (uint16_t)(curCycle - torSweepPrevCycle);
                torSweepPrevCycle = curCycle;
                // 最小驻留时间与最小周期数同时满足 → 进入恢复段
                if (torSweepSegElapsed >= TOR_SWEEP_TEST_MIN_DUR &&
                    torSweepCompletedCycles >= TOR_SWEEP_TEST_MIN_CYCLES) {
                    torSweepSub        = TOR_SWEEP_RECOVER;
                    torSweepSubElapsed = 0.0f;
                    return;
                }
            }
            // 条件在周期中途满足 → 完成当前周期再切换
            if (torSweepSegElapsed >= TOR_SWEEP_TEST_MIN_DUR &&
                torSweepCompletedCycles >= TOR_SWEEP_TEST_MIN_CYCLES) {
                torSweepSub        = TOR_SWEEP_FINISHING;
                torSweepSubElapsed = 0.0f;
            }
            return;
        }

        case TOR_SWEEP_FINISHING: {
            // 继续输出 基础转矩 + 正弦 直至当前周期自然结束
            // 当前频点 [Hz]
            float freq = TorSweepFreqTable[torSweepFreqIdx];
            torSweepSegElapsed += dtSec;
            torSweepSubElapsed += dtSec;

            // 周期边界 → 进入恢复段
            int curCycle = (int)(torSweepSegElapsed * freq);
            if (curCycle > torSweepPrevCycle) {
                torSweepPrevCycle  = curCycle;
                torSweepSub        = TOR_SWEEP_RECOVER;
                torSweepSubElapsed = 0.0f;
                return;
            }

            // 安全超时: 一个周期 + 0.05s
            float period = 1.0f / freq;
            if (torSweepSubElapsed >= (period + 0.05f)) {
                torSweepSub        = TOR_SWEEP_RECOVER;
                torSweepSubElapsed = 0.0f;
            }
            return;
        }

        case TOR_SWEEP_RECOVER:
            torSweepSubElapsed += dtSec;
            if (torSweepSubElapsed >= TOR_SWEEP_TEST_RECOVER_DUR) {
                torSweepFreqIdx++;
                if (torSweepFreqIdx >= TOR_SWEEP_TEST_NUM_FREQS) {
                    // 全部频点完成: 速度斜坡停机 (期间速度环继续出转矩)
                    *sigFreq  = 0;
                    stopClearFreq = 1;
                    mode          = STOPPING;
                    return;
                }
                // 下一频点: 直接进入静置段 (基础转矩沿用起始测量段结果)
                torSweepSub        = TOR_SWEEP_SETTLE;
                torSweepSubElapsed = 0.0f;
            }
            return;
        }
        return;
    }

    case WAVE_TEST: {
        // 基础速度 [rpm]: 每周期取 SigOffset, 运行中可改
        float baseSpd = (sigOffset != NULL) ? *sigOffset : 0.0f;
        // 幅值 [rpm]: 每周期取 SigAmp, 运行中可改
        float ampl    = (sigAmp != NULL) ? *sigAmp : 0.0f;

        // 波形类型 (1=正弦 2=三角 3=梯形)
        if (waveType < 1 || waveType > 3) {
            waveType = 1;
        }

        switch (waveSub) {

        case WAVE_STANDBY: {
            // 0 rpm 保持
            SetSpdCmd(spdCmd, 0.0f);
            waveSubElapsed += dtSec;
            if (waveSubElapsed >= WAVE_TEST_STANDBY_DUR) {
                waveSub        = WAVE_RAMP_UP;
                waveSubElapsed = 0.0f;
            }
            return;
        }

        case WAVE_RAMP_UP: {
            // 匀加速到基础速度
            float rampDir = (baseSpd >= 0.0f) ? 1.0f : -1.0f;
            float rampSpd = rampDir * TEST_RAMP_RATE * waveSubElapsed;
            // 斜坡时长 [s]
            float rampDur = fabsf(baseSpd) / TEST_RAMP_RATE;

            if ((baseSpd >= 0.0f && rampSpd >= baseSpd) ||
                (baseSpd < 0.0f && rampSpd <= baseSpd)) {
                rampSpd = baseSpd;
            }
            SetSpdCmd(spdCmd, rampSpd);
            waveSubElapsed += dtSec;

            // 到位后开始波形, 相位从 0 起
            if (waveSubElapsed >= rampDur) {
                waveSub        = WAVE_ACTIVE;
                waveSubElapsed = 0.0f;
                wavePhase      = 0.0f;
            }
            return;
        }

        case WAVE_ACTIVE: {
            // 频率每周期取 SigFreq [Hz], 0 = 只给基础速度
            float freq = *sigFreq;
            float cmd  = baseSpd;
            if (freq > 0.0f) {
                // 相位累加, 改频率时波形连续
                wavePhase += 6.283185307f * freq * dtSec;
                switch (waveType) {
                case 2:
                    cmd = baseSpd + ampl * WaveTriangle(wavePhase);
                    break;
                case 3:
                    cmd = baseSpd + ampl * WaveTrapezoid(wavePhase);
                    break;
                default:
                    // 正弦
                    cmd = baseSpd + ampl * sinf(wavePhase);
                    break;
                }
            }
            SetSpdCmd(spdCmd, cmd);
            waveSubElapsed += dtSec;
            return;
        }
        }
        return;
    }

    case POS_WAVE_TEST: {
        // 波形原点 [指令单位]: 每周期取 SigOffset (运行中可改)
        if (sigOffset != NULL) {
            posCenter = (LONG)(*sigOffset);
        }
        // 波形原点 (double 累加, 绝对位置可达 2^31)
        double center = (double)posCenter;
        // 幅值每周期取 SigAmp [指令单位], 超范围夹紧并回读
        float ampCnt = (sigAmp != NULL) ? *sigAmp : 0.0f;
        if (ampCnt >  POSWAVE_TEST_AMP_MAX) ampCnt =  POSWAVE_TEST_AMP_MAX;
        if (ampCnt < -POSWAVE_TEST_AMP_MAX) ampCnt = -POSWAVE_TEST_AMP_MAX;
        if (sigAmp != NULL) {
            *sigAmp = ampCnt;
        }

        // 波形类型 (1=正弦 2=三角 3=梯形)
        if (posWaveType < 1 || posWaveType > 3) {
            posWaveType = 1;
        }

        switch (posWaveSub) {

        case POS_WAVE_STANDBY: {
            // 先只给原点: 波形开始前不叠加任何偏移
            posCmdOut = (double)posCenter;
            posWaveSubElapsed += dtSec;
            if (posWaveSubElapsed >= POSWAVE_TEST_STANDBY_DUR) {
                posWaveSub        = POS_WAVE_ACTIVE;
                posWaveSubElapsed = 0.0f;
                // 波形相位从 0 起 (正弦/三角在该相位波形值为 0);
                // 梯形波在 0 相位处是 -1, 改从上升段中点起步 (波形值 0),
                // 避免目标位置出现阶跃; 上升比例为 0 时是阶跃, 相位仍从 0 起
                posWavePhase = 0.0f;
                if (posWaveType == 3) {
                    posWavePhase = 0.5f * PosTrapRise() * 3.141592654f;
                }
            }
            return;
        }

        default:
            break;   // POS_WAVE_ACTIVE
        }

        // 频率每周期取 SigFreq [Hz], 0 = 冻结相位, 保持当前目标位置
        float freq = (sigFreq != NULL) ? *sigFreq : 0.0f;
        if (freq > 0.0f) {
            // 相位累加后回绕, 长时间运行不丢精度 (改频率时波形连续)
            posWavePhase += 6.283185307f * freq * dtSec;
            if (posWavePhase >= 6.283185307f) {
                posWavePhase -= 6.283185307f;
                // 相位每回绕一次算一个完整周期
                posWaveCycles++;
            }
        }
        // 本拍波形值 [-1, 1]
        float wave = WaveValue(posWaveType, posWavePhase);
        posCmdOut = center + (double)(ampCnt * wave);
        posWaveSubElapsed += dtSec;

        // 跑满设定的周期数: 目标直接给基准位置, 保持 HOLD 后禁用 (不做淡出)
        if ((POSWAVE_TEST_CYCLES > 0) &&
            (posWaveCycles >= (uint32_t)POSWAVE_TEST_CYCLES)) {
            posStopElapsed = 0.0f;
            posCmdOut      = (double)posCenter;
            SetSpdCmd(spdCmd, 0.0f);
            mode = STOPPING;
        }
        return;
    }

    case FEED_CAM_TEST: {
        // 虚拟主轴线速度 [m/min] → [mm/s]: 每周期取 SigOffset, 运行中可改
        float lineMMin = (sigOffset != NULL) ? *sigOffset : 0.0f;
        feedMaster.setVelocity((double)lineMMin * 1000.0 / 60.0);

        // Start 触发: SigAmp 0→非0 上升沿触发一次送纸循环 (循环中忽略, 再触发先回 0)
        float startSig = (sigAmp != NULL) ? *sigAmp : 0.0f;
        if (startSig > 0.0f && feedPrevSigAmp <= 0.0f) {
            feedSeq.setStart(true);
        }
        feedPrevSigAmp = startSig;

        // 实际位置换算 [mm] (跟随偏差监视); 换算未挂载 (=0) 时不监视
        double actMm[4] = {0.0, 0.0, 0.0, 0.0};
        const double* pAct = NULL;
        if (feedMmToUnit > 0.0) {
            actMm[0] = (actPosUnits - feedActPos0) / feedMmToUnit;
            pAct = actMm;
        }

        // 虚拟主轴积分 + 状态机推进 (4 轴指令 [mm], v1 仅轴0 下发)
        feedMaster.tick(dtSec);
        feedSeq.tick(feedMaster.getPhase(), dtSec, pAct, feedCmdMm);

        // 轴0 指令: 指令原点 + 凸轮位置 × 换算 (double 累加, 同 POS_WAVE 精度处理)
        posCmdOut = feedActPos0 + feedCmdMm[0] * feedMmToUnit;

        // SigFreq 回读: IDLE = 换算系数 feedMmToUnit (诊断: 应 ≈ 671, 0 = 未挂载),
        //                运行中 = 凸轮相位 camX [mm]
        if (sigFreq != NULL) {
            FEED_MONITOR_T mon;
            feedSeq.getMonitor(&mon);
            *sigFreq = (mon.nState == 0) ? (float)feedMmToUnit : (float)mon.fCamX;
        }
        SetSpdCmd(spdCmd, 0.0f);   // CSP 下速度指令不用
        return;
    }

    case PRBS_TEST: {
        // 基础速度 [rpm] / 幅值 [rpm] / 分析带半宽 [Hz]: 每周期取, 运行中可改
        float baseSpd = (sigOffset != NULL) ? *sigOffset : 0.0f;
        float ampl    = (sigAmp != NULL) ? *sigAmp : 0.0f;
        float freq    = (sigFreq != NULL) ? *sigFreq : 0.0f;

        switch (prbsSub) {

        case PRBS_STANDBY: {
            // 0 rpm 保持
            SetSpdCmd(spdCmd, 0.0f);
            prbsSubElapsed += dtSec;
            if (prbsSubElapsed >= PRBS_TEST_STANDBY_DUR) {
                prbsSub        = PRBS_RAMP_UP;
                prbsSubElapsed = 0.0f;
            }
            return;
        }

        case PRBS_RAMP_UP: {
            // 匀加速到基础速度
            float rampDir = (baseSpd >= 0.0f) ? 1.0f : -1.0f;
            float rampSpd = rampDir * TEST_RAMP_RATE * prbsSubElapsed;
            // 斜坡时长 [s]
            float rampDur = fabsf(baseSpd) / TEST_RAMP_RATE;
            if ((baseSpd >= 0.0f && rampSpd >= baseSpd) ||
                (baseSpd < 0.0f && rampSpd <= baseSpd)) {
                rampSpd = baseSpd;
            }
            SetSpdCmd(spdCmd, rampSpd);
            prbsSubElapsed += dtSec;
            if (prbsSubElapsed >= rampDur) {
                prbsSub        = PRBS_SETTLE;
                prbsSubElapsed = 0.0f;
            }
            return;
        }

        case PRBS_SETTLE: {
            // 基础速度保持: 等速度环进入稳态再激励 (否则第一段被暂态污染)
            SetSpdCmd(spdCmd, baseSpd);
            prbsSubElapsed += dtSec;
            if (prbsSubElapsed >= PRBS_TEST_SETTLE_DUR) {
                prbsSub        = PRBS_ACTIVE;
                prbsSubElapsed = 0.0f;
                prbsSegElapsed = 0.0f;
                prbsAcc        = 0.0f;
                prbsBit        = 1.0f;
                // LFSR 每次从全 1 重新起 (序列可重复, 便于离线复用同一条激励)
                prbsReg        = (uint16_t)((1u << PRBS_TEST_LFSR_BITS) - 1u);
            }
            return;
        }

        default:
            break;   // PRBS_ACTIVE
        }

        // PRBS 位推进: 一个 bit 时长 = 1/SigFreq [s]
        if (freq > 0.0f) {
            // 一个 bit 的时长 [s]
            float bitDur = 1.0f / freq;
            prbsAcc += dtSec;
            if (prbsAcc >= bitDur) {
                prbsAcc -= bitDur;
                // 保护: 位速率高于控制频率时一拍照样只推一位
                if (prbsAcc >= bitDur) {
                    prbsAcc = 0.0f;
                }
                PrbsShift(&prbsReg, &prbsBit);
            }
        }

        // 基础速度 + PRBS (±幅值)
        SetSpdCmd(spdCmd, baseSpd + ampl * prbsBit);
        prbsSegElapsed += dtSec;

        // 时长到: 走通用斜坡停机
        if ((PRBS_TEST_DUR > 0.0f) && (prbsSegElapsed >= PRBS_TEST_DUR)) {
            mode = STOPPING;
        }
        return;
    }

    case STEP_TEST: {
        // 阶跃中心 [rpm] 与幅值 [rpm]: 每周期取 SigOffset / SigAmp
        float stepCenter = (sigOffset != NULL) ? *sigOffset : 0.0f;
        float stepAmp    = (sigAmp != NULL) ? *sigAmp : 0.0f;
        // 相位 0: 0 rpm; 1: 中心+幅值; 2: 中心-幅值
        if (stepPhase == 0) {
            SetSpdCmd(spdCmd, 0.0f);
            if (elapsed >= STEP_TEST_ZERO_DUR) {
                stepPhase = 1;
                elapsed   = 0.0f;
                // 本拍不计时, 各相位等长
                return;
            }
        } else if (stepPhase == 1) {
            // 高电平
            SetSpdCmd(spdCmd, stepCenter + stepAmp);
            if (elapsed >= STEP_TEST_SPD_DUR) {
                stepPhase = 2;
                elapsed   = 0.0f;
                return;
            }
        } else {
            // 低电平
            SetSpdCmd(spdCmd, stepCenter - stepAmp);
            if (elapsed >= STEP_TEST_RETURN_DUR) {
                // 阶跃结束
                *enable = 0;
                SetSpdCmd(spdCmd, 0.0f);
                mode    = DONE;
                return;
            }
        }
        elapsed += dtSec;
        return;
    }

    case LOAD_SCAN_TEST: {
        // 驱动侧载波中心速度与幅值 (固定宏, 不属于注入信号)
        float carrierBase = LOADSCAN_TEST_BASE_SPD;
        float carrierAmp  = LOADSCAN_TEST_AMP_SPD;
        // 启动: 0 速保持 → 匀加速到载波中心; 之后 GAP / ACTIVE
        switch (loadScanSub) {

        case LOAD_SCAN_STANDBY: {
            // 0 速保持
            SetSpdCmd(spdCmd, 0.0f);
            loadTorInj   = loadBase;
            *sigFreq = 0;
            loadScanSubElapsed += dtSec;
            if (loadScanSubElapsed >= LOADSCAN_TEST_STANDBY_DUR) {
                loadScanSub        = LOAD_SCAN_RAMP;
                loadScanSubElapsed = 0.0f;
            }
            return;
        }

        case LOAD_SCAN_RAMP: {
            // 匀加速到载波中心速度
            float rampDir  = (carrierBase >= 0.0f) ? 1.0f : -1.0f;
            float rampSpd  = rampDir * TEST_RAMP_RATE * loadScanSubElapsed;
            float rampDur  = fabsf(carrierBase) / TEST_RAMP_RATE;
            if ((carrierBase >= 0.0f && rampSpd >= carrierBase) ||
                (carrierBase < 0.0f && rampSpd <= carrierBase)) {
                rampSpd = carrierBase;
            }
            SetSpdCmd(spdCmd, rampSpd);
            loadTorInj   = loadBase;
            *sigFreq = 0;
            loadScanSubElapsed += dtSec;
            if (loadScanSubElapsed >= rampDur) {
                // 首个 GAP 后开始注入
                loadScanSub        = LOAD_SCAN_GAP;
                loadScanSubElapsed = 0.0f;
                driveWavePhase     = 0.0f;
            }
            return;
        }

        case LOAD_SCAN_GAP: {
            // 间隔段: 仅载波, 不注入扰动 (负载侧只有基准转矩)
            driveWavePhase += 6.283185307f * LOADSCAN_TEST_WAVE_FREQ * dtSec;
            SetSpdCmd(spdCmd, carrierBase + carrierAmp * sinf(driveWavePhase));
            loadTorInj   = loadBase;
            *sigFreq = 0;
            loadScanSubElapsed += dtSec;
            if (loadScanSubElapsed >= LOADSCAN_TEST_GAP_DUR) {
                if (loadScanStarted == 0) {
                    // 起始驻留结束, 进入首频点
                    loadScanStarted = 1;
                } else if (loadScanGapNext == 1) {
                    // 本频率扫完, 换下一频率
                    loadScanFreqIdx++;
                    if (loadScanFreqIdx >= LOADSCAN_TEST_NUM_FREQS) {
                        // 全部完成: 斜坡停机
                        *sigFreq  = 0;
                        stopClearFreq = 1;
                        mode          = STOPPING;
                        return;
                    }
                }
                // loadScanGapNext = 0: 同频率下一幅度
                loadScanSub        = LOAD_SCAN_ACTIVE;
                loadScanSubElapsed = 0.0f;
                loadScanSegElapsed = 0.0f;
                loadScanPrevCycle  = 0;
                loadScanCycles     = 0;
                distPhase          = 0.0f;
            }
            return;
        }

        default:
            break;   // LOAD_SCAN_ACTIVE
        }

        // 载波 + 当前 (频率, 幅度) 扰动
        driveWavePhase += 6.283185307f * LOADSCAN_TEST_WAVE_FREQ * dtSec;
        SetSpdCmd(spdCmd, carrierBase + carrierAmp * sinf(driveWavePhase));

        {
            float f = LoadScanFreqTable[loadScanFreqIdx];
            // 幅度 [% 负载额定], 在本频率内按步数均分
            float stepsF = (loadScanAmpSteps > 1) ? (float)(loadScanAmpSteps - 1) : 1.0f;
            float pct    = LOADSCAN_TEST_AMP_MIN_PCT +
                           (LOADSCAN_TEST_AMP_MAX_PCT - LOADSCAN_TEST_AMP_MIN_PCT) *
                               (float)loadScanAmpIdx / stepsF;
            distPhase += 6.283185307f * f * dtSec;
            // 负载侧转矩 = 基准 (SigOffset) + 注入正弦 [0.1% 额定]
            loadTorInj = loadBase + (pct * 10.0f) * sinf(distPhase);
            *sigFreq = f;
            // 幅度回读 [% 负载额定转矩]
            *sigAmp  = pct;

            // 整周期边界: 驻留时间与周期数都满足后换段
            loadScanSegElapsed += dtSec;
            int curCycle = (int)(loadScanSegElapsed * f);
            if (curCycle > loadScanPrevCycle) {
                loadScanCycles += (uint16_t)(curCycle - loadScanPrevCycle);
                loadScanPrevCycle = curCycle;
                if (loadScanSegElapsed >= LOADSCAN_TEST_MIN_DUR &&
                    loadScanCycles >= LOADSCAN_TEST_MIN_CYCLES) {
                    // 本段完成, 进 GAP, 由 loadScanGapNext 决定去向
                    loadScanAmpIdx++;
                    loadScanGapNext = (loadScanAmpIdx < loadScanAmpSteps) ? 0 : 1;
                    if (loadScanGapNext == 1) {
                        // 本频率扫完, 下一段回到最小幅度
                        loadScanAmpIdx = 0;
                    }
                    loadScanSub        = LOAD_SCAN_GAP;
                    loadScanSubElapsed = 0.0f;
                    loadTorInj         = loadBase;
                    *sigFreq       = 0;
                }
            }
        }
        return;
    }

    case STOPPING: {
        // 位置环波形: CSP 下不做速度斜坡, 目标位置直接给基准位置 (原点),
        // 保持 POSWAVE_TEST_HOLD_DUR 后禁用驱动器 (不再做幅值淡出)
        if (driveMode == DRIVE_CSP) {
            // 目标位置 = 基准位置 [指令单位]
            posCmdOut = (double)posCenter;

            // 保持期间使能, 速度指令保持 0 (CSP 下不使用)
            *enable = 1;
            SetSpdCmd(spdCmd, 0.0f);
            posStopElapsed += dtSec;

            // 保持完成: 禁用驱动器, 结束实验
            if (posStopElapsed >= POSWAVE_TEST_HOLD_DUR) {
                *enable = 0;
                mode    = DONE;
            }
            return;
        }

        // 以 TEST_RAMP_RATE 减速到 0
        float stopDir = (lastSpdCmd >= 0.0f) ? -1.0f : 1.0f;
        float stopSpd = lastSpdCmd + stopDir * TEST_RAMP_RATE * dtSec;

        // 斜坡期间保持使能
        *enable = 1;

        // 过 0 即到 0
        if ((lastSpdCmd >= 0.0f && stopSpd <= 0.0f) ||
            (lastSpdCmd < 0.0f && stopSpd >= 0.0f)) {
            stopSpd = 0.0f;
        }
        SetSpdCmd(spdCmd, stopSpd);

        // 转矩扫频: CST 下由上位机速度环按斜坡指令出制动转矩
        if (driveMode == DRIVE_CST) {
            float loopOut = 0.0f;
            if (driveReady) {
                loopOut = spdLoop.Step(stopSpd, spdFb);
            }
            if (loopOut >  torLim) loopOut =  torLim;
            if (loopOut < -torLim) loopOut = -torLim;
            torCmdOut = loopOut;
        }

        // 到 0: 禁用驱动器, 结束
        if (fabsf(stopSpd) <= TEST_RAMP_STOP_THRESH) {
            *enable = 0;
            SetSpdCmd(spdCmd, 0.0f);
            if (stopClearFreq) {
                *sigFreq = 0;
            }
            mode          = DONE;
            stopClearFreq = 0;
        }
        return;
    }

    case DONE:
        *enable = 0;
        SetSpdCmd(spdCmd, 0.0f);
        // Enable 释放后回到 IDLE
        if (extEnable == 0) {
            mode      = IDLE;
            driveMode = DRIVE_CSV;
        }
        return;
    }
}
