///////////////////////////////////////////////////////////////////////////////
// TestScriptConfig.h
// @brief  实验激励参数 (编译期宏)。调测试曲线只改本文件。
//
// 单位: 速度 [rpm], 转矩 [0.1% 额定], 时间 [s], 频率 [Hz]。
///////////////////////////////////////////////////////////////////////////////
#ifndef TESTSCRIPTCONFIG_H_
#define TESTSCRIPTCONFIG_H_

// ---------------- 速度扫频 (VelocitySweep) ----------------
// 基础速度 [rpm]
#define SWEEP_TEST_BASE_SPD           900.0f
// 正弦幅值 [rpm]
#define SWEEP_TEST_AMPLITUDE          300.0f
// 每频点最小驻留 [s]
#define SWEEP_TEST_MIN_DUR            0.2f
// 每频点最小完整周期数
#define SWEEP_TEST_MIN_CYCLES         4
// 频点间间隔 [s]
#define SWEEP_TEST_GAP_DUR            0.2f
// 启动 0 速保持 [s]
#define SWEEP_TEST_STANDBY_DUR        1.0f
// 频点数
#define SWEEP_TEST_NUM_FREQS          64

// ---------------- 转矩扫频 (TorqueSweep) ----------------
// 启动: 0 速保持 [s]
#define TOR_SWEEP_TEST_ZERO_DUR      1.0f
// 启动: 匀加速 0 → 基础速度 [s]
#define TOR_SWEEP_TEST_RAMP_DUR      1.0f
// 启动: 基础速度保持 (速度环收敛) [s]
#define TOR_SWEEP_TEST_STANDBY_DUR   5.0f
// 测量段 (测速度环平均转矩) [s]
#define TOR_SWEEP_TEST_MEASURE_DUR   1.0f
// 静置段 [s]
#define TOR_SWEEP_TEST_SETTLE_DUR    0.2f
// 静置段起始过渡占比 (0.5 = 前 50% 内由速度环输出过渡到平均转矩)
#define TOR_SWEEP_TEST_SETTLE_RAMP   0.5f
// 恢复段 [s]
#define TOR_SWEEP_TEST_RECOVER_DUR   0.5f
// 恢复段起始过渡占比 (0.2 = 前 20% 内交还速度环输出)
#define TOR_SWEEP_TEST_RECOVER_RAMP  0.2f
// 基础速度 [rpm]
#define TOR_SWEEP_TEST_BASE_SPD      900.0f
// 扫频幅值 [0.1% 额定转矩] (1000 = 100%)
#define TOR_SWEEP_TEST_AMPLITUDE     1000.0f
// 每频点最小驻留 [s]
#define TOR_SWEEP_TEST_MIN_DUR       0.2f
// 每频点最小完整周期数
#define TOR_SWEEP_TEST_MIN_CYCLES    4
// 频点数
#define TOR_SWEEP_TEST_NUM_FREQS     75

// ---------------- 速度波形 (WaveResponse) ----------------
// 波形: 1=正弦 2=三角 3=梯形
#define WAVE_TEST_SIGNAL_TYPE         1
// 基础速度 [rpm]
#define WAVE_TEST_BASE_SPD            -100.0f
// 启动初值: 频率 [Hz] → SigFreq
#define WAVE_TEST_FREQ                0.5f
// 启动初值: 幅值 [rpm] → SigAmp
#define WAVE_TEST_AMPLITUDE           300.0f
// 启动 0 速保持 [s]
#define WAVE_TEST_STANDBY_DUR         1.0f
// 梯形波斜坡时长占周期的比例
#define WAVE_TEST_TRAP_SLOPE          0.25f

// ---------------- 位置环波形 (PosLoopWave, CSP) ----------------
// 伺服工作在周期同步位置模式 (CSP, 6060h = 8), 目标位置 607Ah 由本模块给:
//   目标位置 = 原点 (宏 POSWAVE_TEST_ORIGIN, 绝对位置) + 幅值 × 波形(相位)
// 进入实验后先只给原点并保持 POSWAVE_TEST_STANDBY_DUR 秒 (默认 1 s), 之后才
// 开始叠加波形。注意原点就是绝对目标位置, 伺服使能后会直接走到该位置。
// 跑满 POSWAVE_TEST_CYCLES 个整周期 (0 = 不限) 后实验自动结束; Enable 提前释放
// 也走同样的收尾: 目标直接回原点, 保持 POSWAVE_TEST_HOLD_DUR 秒后禁用驱动器。
// 单位: 6064h 与 607Ah 都是**指令单位** (VD5 手册 7.4 节; SV680N 手册 13.1 节
// "用户位置反馈(6064h) × 齿轮比(6091h) = 电机位置反馈(6063h)"), 所以原点、幅值
// 也用**指令单位**: 与位置反馈、位置偏差窗口 6065h 同一单位, 代码里对 6064 和
// 607A 不做任何比例换算。
// 参考量级: 电机轴 1 转 = 编码器每转计数 × 6091:02 / 6091:01 指令单位
//   Servo0 (VD5L, 23bit, 6091 = 8388608:10000) → 10000 指令单位/电机转
//   Servo1 (SV680N, 26bit, 6091 = 8:1)          → 8388608 指令单位/电机转
//   下面的原点/幅值宏按 Servo0 的 10000 指令单位/转 填写; 换到 Servo1 做位置
//   波形时同一物理值要 ×838.8608
//   负载轴 1 转 = 编码器每转计数 指令单位 (26bit + 8:1 → 67108864)
// 波形峰值速度 [rpm] = 2π × 幅值[指令单位] / 每转指令单位数 × 频率[Hz] × 60
// 频率 SigFreq [Hz] 与幅值 SigAmp [指令单位] 每周期从 Status 读取,
// 运行中可改; 频率 0 Hz 冻结相位 (保持当前目标位置), 幅值 0 停在波形中心。
// 幅值超过 ±POSWAVE_TEST_AMP_MAX 时夹紧并回读到 SigAmp。
// 峰值速度必须低于电机最高转速, 而且不能超出位置偏差窗口 (6065h) 与转矩限幅,
// 否则驱动器报警。
// 波形原点 [指令单位] (绝对位置, 与 6064h/607Ah 同一单位; 指令单位是整数)
// 目标位置就在这个值附近摆动, 伺服使能后会直接走到该位置
#define POSWAVE_TEST_ORIGIN         500
// 波形: 1=正弦 2=三角 3=梯形 (三角/梯形都是位置斜坡)
#define POSWAVE_TEST_SIGNAL_TYPE    3
// 位置梯形波上升段占**半周期**的比例 [0, 1] (仅 SIGNAL_TYPE = 3 时用)
//   1.0 → 上升段占满半周期, 没有平顶, 退化为三角波
//   0.0 → 上升段瞬时, 退化为阶跃 (目标在原点 ± 幅值之间跳变)
// 波形开始时从上升段中点起步 (波形值 0), 不会在开始瞬间跳变;
// 比例为 0 (阶跃) 时相位从 0 起, 波形一开始就直接跳到 +幅值
#define POSWAVE_TEST_TRAP_RISE      0.2f
// 启动保持 [s] (先只给原点并停在该位置, 到时才开始叠加波形;
// 原点到当前位置较远时要加大, 否则波形会在走到原点的过程中开始)
#define POSWAVE_TEST_STANDBY_DUR    1.0f
// 幅值上限 [指令单位] (绝对值; = 2 电机转 @ 10000 指令单位/转)
#define POSWAVE_TEST_AMP_MAX        20000.0f
// 切换 ExperimentType 时写入 SigAmp 的幅值初值 [指令单位]
#define POSWAVE_TEST_AMPLITUDE      500.0f
// 切换 ExperimentType 时写入 SigFreq 的频率初值 [Hz]
#define POSWAVE_TEST_FREQ           1.0f
// 自动结束: 波形重复多少个完整周期后结束 (0 = 不限, 由 Enable 释放停止)
// 周期按相位回绕计数, 运行时改频率不影响计数准确性 (每个周期都算一个)
#define POSWAVE_TEST_CYCLES         40
// 结束/停止时: 目标位置直接给基准位置 (原点), 保持多长时间后禁用驱动器 [s]
// 不做幅值淡出, 到点后直接关停
#define POSWAVE_TEST_HOLD_DUR       1.0f

// ---------------- 阶跃 (StepResponse) ----------------
// 0 速保持 [s]
#define STEP_TEST_ZERO_DUR      1.0f
// 高电平保持 [s]
#define STEP_TEST_SPD_DUR       0.5f
// 低电平保持 [s]
#define STEP_TEST_RETURN_DUR    0.5f
// 阶跃中心 [rpm]
#define STEP_TEST_BASE_SPD      0.0f
// 阶跃幅值 [rpm]
#define STEP_TEST_AMP_SPD       500.0f

// ---------------- 负载侧扰动转矩扫描 (LoadDisturbScan) ----------------
// 驱动侧载波中心速度 [rpm]
#define LOADSCAN_TEST_BASE_SPD      900.0f
// 驱动侧载波幅值 [rpm]
#define LOADSCAN_TEST_AMP_SPD       0.0f
// 驱动侧载波频率 [Hz]
#define LOADSCAN_TEST_WAVE_FREQ     4.0f
// 启动 0 速保持 [s]
#define LOADSCAN_TEST_STANDBY_DUR   1.0f
// 扰动幅度下限 [% 负载额定转矩]
#define LOADSCAN_TEST_AMP_MIN_PCT   40.0f
// 扰动幅度上限 [% 负载额定转矩]
#define LOADSCAN_TEST_AMP_MAX_PCT   100.0f
// 每频率的幅度步数
#define LOADSCAN_TEST_AMP_STEPS     5
// 每 (频率, 幅度) 段最小驻留 [s]
#define LOADSCAN_TEST_MIN_DUR       0.375f
// 每 (频率, 幅度) 段最小完整周期数
#define LOADSCAN_TEST_MIN_CYCLES    2
// 段间间隔 [s]
#define LOADSCAN_TEST_GAP_DUR       0.375f
// 频点数 (与 LoadScanFreqTable 一致)
#define LOADSCAN_TEST_NUM_FREQS     23

// ---------------- 速度闭环 PRBS (VelocityClosedLoopPRBS) ----------------
// 伺服工作在 CSV(9), 用伺服自带速度环, 只输出激励序列 (不做任何在线频响计算):
//   速度指令 = SigOffset (基础速度) + SigAmp × PRBS(±1)
// 时序: 0 速保持 → 斜坡到基础速度 → 静置 → PRBS 段 → 斜坡停机。
// 频响由上位机用录波数据 (SpdCmd / SpdFb) 离线做 FFT。
//
// 离线分析要用的两件事:
//   1) 位序列是确定的: LFSR 从全 1 起, Galois 掩码见 PRBS_TEST_LFSR_MASK,
//      可以按同样的算法在 PC 上重放出逐 bit 的 ±1。
//   2) 一个 bit 的时长 = 1/SigFreq [s], 实际边沿量化到控制周期 dt,
//      所以第 n 个 bit 的起点是 round(n×dt_bit/dt)×dt。
// PRBS 功率谱是 sinc^2, 在 n×SigFreq 处有零点 —— 离线取频点时避开 1×、2×。
// 启动初值: 位速率 [Hz] → SigFreq
#define PRBS_TEST_FREQ              100.0f
// 启动初值: 幅值 [rpm] → SigAmp
#define PRBS_TEST_AMPLITUDE         100.0f
// 启动初值: 基础速度 [rpm] → SigOffset
#define PRBS_TEST_BASE_SPD          300.0f
// PRBS 序列位数 [-] (序列长度 = 2^n-1 bit, 周期 = (2^n-1)/SigFreq [s])
#define PRBS_TEST_LFSR_BITS         8
// PRBS 反馈掩码 (Galois 型, 与上面位数配套: 8 位 = x^8+x^6+x^5+x^4+1 = 0xB8)
#define PRBS_TEST_LFSR_MASK         0xB8u
// 启动 0 速保持 [s]
#define PRBS_TEST_STANDBY_DUR       1.0f
// 到达基础速度后静置多久再开始 PRBS [s] (等速度环进稳态, 否则首段被暂态污染)
#define PRBS_TEST_SETTLE_DUR        1.0f
// PRBS 段时长 [s] (0 = 一直跑到 Enable 释放)
#define PRBS_TEST_DUR               60.0f

// ---------------- 公共 ----------------
// 匀加速/减速斜率 [rpm/s]
#define TEST_RAMP_RATE                1200.0f
// 停机判定: 速度低于该值后禁用驱动器 [rpm]
#define TEST_RAMP_STOP_THRESH         5.0f
// Manual 模式基础速度 [rpm]
#define TEST_MANUAL_BASE_SPD    0.0f
// Manual 模式幅值 [rpm]
#define TEST_MANUAL_AMP_SPD     0.0f

// ---------------- Status.Sig* 初值 (切换 ExperimentType 时写入) ----------------
// 每个实验的注入信号口径见 SpeedTestSequencer.h:
//   SigFreq [Hz] / SigAmp / SigOffset, 单位随实验
// 上面对应的宏就是各实验的初值:
//   StepResponse    : 0 / STEP_TEST_AMP_SPD / STEP_TEST_BASE_SPD
//   WaveResponse    : WAVE_TEST_FREQ / WAVE_TEST_AMPLITUDE / WAVE_TEST_BASE_SPD
//   VelocitySweep   : 0 (回读) / SWEEP_TEST_AMPLITUDE / SWEEP_TEST_BASE_SPD
//   TorqueSweep     : 0 (回读) / TOR_SWEEP_TEST_AMPLITUDE / TOR_SWEEP_TEST_BASE_SPD
//   LoadDisturbScan : 0 (回读) / 0 (回读) / 负载侧基准转矩 0
//   PosLoopWave     : POSWAVE_TEST_FREQ / POSWAVE_TEST_AMPLITUDE / POSWAVE_TEST_ORIGIN
//   VelocityClosedLoopPRBS : PRBS_TEST_FREQ (位速率) / PRBS_TEST_AMPLITUDE / PRBS_TEST_BASE_SPD
//   Manual          : 0 / 0 / 负载侧基准转矩 0

#endif // TESTSCRIPTCONFIG_H_
