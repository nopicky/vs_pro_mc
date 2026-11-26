#ifndef FEED_CONFIG_H__20261117
#define FEED_CONFIG_H__20261117

/* 送纸工艺/机械参数：调参唯一入口，只改这里
   注意：编码器位数 / 齿轮比 / 电机每转工艺 mm 现在由站点表 (SiteConfig.h) 与
   电机规格表 (MotorSpec.h) 决定；挂载层每拍把轴数与 mm→指令单位换算写进
   nAxisCount / fMmPerMotorRev（下面的默认值只作兜底），换机台改站点表即可
   标注「仅记录」的字段算法不读，只用于工艺核对与图纸对照，别指望改了就生效 */
typedef struct tagFEED_CONFIG_T {
    int    nAxisCount;          /* 启用轴数 1~4：挂载层按站点表的驱动轴数覆盖 */
    double fFeedRollerCircum;   /* 仅记录：送纸辊周长 mm（实际换算走站点表 per-axis） */
    double fMasterCircum;       /* 主轴(印刷滚筒)周长 mm，默认 1188.78：整圈对齐与相位回绕修正用它 */
    double fRollerPitch;        /* 仅记录：轮间距 mm，默认 90（相邻轴同步段长度差，手工核对用） */
    double fMmPerMotorRev;      /* 兼容标量：轴0 的电机每转工艺 mm，只在 init 里做合法性校验；
                                   轴级换算以站点表 afMmPerMotorRev[] 为准 */
    double fAccelX;             /* 加速段主轴行程 mm，默认 300（FAE 自选柔和版；威科达更短，实测扫参） */
    double fAccelY;             /* 加速段从轴行程 mm，默认 120（威科达参考：加速距离 100） */
    double fDecelX;             /* 减速段主轴行程 mm，默认 300 */
    double fDecelY;             /* 减速段从轴行程 mm，默认 120（威科达参考：减速距离 25） */
    double afSyncLen[4];        /* 各轴同步段长 mm：{430, 340, 250, 160} = A1~A4 */
    double afOvertravel[4];     /* 各轴过送补偿 mm：{10,0,0,0}（科威达工艺：轴1 +10 使纸尾脱离） */
    double fStartPhaseComp;     /* 启动相位补偿 mm：整圈对齐基础上的手动微调，默认 0 */
    double fMaxLineSpeedMMin;   /* 线速度上限 m/min，默认 400：也用作主轴速度估计的异常样本上限（1.5×） */
    bool   bContinuous;         /* 运行模式：false=单次触发（默认，调试）；true=连续循环
                                   （凸轮走完立即等主轴下一整圈自动续送，Start 撤销后当前循环走完即停） */
    bool   bLoadSideEnable;     /* 负载侧(CST)使能开关：false=不接负载伺服（默认，首测/实际机台） */
    double fLeadTimeMs;         /* 位置超前(相位提前)补偿 ms：0=关闭
                                   位置环滞后 T 使实际 ≈ 指令延迟 T，把指令按「轴速度×T」前移即可抵消
                                   350m/min 实测最优 ≈20.3ms
                                   ★ 用驱动器速度前馈(fVelFfGain>0)时必须归 0，否则双重补偿 → 反向超调 */
    double fLeadAccMs2;         /* 二阶(加速度)前馈 ms²：0=关闭（推荐起步 0.05~0.2）
                                   一阶超前在加速段会过冲（实测同步段入口略超前），
                                   理论上应按「T·v - T²·a」补偿，此项即 -T²·a 的系数 */
    double fVelFfGain;          /* 驱动器速度前馈增益 0~1.2（60B1h，单位 指令单位/s）
                                   1.0 = 满量：把轴速度原样送进驱动器位置环之后的求和点，
                                         位置环只修残差 → 稳态滞后理论上归零，不需要 fLeadTimeMs
                                   0.0 = 关闭（此时靠 fLeadTimeMs 指令超前，老方案）
                                   调参：0.2 验方向 → 0.5/0.8/1.0 找 Axis*IdealErr 过零点
                                   ★ 与 fLeadTimeMs 二选一，不能同时满量 */
    double fVelFfAccGainMs;     /* 速度前馈的加速度项系数 ms（方案 B，0=关闭）
                                   前馈量 = 速比·v_master + Ka·a  [mm/s]
                                   纯速度前馈(Ka=0)在加速段只补速度不补加速度，误差累积到
                                   约 10mm 后由位置环出力去追 → 进同步段冲过头。
                                   加这一项提前把「加速需要的额外速度」给出去，位置环就不用
                                   攒误差，过冲自然消失。
                                   量纲: a[mm/s²] × Ka[ms] × 1e-3 = mm/s
                                   理论起点 Ka ≈ Kv×Δerr/a ≈ 524/48400×1000 ≈ 10.8ms
                                   扫参: 0 → 3 → 5 → 8 → 12 → 15，看过冲与稳态误差的平衡 */
    double fMaxFollowErrMm;     /* 跟随偏差跳闸阈值 mm：|指令-实际| 超过它 → 自动撤 bFeedStart
                                   并停主轴（冻结相位），0 = 关闭保护。
                                   为什么必须有：虚拟主轴是纯积分器，电机跟不上时它照样
                                   按线速度往前跑，偏差无界发散（实测飞过 572km = 2929 圈）。
                                   只在 CAM_RUN 判，避免停机保持段误跳。
                                   粗调建议：先按 1/4 辊周长（195.349/4 ≈ 49）起步，
                                   正常带前馈跑稳态偏差应在个位数 mm，按实测收紧到 5~20 */

    /* ================= 方案 C：转矩前馈（60B2h Torque offset） ================= */
    double fTorFfGain;          /* 转矩前馈总增益 0~1.5（60B2h，单位 0.1% 额定转矩）
                                   0.0 = 关闭（默认）
                                   1.0 = 满量：T_ff = J_total × α 原样下发
                                   为什么需要它：纯速度前馈(60B1)只告诉驱动器「该跑多快」，
                                   但加速段真正缺的是「力」。位置环要攒到 ~10mm 误差才出力
                                   追得上，于是进同步段冲过头（实测 +415mm/s）。
                                   转矩前馈直接从转矩环补这份力，位置环不用攒误差。
                                   同步段曲率=0 → 该项自然为 0，不影响稳态精度。
                                   标定：0.3 验方向 → 0.7 / 1.0 看过冲是否压平
                                   ★ 与 MFC 大概率二选一（MFC 本质也是惯量前馈），开 MFC 时先归 0 */
    double fInertiaRatio[4];    /* 各轴负载惯量比 R = J_load / J_motor（0~10，无量纲）
                                   与驱动器 2003-01 显示值对应：显示 272 → 这里填 2.72
                                   J_total = J_motor(电机规格表) × (1 + R)
                                   实测 4 台：2.72 / 2.73 / 2.65 / 2.74（±2%，机械对称性良好）
                                   ★ 必须在对拖机械连好的状态下测（VD3E 可不使能，但要连着轴）
                                   ★ 换辊/改负载后重测；与初值差超过 15% 就重新标 J_total
                                   ★ Status.InertiaRatioN > 0 时本项被手填值覆盖；
                                     <=0 时用 SDO 2003:01 自动读回值（常开，2026-11-26） */

    int    nAutoStopCycles;     /* 自动停圈数（0 = 不限，默认）
                                   跑够 N 个**完整凸轮循环**后自动撤 bFeedStart，停在圈末。
                                   为什么按圈而不是按时间（2026-11-25 现场需求）：
                                     1) 手动停常停在半圈，每次采集长度都不一样，后期对比麻烦；
                                     2) 按圈停保证整数圈，采集长度 = N × 周期，Scope 窗口可提前定死；
                                     3) 位置行程被限在 N × 一圈位移内，顺带避开 23bit 位置空间
                                        （约 68 圈会把 607Ah 顶到 int32 上限）。
                                   实现：圈数在 CFeedSequencer 里数（唯一知道"一圈走完"的地方），
                                   判上限与撤启动位在 SpeedTestSequencer 里做 —— 那一层才能写回
                                   Status.bFeedStart（内部 m_bStart 每拍会被外部值覆盖，必须写回）。
                                   ★ 在线改立即生效：每拍从 g_cfgFeed 现读，不进 m_cfg 快照。 */
} FEED_CONFIG_T;

/* 目标位置(607Ah)可表示的上下限 [指令单位]
   ---- 为什么要有这个常量 (2026-11-23 踩到的坑) ----
   607Ah 是 int32 (LONG), 上限 2147483647。绝对位置是累积值, VD5E 用 23bit 编码器
   (8388608/转)时轻轻松松就到 2e9 量级。原来 PosToLong 里写死钳位 ±2,000,000,000,
   一旦实际位置越过 2e9, 目标位置就被钉死在 2e9 —— 表现为:
     电机一步不动 / 转矩只有 1% / 驱动器不报错 / 程序侧跟随偏差持续发散。
   实机现象: 6064 = 2000050025 (刚好超 50025 counts = 1.165mm), 607A 恒为 2e9。
   ★ 根治: 在驱动器侧把绝对位置清零(回零/清多圈), 让 6064 回到 0 附近;
     这个常量只是把可用的行程余量放大, 不能无限撑下去。
   2.1e9 距 int32 上限留 ~47M counts (≈1100mm @42941 unit/mm), 给四舍五入/前馈留余量 */
#define FEED_POS_LIMIT_UNIT   2.1e9

/* 跳闸原因回显 (送纸凸轮借用 Status.SigAmp, 该实验本就不用这个变量):
     0            = 未跳闸 (正常)
     正的小值     = 跟随偏差超限, 值 = 跳闸瞬间的偏差 [mm]
     FEED_TRIP_FLAG_POSLIMIT = 绝对位置(607Ah)顶到 int32 上限 → 必须去驱动器侧清零
   两者量级差 7 个数量级, 现场不会看错 */
#define FEED_TRIP_FLAG_POSLIMIT   999999.0f

/* 全局工艺参数实例
   取值范围：见各字段注释
   存取者：CFeedSequencer::init 读取并做快照；调试期可经 ADS 在线修改

   ---- 在线调参（2026-11-24）----
   调补偿参数不必再改代码重编译。Status 里开 bLiveTuning=1 后，下面这几项
   每拍从 Status 覆盖进来，立即生效（不需要重新 Enable）：
       fVelFfGain / fVelFfAccGainMs / fLeadAccMs2 / fMaxFollowErrMm / fLeadTimeMs
       fTorFfGain / fInertiaRatio[0..3]        ← 方案C（2026-11-24 加）
   bLiveTuning=0 时（默认）全部用本文件里的编译期初值，行为与原来完全一致。
   ★ fLeadTimeMs 在线改时做了每拍 0.05ms 的限速（见 CFeedSequencer::setLiveTuning），
     防止手滑输大值让位置指令阶跃（5833mm/s 下 1ms 就是 5.8mm 跳变）。
   ★ 凸轮参数（同步段长/超程/加减速段）和 fStartPhaseComp 不支持在线改：
     前者要重建凸轮对象，后者改的是相位原点，运动中改会直接错相位。 */
extern FEED_CONFIG_T g_cfgFeed;

#endif /* FEED_CONFIG_H__20261117 */
