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
} FEED_CONFIG_T;

/* 全局工艺参数实例
   取值范围：见各字段注释
   存取者：CFeedSequencer::init 读取并做快照；调试期可经 ADS 在线修改
   注意：在线修改后需重新进入实验（Enable 上升沿重新 init）才生效 */
extern FEED_CONFIG_T g_cfgFeed;

#endif /* FEED_CONFIG_H__20261117 */
