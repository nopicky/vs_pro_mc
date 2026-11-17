#ifndef FEED_CONFIG_H__20261117
#define FEED_CONFIG_H__20261117

/* 送纸工艺/机械参数：调参唯一入口，只改这里
   注意：编码器位数与「每转指令单位数」由 ServoPlatformConfig.h 电机分支
   (WASR_S0_MOTOR 宏) 决定；mm→指令单位换算由挂载层按驱动侧齿轮比自动计算 */
typedef struct tagFEED_CONFIG_T {
    int    nAxisCount;          /* 启用轴数 1~4：默认 1（单轴 VD5L 先行），借齐伺服后改 4 */
    double fFeedRollerCircum;   /* 送纸轴周长 mm，默认 195.349 */
    double fMasterCircum;       /* 主轴(印刷滚筒)周长 mm，默认 1188.78 */
    double fRollerPitch;        /* 轮间距 mm，默认 90（相邻轴同步段长度差） */
    double fMmPerMotorRev;      /* 电机每转对应的送纸工艺 mm（减速比折算），默认 195.349（直连 1:1） */
    double afSyncLen[4];        /* 各轴同步段长 mm：{430, 340, 250, 160} = A1~A4 */
    double afOvertravel[4];     /* 各轴过送补偿 mm：{10,0,0,0}（科威达工艺：轴1 +10 使纸尾脱离） */
    double fStartPhaseComp;     /* 启动相位补偿 mm：整圈对齐基础上的手动微调，默认 0 */
    double fMaxLineSpeedMMin;   /* 线速度上限 m/min，默认 350 */
    bool   bLoadSideEnable;     /* 负载侧(CST)使能开关：false=不接负载伺服（默认，首测/实际机台） */
} FEED_CONFIG_T;

/* 全局工艺参数实例
   取值范围：见各字段注释
   存取者：CFeedSequencer::init 读取并做快照；调试期可经 ADS 在线修改
   注意：在线修改后需重新进入实验（Enable 上升沿重新 init）才生效 */
extern FEED_CONFIG_T g_cfgFeed;

#endif /* FEED_CONFIG_H__20261117 */
