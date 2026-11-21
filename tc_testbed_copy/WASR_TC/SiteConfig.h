///////////////////////////////////////////////////////////////////////////////
// SiteConfig.h
// @brief  站点表: 「哪个槽位 (Servo0..Servo7) 接什么、配什么电机」。
//
// 设计意图 (为什么不是 #if 分叉):
//   本机单轴 / 对拖 4 轴 两种形态, 差异全在"哪个槽位接什么、接几台",
//   逻辑完全一样。用一张表描述, 换机台只换 profile, 代码一行不改。
//   工程里各槽位已固定链接到具体从站, 所以切场景 = 启用/禁用从站 + 选 profile。
//
// 运行期选择: Status.SiteProfile (只能在 Enable = 0 且所有轴回到 IDLE 时改),
//   模块把选中的 profile 应用到 8 个槽位, 并把结果回写到
//   Status.AxisRole0..7 / Status.AxisCount 供现场核对。
//
// 槽位角色:
//   SLOT_ROLE_NONE  = 该槽位不接物理从站 -> 模块完全不碰它 (输出全 0, 不跑 CiA402)。
//                     未链接的槽位输入恒为 0, 若仍去跑状态机会每拍重试使能、
//                     状态乱跳, 所以"没接"必须显式声明为 NONE。
//   SLOT_ROLE_DRIVE = 送纸驱动轴 (CSP, 跟凸轮走), 按槽位顺序 = 轴0..轴3
//   SLOT_ROLE_LOAD  = 对拖负载轴 (CST 恒转矩), 不影响凸轮相位
//
// 新增机台形态: 在 SiteConfig.cpp 的表里加一条 profile 即可。
///////////////////////////////////////////////////////////////////////////////
#ifndef SITECONFIG_H_
#define SITECONFIG_H_

#include "MotorSpec.h"

/* 槽位总数 (Servo0..Servo7, 对应 IO 结构的 8 个槽) */
#define SITE_SLOT_MAX   8
/* 送纸驱动轴上限 (凸轮最多 4 个轴) */
#define SITE_AXIS_MAX   4

/* 槽位角色 */
#define SLOT_ROLE_NONE     0
#define SLOT_ROLE_DRIVE    1
#define SLOT_ROLE_LOAD     2

/* 驱动器监视 (SDO 惯量比/刚性等级) 对象口径 */
#define SLOT_MON_WECON     0   /* 维控: 0x2003:01 惯量比 / 0x2003:02 刚性 */
#define SLOT_MON_INOVANCE  1   /* 汇川: 0x2008:10 惯量比 / 0x2009:02 刚性 */

/* 槽位配置 */
typedef struct _SITE_SLOT_T
{
    unsigned char  ucRole;       /* SLOT_ROLE_xxx */
    unsigned char  ucMotor;      /* MOTOR_xxx 编号 (MotorSpec.h) */
    unsigned char  ucMon;        /* SLOT_MON_xxx 监视对象口径 */
    unsigned long  ulSlaveAddr;  /* EtherCAT 从站地址 (XAE 从站页的 EtherCAT Addr, SDO 用) */
} SITE_SLOT_T;

/* 站点形态 */
typedef struct _SITE_PROFILE_T
{
    const char* pcName;                       /* 形态名 (日志/调试用) */
    double      afMmPerMotorRev[SITE_AXIS_MAX]; /* 各驱动轴: 电机每转的工艺 mm */
    SITE_SLOT_T aSlot[SITE_SLOT_MAX];         /* 8 个槽位的角色/电机/监视口径 */
} SITE_PROFILE_T;

/* 站点形态编号 (Status.SiteProfile 填这个) */
#define SITE_PROFILE_LOCAL_1AX    0   /* 本机: 单轴 VD5L (Servo0) */
#define SITE_PROFILE_DRAG_4AX     1   /* 对拖: 4 个 VD3E 驱动轴 (Servo1..4) */
#define SITE_PROFILE_DRAG_VD5E    2   /* 对拖: 4 个 VD5E 驱动轴 23bit (Servo1..4), 3E 保留作负载 */
#define SITE_PROFILE_COUNT        3

/* 工程里的链接是固定映射（一次接好, 切场景只改从站启停 + SiteProfile）:
     Servo0 = 维控VD5L   Servo1~4 = 维控VD3E_1~4   Servo5~7 留空(汇川暂不接)
   所以「同一槽位换一台驱动器」是做不到的 —— TwinCAT 的 PDO 链接是静态 1:1,
   一个模块变量只能指向一台从站。要让某槽位换机, 必须改链接(tsproj)。

   4 驱动 + 4 负载 (8 台 VD3E) 暂不支持: 8 台 VD3E 已占满 8 槽, 而 Servo0 还要
   留给 VD5L -> 总共要 9 槽。真要做时先把 IO 结构扩到 9~12 槽 (WASR_TC.tmc
   升版本), 再在这里加一条 profile。 */

/* 站点形态表 */
extern const SITE_PROFILE_T g_aSiteProfile[SITE_PROFILE_COUNT];

/* 取站点形态 (越界回退到 SITE_PROFILE_LOCAL_1AX)
   @param ucProfile SITE_PROFILE_xxx
   @return 形态指针 (不为 NULL) */
const SITE_PROFILE_T* SiteProfileGet(unsigned char ucProfile);

#endif // SITECONFIG_H_
