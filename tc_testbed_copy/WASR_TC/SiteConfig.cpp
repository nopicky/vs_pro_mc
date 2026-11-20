///////////////////////////////////////////////////////////////////////////////
// SiteConfig.cpp
// @brief  站点表数据 (哪个槽位接什么、配什么电机)
//
// afMmPerMotorRev: 电机每转对应多少 mm 工艺位移 (减速比折算后)。
//   送纸辊直连 1:1 时 = 辊周长。台架用的 195.349 mm/转 与 FAE 工程
//   (131072000 : 195349) 一致; 现场若加减速机或换辊, 只改这里。
//
// ulSlaveAddr: EtherCAT 从站地址, 与 XAE 从站页的 "EtherCAT Addr" 一致,
//   只用于 CoE SDO 监视 (惯量比/刚性等级)。地址对不上只会读不到 (保留旧值),
//   不会影响运动; 但建议按 1001..1008 顺序分配, 便于对照。
#include "TcPch.h"
#pragma hdrstop

#include "SiteConfig.h"

// 送纸辊周长 [mm] (电机直连辊, 1:1)
#define SITE_MM_PER_REV_ROLLER   195.349

// 槽位缺省: 不接 (用于快速铺满一行)
#define SITE_SLOT_OFF(addr)   { SLOT_ROLE_NONE,  MOTOR_WE130M_17B, SLOT_MON_WECON, (addr) }
// 槽位: 送纸驱动轴 (VD3E/VD5L + WE130M 17bit)
#define SITE_SLOT_DRV(addr)   { SLOT_ROLE_DRIVE, MOTOR_WE130M_17B, SLOT_MON_WECON, (addr) }
// 槽位: 对拖负载轴
#define SITE_SLOT_LD(addr)    { SLOT_ROLE_LOAD,  MOTOR_WE130M_17B, SLOT_MON_WECON, (addr) }

// 站点形态表
//   槽位顺序 = IO 结构里 Servo0..Servo7, 与 XAE 里 PDO 链接顺序对应
const SITE_PROFILE_T g_aSiteProfile[SITE_PROFILE_COUNT] =
{
    // ---------------------------------------------------------------- 0
    // 本机单轴: Servo0 = VD5L + WE130M (17bit), 其余槽位不接
    {
        "Local_1Axis_VD5L",
        { SITE_MM_PER_REV_ROLLER, SITE_MM_PER_REV_ROLLER,
          SITE_MM_PER_REV_ROLLER, SITE_MM_PER_REV_ROLLER },
        {
            SITE_SLOT_DRV(1001),
            SITE_SLOT_OFF(1002),
            SITE_SLOT_OFF(1003),
            SITE_SLOT_OFF(1004),
            SITE_SLOT_OFF(1005),
            SITE_SLOT_OFF(1006),
            SITE_SLOT_OFF(1007),
            SITE_SLOT_OFF(1008)
        }
    },

    // ---------------------------------------------------------------- 1
    // 对拖 4 轴: Servo0..3 = 4 个 VD3E 驱动轴 (各带 WE130M 17bit),
    //             Servo4..7 负载槽预留 —— 负载电机还没接, 先声明 NONE,
    //             接上以后改选 SITE_PROFILE_DRAG_8AX 即可, 不用重编译。
    {
        "Drag_4Axis_VD3E",
        { SITE_MM_PER_REV_ROLLER, SITE_MM_PER_REV_ROLLER,
          SITE_MM_PER_REV_ROLLER, SITE_MM_PER_REV_ROLLER },
        {
            SITE_SLOT_DRV(1001),
            SITE_SLOT_DRV(1002),
            SITE_SLOT_DRV(1003),
            SITE_SLOT_DRV(1004),
            SITE_SLOT_OFF(1005),
            SITE_SLOT_OFF(1006),
            SITE_SLOT_OFF(1007),
            SITE_SLOT_OFF(1008)
        }
    },

    // ---------------------------------------------------------------- 2
    // 对拖 8 轴: Servo0..3 驱动 + Servo4..7 负载 (4 对全接齐)
    {
        "Drag_8Axis_VD3E",
        { SITE_MM_PER_REV_ROLLER, SITE_MM_PER_REV_ROLLER,
          SITE_MM_PER_REV_ROLLER, SITE_MM_PER_REV_ROLLER },
        {
            SITE_SLOT_DRV(1001),
            SITE_SLOT_DRV(1002),
            SITE_SLOT_DRV(1003),
            SITE_SLOT_DRV(1004),
            SITE_SLOT_LD(1005),
            SITE_SLOT_LD(1006),
            SITE_SLOT_LD(1007),
            SITE_SLOT_LD(1008)
        }
    }
};

///////////////////////////////////////////////////////////////////////////////
const SITE_PROFILE_T* SiteProfileGet(unsigned char ucProfile)
{
    if (ucProfile >= SITE_PROFILE_COUNT) {
        ucProfile = SITE_PROFILE_LOCAL_1AX;
    }
    return &g_aSiteProfile[ucProfile];
}
