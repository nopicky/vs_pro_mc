///////////////////////////////////////////////////////////////////////////////
// SiteConfig.cpp
// @brief  站点表数据 (哪个槽位接什么、配什么电机)
//
// afMmPerMotorRev: 电机每转对应多少 mm 工艺位移 (减速比折算后)。
//   送纸辊直连 1:1 时 = 辊周长。台架用的 195.349 mm/转 与 FAE 工程
//   (131072000 : 195349) 一致; 现场若加减速机或换辊, 只改这里。
//
// ulSlaveAddr: EtherCAT 从站地址, 只用于 CoE SDO 读惯量比/刚性等级。
//   工程里用的是自动地址(按拓扑分配), 所以实机启用集合下从 1001 起顺排:
//     只启用 VD5L      -> VD5L = 1001
//     只启用 4 台 VD3E -> VD3E_1~4 = 1001~1004
//   下表按这个填。地址对不上只会显示别的驱动器的值(不报错), 不影响运动;
//   激活后到从站页扫一眼实际地址即可确认。
#include "TcPch.h"
#pragma hdrstop

#include "SiteConfig.h"

// 送纸辊周长 [mm] (电机直连辊, 1:1)
#define SITE_MM_PER_REV_ROLLER   195.349

// 槽位缺省: 不接 (用于快速铺满一行)
#define SITE_SLOT_OFF(addr)   { SLOT_ROLE_NONE,  MOTOR_WE130M_17B, SLOT_MON_WECON, (addr) }
// 槽位: 送纸驱动轴, 可指定电机编号 (换电机或混机型时用)
//   例: SITE_SLOT_DRV_M(1003, MOTOR_WD80M_17B) —— 编码器位数/齿轮比/惯量/额定转矩
//   全部跟着 MotorSpec.h 走, 不用改逻辑
#define SITE_SLOT_DRV_M(addr, m)   { SLOT_ROLE_DRIVE, (m), SLOT_MON_WECON, (addr) }
// 槽位: 送纸驱动轴, 默认配 WE130M 17bit (VD5L / VD3E 现在带的都是它)
#define SITE_SLOT_DRV(addr)        SITE_SLOT_DRV_M(addr, MOTOR_WE130M_17B)
// 槽位: 送纸驱动轴, 配 VD5E 23bit 绝对值编码器电机 (换 VD5E 对拖时用)
#define SITE_SLOT_DRV23(addr)      SITE_SLOT_DRV_M(addr, MOTOR_WE130_23B)
// 槽位: 对拖负载轴 (CST 恒转矩)。当前没有 8 台 VD3E, 暂无 profile 用到, 保留给 4+4 形态
#define SITE_SLOT_LD(addr)         { SLOT_ROLE_LOAD,  MOTOR_WE130M_17B, SLOT_MON_WECON, (addr) }

// 站点形态表
//   槽位顺序 = IO 结构里 Servo0..Servo7, 与 XAE 里 PDO 链接顺序对应
//   工程里已按固定映射把链接一次接好（tsproj, 每槽 14 条）:
//     Servo0 = 维控VD5L   Servo1~4 = 维控VD3E_1~4   Servo5~7 留空
//   所以切场景只有两步: 启用对应从站/禁用其余的 -> 改 Status.SiteProfile, 不用再改链接
const SITE_PROFILE_T g_aSiteProfile[SITE_PROFILE_COUNT] =
{
    // ---------------------------------------------------------------- 0
    // 本机单轴: Servo0 = VD5L + WE130M (17bit), 其余槽位不接
    // 用法: 只启用 VD5L, 其它从站全部 Disabled
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
    // 对拖 4 轴: Servo1..4 = 4 个 VD3E 驱动轴 (各带 WE130M 17bit),
    //             Servo0 = VD5L 保持 NONE（单轴场景用), Servo5..7 留空
    // 用法: 启用 4 台 VD3E、禁用 VD5L
    // 注意: 轴号从槽位 1 开始 —— 轴0 = Servo1。上位机/Scope 看轴请用
    //       Status.Feed*0~3（按轴号）与 Status.AxisRole0~7（按槽位）核对
    {
        "Drag_4Axis_VD3E",
        { SITE_MM_PER_REV_ROLLER, SITE_MM_PER_REV_ROLLER,
          SITE_MM_PER_REV_ROLLER, SITE_MM_PER_REV_ROLLER },
        {
            SITE_SLOT_OFF(1001),
            SITE_SLOT_DRV(1001),
            SITE_SLOT_DRV(1002),
            SITE_SLOT_DRV(1003),
            SITE_SLOT_DRV(1004),
            SITE_SLOT_OFF(1006),
            SITE_SLOT_OFF(1007),
            SITE_SLOT_OFF(1008)
        }
    },

    // ---------------------------------------------------------------- 2
    // 对拖 4 轴: Servo1..4 = 4 个 VD5E 驱动轴 (各带 23bit 绝对值编码器电机),
    //             Servo0 = VD5L 保持 NONE, Servo5..7 留空
    // 用法: 启用 4 台 VD5E、禁用 VD5L 和 VD3E
    // 注意: 23bit → 每转 8388608 单位, 换算 feedMmToUnit = 8388608/195.349 ≈ 42946 单位/mm
    //       (17bit 是 671), 位置/速度前馈换算由 MotorSpec 表自动跟上, 无需改逻辑
    {
        "Drag_4Axis_VD5E_23B",
        { SITE_MM_PER_REV_ROLLER, SITE_MM_PER_REV_ROLLER,
          SITE_MM_PER_REV_ROLLER, SITE_MM_PER_REV_ROLLER },
        {
            SITE_SLOT_OFF(1001),
            SITE_SLOT_DRV23(1001),
            SITE_SLOT_DRV23(1002),
            SITE_SLOT_DRV23(1003),
            SITE_SLOT_DRV23(1004),
            SITE_SLOT_OFF(1006),
            SITE_SLOT_OFF(1007),
            SITE_SLOT_OFF(1008)
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
