///////////////////////////////////////////////////////////////////////////////
// CoeSdoConfig.h
// @brief  CoE SDO 工具 (CoeSdo.h) 的默认环境参数。
//
// 这里只放"和台架/网络有关"的常量: 主站 NetId、每台伺服的 EtherCAT 从站地址、
// 超时、请求表长度。要读写的对象 (index:subIndex) 与数值**全部由调用方在
// Read/Write 的参数里给**, 没有任何"固定通道"的宏, 换对象不用改本文件。
//
// 机制: AmsAddr     = { EtherCAT 主站设备 NetId, 从站固定 EtherCAT 地址 }
//       IndexGroup  = ADSIGRP_CANOPEN_SDO (0xF302)
//       IndexOffset = (index << 16) | subIndex
//       AdsWriteReq / AdsReadReq 异步发出, 结果在 AdsWriteCon / AdsReadCon 回调返回
//
// NetId 必须是 EtherCAT 主站设备的 NetId, 不能用模块本机 NetId (会返回 0x6
// ERR_TARGETPORTNOTFOUND)。从站地址就是 XAE 从站页的 "EtherCAT Addr", 不扫描。
///////////////////////////////////////////////////////////////////////////////
#pragma once

// EtherCAT 主站设备的 NetId (= tsproj 里 Device 的 AmsNetId, 当前 172.31.226.166.2.1)
// 运行中可用 CoeSdo::SetMasterNetId() 覆盖
#define COE_SDO_MASTER_NETID_B0     172
#define COE_SDO_MASTER_NETID_B1     31
#define COE_SDO_MASTER_NETID_B2     226
#define COE_SDO_MASTER_NETID_B3     166
#define COE_SDO_MASTER_NETID_B4     2
#define COE_SDO_MASTER_NETID_B5     1

// 伺服序号个数 (0 = Servo0, 1 = Servo1)
#define COE_SDO_SERVO_COUNT         2

// Servo0 = 维控 VD5L, EtherCAT 地址 1001
#define COE_SDO_SLAVE_ADDR_0        1001
// Servo1 = 汇川 SV680N / SV630N, EtherCAT 地址 1002
#define COE_SDO_SLAVE_ADDR_1        1002

// 单个请求等待 ADS 确认的超时 [ms]
#define COE_SDO_TIMEOUT_MS          3000

// 写失败/超时日志的最小间隔 [ms] (驱动器离线时不刷屏)
#define COE_SDO_ERR_LOG_MS          1000

// 请求表长度 (排队 + 在飞 + 已完成未取走 的总数)
#define COE_SDO_MAX_REQ             8

// ADS invokeId 基址 (每个请求在此基础上递增, 保证不被迟到的确认误认)
#define COE_SDO_INVOKEID_BASE       0x00001000
