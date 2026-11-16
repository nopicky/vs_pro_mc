///////////////////////////////////////////////////////////////////////////////
// CoeSdo.h
// @brief  CoE SDO 工具: 按伺服序号 + 对象索引读/写从站对象字典 (ADS + CoE SDO)。
//
// 设计目标
//   1) 函数化: Read/Write 给伺服序号和 index:subIndex 就发请求, 结果用
//      GetResult(请求号) 取; 不再有任何"固定通道"的宏。
//   2) 纯净启动: 构造/模块进入 SAFEOP 都不碰 ADS。第一次真正的 Read/Write
//      之后, 下一拍 Poll() 才建立本工具自己的 ADS 端口 (InitAmsPort)。
//      没读过/写过一次, 就没有端口、没有请求、没有初始化开销。
//   3) 异步 + 线程安全: ADS 请求天然异步 (确认在 AdsWriteCon/AdsReadCon 回来),
//      所以 Read/Write 只登记请求并返回请求号, 由 RT 周期里的 Poll() 推进:
//      处理确认 → 判超时 → 派发排队请求。请求表与计数器由 CSyncDefault 保护,
//      提交/取结果可以从别的上下文调用; Poll() 必须只有一个 RT 任务调用。
//
// 典型用法 (在模块的周期函数里):
//      // 第一拍: 提交 (返回请求号 > 0; 0 = 提交失败)
//      ULONG req = m_coeSdo.Write(1, 0x2008, 0x10, 300, 2);
//      // 之后每拍: 取结果 (TRUE = 完成, 填结果)
//      CoeSdoResult r;
//      if (m_coeSdo.GetResult(req, &r)) { ... r.Status / r.Value ... }
//
// 时序: 同一个从站同一时刻只允许一个请求在飞 (SDO 邮箱本来就是串行),
//       其余请求排队; 不同从站可以并行。超时按 Poll 的拍数算 (COE_SDO_TIMEOUT_MS)。
//
// 注意驱动器 EEPROM: SDO 写参数会进 EEPROM (VD5L 建议 4 小时/150 次,
//       SV680N 也有限制), 高频写之前先把驱动器参数存储改成"只进 RAM"。
//       读 (SDO upload) 不磨损 EEPROM, 随便读。
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "TcInterfaces.h"
#include "TcRtInterfaces.h"
#include "Ads.h"
#include "AdsR0.h"
#include "SyncObjects.h"

#include "CoeSdoConfig.h"

// 一次 SDO 请求的状态
typedef enum
{
	// 完成, 无错误
	COE_SDO_OK = 0,
	// 已提交: 排队中或等待 ADS 确认
	COE_SDO_PENDING = 1,
	// 等待 ADS 确认超时
	COE_SDO_TIMEOUT = 2,
	// ADS 返回错误 (见 CoeSdoResult::AdsResult)
	COE_SDO_ADSERR = 3,
	// 提交失败 (参数非法 / 请求表满 / ADS 端口不可用)
	COE_SDO_REJECT = 4
} CoeSdoStatus;

// 一次 SDO 请求的结果 (CoeSdo::GetResult 填写)
typedef struct
{
	// 请求号 (Read/Write 的返回值)
	ULONG ReqId;
	// 从站 EtherCAT 地址
	ULONG SlaveAddr;
	// CoE 主索引 / 子索引
	USHORT Index;
	UCHAR SubIndex;
	// TRUE = 写请求, FALSE = 读请求
	BOOL IsWrite;
	// 完成状态
	CoeSdoStatus Status;
	// ADS 返回码 (Status = COE_SDO_ADSERR 时有效)
	ULONG AdsResult;
	// 数据长度 [字节]: 写 = 请求长度, 读 = 实际返回长度
	ULONG DataLen;
	// 读回值 / 写入值 (小端, 最多 4 字节)
	ULONG Value;
} CoeSdoResult;

// CoE SDO 工具 (每个模块一个实例; 自带一个 ADS 端口, 首次读写时才建)
class CoeSdo : public CAdsR0FifoPort
{
public:
	CoeSdo();
	virtual ~CoeSdo();

	/**
	 * 设置日志对象
	 * @param pTrace 模块 Trace (可为 NULL = 不输出)
	 */
	void SetTrace(CTcTrace* pTrace);

	/**
	 * PREOP→SAFEOP: 只记下 TCom 服务器指针, 不建 ADS 端口
	 * @param ipSrv 模块的 TCom 服务器 (ITComObjectServer)
	 */
	void SetServer(ITComObjectServer* ipSrv);

	/**
	 * SAFEOP→PREOP: 关 ADS 端口并丢弃所有请求
	 */
	void Shutdown();

	/**
	 * 每个控制周期调用一次 (只能由同一个 RT 任务调用)
	 * 首次有请求时懒建 ADS 端口, 然后处理确认 / 判超时 / 派发排队请求
	 * @param ipTask 当前任务 (取任务周期换算超时拍数; 可为 NULL, 沿用上次)
	 */
	void Poll(ITcTask* ipTask);

	/**
	 * 提前建立 ADS 端口 (可选, 默认不用调: 第一次读写时 Poll 会自动建)
	 * 只有现场需要"进 OP 之前就准备好端口"时才用; 调了就不再是纯净启动。
	 * @return TRUE = 端口已就绪
	 */
	BOOL Open();

	/**
	 * 设置伺服序号对应的从站固定 EtherCAT 地址 (初始化时调用)
	 * @param servo     伺服序号 (0..COE_SDO_SERVO_COUNT-1)
	 * @param slaveAddr 从站 EtherCAT 地址 (= ADS 端口), 例 1001
	 */
	void SetServoAddr(int servo, ULONG slaveAddr);

	/**
	 * 读取伺服序号对应的从站 EtherCAT 地址
	 * @param servo 伺服序号
	 * @return 从站 EtherCAT 地址 (序号非法返回 0)
	 */
	ULONG ServoAddr(int servo) const;

	/**
	 * 设置 EtherCAT 主站设备的 NetId (默认取 CoeSdoConfig.h)
	 * @param netId 主站 NetId
	 */
	void SetMasterNetId(const AmsNetId& netId);

	/**
	 * 提交一次 SDO 读 (异步, 结果用 GetResult 取)
	 * @param servo    伺服序号 (0..COE_SDO_SERVO_COUNT-1)
	 * @param index    CoE 主索引, 例 0x2008
	 * @param subIndex CoE 子索引, 例 0x10
	 * @param dataLen  对象数据长度 [字节] (1..4; 例 0x2008:10 惯量比 = 2)
	 * @return 请求号 (>0); 0 = 提交失败 (见 LastReject)
	 */
	ULONG Read(int servo, USHORT index, UCHAR subIndex, ULONG dataLen = 2);

	/**
	 * 提交一次 SDO 写 (异步, 结果用 GetResult 取)
	 * @param servo    伺服序号 (0..COE_SDO_SERVO_COUNT-1)
	 * @param index    CoE 主索引
	 * @param subIndex CoE 子索引
	 * @param value    写入值 (原始值, 按 dataLen 截取低字节)
	 * @param dataLen  对象数据长度 [字节] (1..4)
	 * @return 请求号 (>0); 0 = 提交失败 (见 LastReject)
	 */
	ULONG Write(int servo, USHORT index, UCHAR subIndex, ULONG value, ULONG dataLen = 2);

	/**
	 * 直接按从站地址提交读 (不经过伺服序号表)
	 * @param slaveAddr 从站 EtherCAT 地址
	 * @param index     CoE 主索引
	 * @param subIndex  CoE 子索引
	 * @param dataLen   对象数据长度 [字节] (1..4)
	 * @return 请求号 (>0); 0 = 提交失败
	 */
	ULONG ReadAddr(ULONG slaveAddr, USHORT index, UCHAR subIndex, ULONG dataLen = 2);

	/**
	 * 直接按从站地址提交写 (不经过伺服序号表)
	 * @param slaveAddr 从站 EtherCAT 地址
	 * @param index     CoE 主索引
	 * @param subIndex  CoE 子索引
	 * @param value     写入值 (原始值)
	 * @param dataLen   对象数据长度 [字节] (1..4)
	 * @return 请求号 (>0); 0 = 提交失败
	 */
	ULONG WriteAddr(ULONG slaveAddr, USHORT index, UCHAR subIndex, ULONG value, ULONG dataLen = 2);

	/**
	 * 取请求结果 (非阻塞): 完成后取走结果并释放请求槽, 同一请求号只成功一次。
	 *
	 * 正常用法是"提交后每拍查一次, 返回 TRUE 就收工"。
	 * 请求表共 COE_SDO_MAX_REQ 项; 一直没取走的结果, 在表满时会被最老的新请求
	 * 回收, 那时本函数对该请求号返回 FALSE。
	 * @param reqId   Read/Write 返回的请求号
	 * @param pResult 输出: 请求结果
	 * @return TRUE = 已完成并填写 pResult (请求槽已释放); FALSE = 还没完成 / 已取走 / 未知
	 */
	BOOL GetResult(ULONG reqId, CoeSdoResult* pResult);

	/**
	 * 是否还有请求没完成 (排队或等待确认)
	 * @return TRUE = 有
	 */
	BOOL IsBusy();

	/**
	 * 最近一次 Read/Write 提交失败的原因
	 * @return COE_SDO_REJECT (失败) 或 COE_SDO_OK (成功)
	 */
	CoeSdoStatus LastReject() const { return m_lastReject; }

	/**
	 * ADS 端口是否已经建立 (懒初始化是否已发生)
	 * @return TRUE = 端口已建
	 */
	BOOL IsPortOpen() const { return m_portOpen ? TRUE : FALSE; }

	// 本工具自己 ADS 端口的写确认 (CAdsR0FifoPort → CAds)
	virtual void AdsWriteCon(ADS_CONST AmsAddr& rAddr, ULONG invokeId, ULONG nResult);
	// 本工具自己 ADS 端口的读确认
	virtual void AdsReadCon(ADS_CONST AmsAddr& rAddr, ULONG invokeId, ULONG nResult,
	                        ULONG cbLength, PVOID pData);

private:
	// 请求槽状态
	enum
	{
		SLOT_FREE = 0,
		SLOT_QUEUED = 1,
		SLOT_WAIT = 2,
		SLOT_DONE = 3
	};

	// 一个请求槽
	typedef struct
	{
		// 槽状态 (SLOT_*)
		int     State;
		// TRUE = 写请求
		BOOL    IsWrite;
		// 请求号 (从 1 开始)
		ULONG   ReqId;
		// ADS invokeId (每个请求唯一)
		ULONG   InvokeId;
		// 从站地址 + CoE 对象
		ULONG   SlaveAddr;
		USHORT  Index;
		UCHAR   SubIndex;
		// 数据长度 [字节]
		ULONG   DataLen;
		// 写入数据 (ADS 写请求的常驻缓冲) / 读回数据 (小端)
		ULONG   Value;
		// 已等待的周期数
		ULONG   Wait;
		// ADS 返回码
		ULONG   AdsResult;
		// 完成状态
		CoeSdoStatus Status;
		// 完成日志是否已打过 (Poll 出锁后补日志, 避免同一结果重复打)
		BOOL    Logged;
	} Slot;

	// 懒建 ADS 端口: 只在第一次真正有请求时调用
	BOOL EnsurePort();
	// 按 invokeId 找正在等待确认的槽
	int  FindSlotByInvoke(ULONG invokeId) const;
	// 分配一个槽: 先找空闲槽, 没有则回收最老的已完成槽
	int  AllocSlotLocked();
	// 重置所有槽
	void ResetSlotsLocked();
	// 让所有排队/在飞请求以失败结束
	void FailPendingLocked(CoeSdoStatus status, ULONG adsResult);
	// 该从站是否有请求在飞
	BOOL SlaveInFlightLocked(ULONG slaveAddr) const;
	// 发一个排队请求 (AdsWriteReq / AdsReadReq)
	void SendLocked(Slot& s);
	// 请求完成: 记录状态/数据并置为 SLOT_DONE
	void FinishLocked(Slot& s, CoeSdoStatus status, ULONG adsResult, ULONG dataLen, ULONG value);
	// 生成请求号
	ULONG NewReqIdLocked();
	// 生成 invokeId
	ULONG NewInvokeIdLocked();
	// 提交一个请求 (Read/Write 的公共实现)
	ULONG Submit(BOOL isWrite, ULONG slaveAddr, USHORT index, UCHAR subIndex,
	             ULONG value, ULONG dataLen);

	// 模块 Trace (可为 NULL)
	CTcTrace* m_pTrace;
	// 模块的 TCom 服务器 (InitAmsPort 用; 只在 SAFEOP..PREOP 之间有效)
	ITComObjectServerPtr m_spSrv;
	// ADS 端口是否已建立 (懒初始化)
	BOOL m_portOpen;
	// EtherCAT 主站设备 NetId
	AmsNetId m_masterNetId;
	// 伺服序号 → 从站 EtherCAT 地址
	ULONG m_servoAddr[COE_SDO_SERVO_COUNT];
	// 控制周期 [ns] 与超时/错误日志限频拍数 (0 = 还没换算)
	ULONG m_cycleNS;
	ULONG m_cyclesTimeout;
	ULONG m_cyclesErrLog;
	// 任务周期是否已经取到 (取到后不再调 GetCycleTime)
	BOOL  m_timingKnown;
	// 错误日志限频剩余拍数
	ULONG m_errLogWait;
	// 请求号 / invokeId 递增计数器
	ULONG m_nextReqId;
	ULONG m_nextInvokeId;
	// 最近一次提交失败原因
	CoeSdoStatus m_lastReject;
	// 请求槽表
	Slot m_slot[COE_SDO_MAX_REQ];
	// 未完成请求数 (排队 + 在飞; 只在锁内改, Poll 用它做无锁快速返回)
	volatile LONG m_pending;
	// 保护请求表与计数器 (ADS 确认回调与外部提交可能在不同上下文)
	CSyncDefault m_lock;
};
