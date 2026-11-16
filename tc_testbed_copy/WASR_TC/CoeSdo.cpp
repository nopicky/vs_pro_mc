///////////////////////////////////////////////////////////////////////////////
// CoeSdo.cpp
// @brief  CoE SDO 工具实现 (ADS + ADSIGRP_CANOPEN_SDO)
//
// 分层:
//   Read/Write  只登记请求, 返回请求号 (可在任意上下文调用, 有锁保护)
//   Poll        由 RT 周期调用: 懒建端口 → CheckOrders → 超时 → 派发
//   AdsWriteCon/AdsReadCon  本工具自己 ADS 端口的确认, 只改请求槽
//   GetResult   取结果 (有锁保护)
//
// 超时/限频按 Poll 的拍数计, 任务周期只在第一次 Poll 时换算一次。
// 所有日志都在锁外打, 且控制台文本只用 ASCII。
///////////////////////////////////////////////////////////////////////////////

#include "TcPch.h"
#pragma hdrstop

#include "CoeSdo.h"

// 时间 [ms] → 控制周期数 (向上取整; ms = 0 或周期未知时返回 0 = 不限)
static ULONG CyclesFromMs(ULONG cycleNS, ULONG ms)
{
	ULONGLONG n;

	if (ms == 0 || cycleNS == 0) {
		return 0;
	}
	n = ((ULONGLONG)ms * 1000000ULL + (ULONGLONG)cycleNS - 1ULL) / (ULONGLONG)cycleNS;
	if (n < 1ULL) {
		n = 1ULL;
	}
	return (ULONG)n;
}

// 请求表锁的 RAII 作用域 (CSyncDefault 在 RT 下是关中断/全局锁, 只包几条语句)
class CoeSdoGuard
{
public:
	explicit CoeSdoGuard(CSyncDefault& lock)
	{
		m_pLock = &lock;
		m_pLock->Lock();
	}
	~CoeSdoGuard()
	{
		m_pLock->Unlock();
	}
private:
	CoeSdoGuard(const CoeSdoGuard&);
	CoeSdoGuard& operator=(const CoeSdoGuard&);
	CSyncDefault* m_pLock;
};

///////////////////////////////////////////////////////////////////////////////
// 构造: 只填默认值; ADS 端口/定时器都要等到第一次真正读写才动
///////////////////////////////////////////////////////////////////////////////
CoeSdo::CoeSdo()
{
	m_pTrace = NULL;
	m_portOpen = FALSE;
	m_cycleNS = 0;
	m_cyclesTimeout = 0;
	m_cyclesErrLog = 0;
	m_timingKnown = FALSE;
	m_nextReqId = 0;
	m_nextInvokeId = (ULONG)COE_SDO_INVOKEID_BASE;
	m_lastReject = COE_SDO_OK;
	m_errLogWait = 0;
	m_pending = 0;

	// 主站 NetId 默认取 CoeSdoConfig.h
	memset(&m_masterNetId, 0, sizeof(m_masterNetId));
	m_masterNetId.b[0] = (BYTE)COE_SDO_MASTER_NETID_B0;
	m_masterNetId.b[1] = (BYTE)COE_SDO_MASTER_NETID_B1;
	m_masterNetId.b[2] = (BYTE)COE_SDO_MASTER_NETID_B2;
	m_masterNetId.b[3] = (BYTE)COE_SDO_MASTER_NETID_B3;
	m_masterNetId.b[4] = (BYTE)COE_SDO_MASTER_NETID_B4;
	m_masterNetId.b[5] = (BYTE)COE_SDO_MASTER_NETID_B5;

	// 伺服序号 → 从站 EtherCAT 地址
	for (int i = 0; i < COE_SDO_SERVO_COUNT; i++) {
		m_servoAddr[i] = 0;
	}
#if (COE_SDO_SERVO_COUNT > 0)
	m_servoAddr[0] = (ULONG)COE_SDO_SLAVE_ADDR_0;
#endif
#if (COE_SDO_SERVO_COUNT > 1)
	m_servoAddr[1] = (ULONG)COE_SDO_SLAVE_ADDR_1;
#endif

	// 请求槽全部空闲
	for (int i = 0; i < COE_SDO_MAX_REQ; i++) {
		memset(&m_slot[i], 0, sizeof(m_slot[i]));
		m_slot[i].State = SLOT_FREE;
		m_slot[i].Status = COE_SDO_REJECT;
	}
}

///////////////////////////////////////////////////////////////////////////////
// 析构: 端口如果还开着就关掉 (正常路径是 SetObjStateSP → Shutdown 已经关过)
///////////////////////////////////////////////////////////////////////////////
CoeSdo::~CoeSdo()
{
	if (m_portOpen) {
		m_portOpen = FALSE;
		ShutdownAmsPort();
	}
}

///////////////////////////////////////////////////////////////////////////////
// 设置日志对象
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::SetTrace(CTcTrace* pTrace)
{
	m_pTrace = pTrace;
}

///////////////////////////////////////////////////////////////////////////////
// PREOP→SAFEOP: 只记服务器指针; 不建端口 (纯净启动)
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::SetServer(ITComObjectServer* ipSrv)
{
	m_spSrv = ipSrv;
}

///////////////////////////////////////////////////////////////////////////////
// SAFEOP→PREOP: 关端口并丢弃所有请求
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::Shutdown()
{
	BOOL wasOpen;

	{
		CoeSdoGuard guard(m_lock);
		wasOpen = m_portOpen;
		m_portOpen = FALSE;
		ResetSlotsLocked();
	}

	// 释放服务器指针、关端口都放在锁外 (可能要和 router/引用计数交互)
	m_spSrv = NULL;
	if (wasOpen) {
		ShutdownAmsPort();
		if (m_pTrace != NULL) {
			m_pTrace->Log(tlVerbose, "CoE: ADS port closed");
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// 设置伺服序号对应的从站 EtherCAT 地址 (初始化时调用, 不和读写并发)
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::SetServoAddr(int servo, ULONG slaveAddr)
{
	if (servo < 0 || servo >= COE_SDO_SERVO_COUNT) {
		return;
	}
	m_servoAddr[servo] = slaveAddr;
}

///////////////////////////////////////////////////////////////////////////////
// 读取伺服序号对应的从站 EtherCAT 地址
///////////////////////////////////////////////////////////////////////////////
ULONG CoeSdo::ServoAddr(int servo) const
{
	if (servo < 0 || servo >= COE_SDO_SERVO_COUNT) {
		return 0;
	}
	return m_servoAddr[servo];
}

///////////////////////////////////////////////////////////////////////////////
// 设置 EtherCAT 主站设备的 NetId (初始化时调用)
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::SetMasterNetId(const AmsNetId& netId)
{
	m_masterNetId = netId;
}

///////////////////////////////////////////////////////////////////////////////
// 提前建立 ADS 端口 (可选; 默认由 Poll 在第一次有请求时懒建)
///////////////////////////////////////////////////////////////////////////////
BOOL CoeSdo::Open()
{
	return EnsurePort();
}

///////////////////////////////////////////////////////////////////////////////
// 懒建 ADS 端口: 只有第一次真正提交读写后才走这里
// 调用方: Poll (单 RT 任务), 不需要额外加锁
///////////////////////////////////////////////////////////////////////////////
BOOL CoeSdo::EnsurePort()
{
	HRESULT hr;

	if (m_portOpen) {
		return TRUE;
	}
	if (m_spSrv == NULL) {
		return FALSE;
	}

	hr = InitAmsPort(m_spSrv, AMSPORT_USEDEFAULT);
	if (FAILED(hr)) {
		if (m_pTrace != NULL) {
			m_pTrace->Log(tlAlways, "CoE: InitAmsPort failed hr=0x%08X", (unsigned)hr);
		}
		return FALSE;
	}

	m_portOpen = TRUE;
	if (m_pTrace != NULL) {
		m_pTrace->Log(tlAlways, "CoE: ADS port %u created (lazy init on first request)",
			(unsigned)AmsGetPort());
	}
	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
// 提交读: 伺服序号 + CoE 对象
///////////////////////////////////////////////////////////////////////////////
ULONG CoeSdo::Read(int servo, USHORT index, UCHAR subIndex, ULONG dataLen)
{
	if (servo < 0 || servo >= COE_SDO_SERVO_COUNT) {
		m_lastReject = COE_SDO_REJECT;
		return 0;
	}
	return ReadAddr(m_servoAddr[servo], index, subIndex, dataLen);
}

///////////////////////////////////////////////////////////////////////////////
// 提交写: 伺服序号 + CoE 对象 + 原始值
///////////////////////////////////////////////////////////////////////////////
ULONG CoeSdo::Write(int servo, USHORT index, UCHAR subIndex, ULONG value, ULONG dataLen)
{
	if (servo < 0 || servo >= COE_SDO_SERVO_COUNT) {
		m_lastReject = COE_SDO_REJECT;
		return 0;
	}
	return WriteAddr(m_servoAddr[servo], index, subIndex, value, dataLen);
}

///////////////////////////////////////////////////////////////////////////////
// 按从站地址提交读
///////////////////////////////////////////////////////////////////////////////
ULONG CoeSdo::ReadAddr(ULONG slaveAddr, USHORT index, UCHAR subIndex, ULONG dataLen)
{
	return Submit(FALSE, slaveAddr, index, subIndex, 0, dataLen);
}

///////////////////////////////////////////////////////////////////////////////
// 按从站地址提交写
///////////////////////////////////////////////////////////////////////////////
ULONG CoeSdo::WriteAddr(ULONG slaveAddr, USHORT index, UCHAR subIndex, ULONG value, ULONG dataLen)
{
	return Submit(TRUE, slaveAddr, index, subIndex, value, dataLen);
}

///////////////////////////////////////////////////////////////////////////////
// 提交一个请求 (Read/Write 的公共实现): 只登记, 不发 ADS
// @param isWrite   TRUE = 写
// @param slaveAddr 从站 EtherCAT 地址
// @param index     CoE 主索引
// @param subIndex  CoE 子索引
// @param value     写入值 (读请求忽略)
// @param dataLen   数据长度 [字节] (1..4)
// @return 请求号 (>0); 0 = 失败 (见 LastReject)
///////////////////////////////////////////////////////////////////////////////
ULONG CoeSdo::Submit(BOOL isWrite, ULONG slaveAddr, USHORT index, UCHAR subIndex,
                     ULONG value, ULONG dataLen)
{
	CoeSdoGuard guard(m_lock);
	int i;
	Slot* pSlot;

	// 参数检查: CoE SDO 单次数据 1..4 字节, 从站地址不能为 0
	if (dataLen == 0 || dataLen > 4 || slaveAddr == 0 || slaveAddr > 0xFFFFUL) {
		m_lastReject = COE_SDO_REJECT;
		return 0;
	}

	i = AllocSlotLocked();
	if (i < 0) {
		m_lastReject = COE_SDO_REJECT;
		return 0;
	}

	pSlot = &m_slot[i];
	pSlot->State = SLOT_QUEUED;
	pSlot->IsWrite = isWrite;
	pSlot->ReqId = NewReqIdLocked();
	pSlot->InvokeId = NewInvokeIdLocked();
	pSlot->SlaveAddr = slaveAddr;
	pSlot->Index = index;
	pSlot->SubIndex = subIndex;
	pSlot->DataLen = dataLen;
	pSlot->Value = isWrite ? value : 0;
	pSlot->Wait = 0;
	pSlot->AdsResult = 0;
	pSlot->Status = COE_SDO_PENDING;
	pSlot->Logged = FALSE;
	m_pending++;

	m_lastReject = COE_SDO_OK;
	return pSlot->ReqId;
}

///////////////////////////////////////////////////////////////////////////////
// 每个控制周期调用一次
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::Poll(ITcTask* ipTask)
{
	Slot doneList[COE_SDO_MAX_REQ];
	int  doneCnt;
	int  i;

	// 任务周期 → 超时/限频拍数 (取到一次就不再用)
	if (!m_timingKnown) {
		ULONG cycleNS = 0;

		if (ipTask != NULL && SUCCEEDED(ipTask->GetCycleTime(&cycleNS)) && cycleNS > 0) {
			m_cycleNS = cycleNS;
			m_timingKnown = TRUE;
		} else if (m_cycleNS == 0) {
			// 取不到任务周期也先把超时算出来, 按 1 ms 估
			m_cycleNS = 1000000UL;
		}
		m_cyclesTimeout = CyclesFromMs(m_cycleNS, (ULONG)COE_SDO_TIMEOUT_MS);
		m_cyclesErrLog = CyclesFromMs(m_cycleNS, (ULONG)COE_SDO_ERR_LOG_MS);
	}

	// 错误日志限频计时 (只在 Poll 里改, 不需要锁)
	if (m_errLogWait > 0) {
		m_errLogWait--;
	}

	// 还没有 ADS 端口: 只有真的提交过读写才建 (没有请求就什么都不做)
	if (!m_portOpen) {
		if (m_pending == 0) {
			return;
		}
		if (!EnsurePort()) {
			// 端口建不起来: 让请求以失败结束, 调用方能拿到结果
			CoeSdoGuard guard(m_lock);
			FailPendingLocked(COE_SDO_REJECT, 0);
			return;
		}
	}

	// 处理本拍回来的 ADS 确认 (确认回调里自己拿锁)
	CheckOrders();

	doneCnt = 0;
	{
		CoeSdoGuard guard(m_lock);

		// 1) 在飞请求超时 → 记超时结果并释放该从站的在飞名额
		for (i = 0; i < COE_SDO_MAX_REQ; i++) {
			Slot* pSlot = &m_slot[i];
			if (pSlot->State != SLOT_WAIT || m_cyclesTimeout == 0) {
				continue;
			}
			pSlot->Wait++;
			if (pSlot->Wait > m_cyclesTimeout) {
				FinishLocked(*pSlot, COE_SDO_TIMEOUT, 0, 0, 0);
			}
		}

		// 2) 派发排队请求: 同一个从站同时只允许一个在飞, 不同从站可并行
		for (i = 0; i < COE_SDO_MAX_REQ; i++) {
			Slot* pSlot = &m_slot[i];
			if (pSlot->State != SLOT_QUEUED) {
				continue;
			}
			if (SlaveInFlightLocked(pSlot->SlaveAddr)) {
				continue;
			}
			SendLocked(*pSlot);
		}

		// 3) 收集本拍完成且还没打日志的请求 (锁内不调 Trace, 出锁再打)
		for (i = 0; i < COE_SDO_MAX_REQ; i++) {
			Slot* pSlot = &m_slot[i];
			if (pSlot->State != SLOT_DONE || pSlot->Logged) {
				continue;
			}
			pSlot->Logged = TRUE;
			doneList[doneCnt] = *pSlot;
			doneCnt++;
		}
	}

	// 打完成日志
	for (i = 0; i < doneCnt; i++) {
		const Slot* pSlot = &doneList[i];

		if (m_pTrace == NULL) {
			break;
		}
		if (pSlot->Status == COE_SDO_OK) {
			// 成功只打 verbose (默认 TraceLevelMax = tlAlways 时不输出)
			if (pSlot->IsWrite) {
				m_pTrace->Log(tlVerbose, "CoE: write 0x%04X:%02X @%u = %u -> OK",
					(unsigned)pSlot->Index, (unsigned)pSlot->SubIndex,
					(unsigned)pSlot->SlaveAddr, (unsigned)pSlot->Value);
			} else {
				m_pTrace->Log(tlVerbose, "CoE: read 0x%04X:%02X @%u -> OK value %u (%u bytes)",
					(unsigned)pSlot->Index, (unsigned)pSlot->SubIndex,
					(unsigned)pSlot->SlaveAddr, (unsigned)pSlot->Value,
					(unsigned)pSlot->DataLen);
			}
			continue;
		}

		// 错误/超时: 限频 (每 COE_SDO_ERR_LOG_MS 最多一行), 其余只计数
		if (m_errLogWait > 0) {
			continue;
		}
		m_errLogWait = (m_cyclesErrLog > 0) ? m_cyclesErrLog : 1UL;
		if (pSlot->Status == COE_SDO_TIMEOUT) {
			m_pTrace->Log(tlAlways, "CoE: %s 0x%04X:%02X @%u -> TIMEOUT after %u ms",
				pSlot->IsWrite ? "write" : "read",
				(unsigned)pSlot->Index, (unsigned)pSlot->SubIndex,
				(unsigned)pSlot->SlaveAddr, (unsigned)COE_SDO_TIMEOUT_MS);
		} else {
			m_pTrace->Log(tlAlways, "CoE: %s 0x%04X:%02X @%u -> FAIL 0x%08X",
				pSlot->IsWrite ? "write" : "read",
				(unsigned)pSlot->Index, (unsigned)pSlot->SubIndex,
				(unsigned)pSlot->SlaveAddr, (unsigned)pSlot->AdsResult);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// 取请求结果 (非阻塞, 可重复调用)
///////////////////////////////////////////////////////////////////////////////
BOOL CoeSdo::GetResult(ULONG reqId, CoeSdoResult* pResult)
{
	BOOL found = FALSE;
	int i;

	if (pResult == NULL || reqId == 0) {
		return FALSE;
	}

	{
		CoeSdoGuard guard(m_lock);

		for (i = 0; i < COE_SDO_MAX_REQ; i++) {
			Slot* pSlot = &m_slot[i];
			if (pSlot->State == SLOT_FREE || pSlot->ReqId != reqId) {
				continue;
			}
			if (pSlot->State != SLOT_DONE) {
				// 还在排队/等确认
				break;
			}
			pResult->ReqId = pSlot->ReqId;
			pResult->SlaveAddr = pSlot->SlaveAddr;
			pResult->Index = pSlot->Index;
			pResult->SubIndex = pSlot->SubIndex;
			pResult->IsWrite = pSlot->IsWrite;
			pResult->Status = pSlot->Status;
			pResult->AdsResult = pSlot->AdsResult;
			pResult->DataLen = pSlot->DataLen;
			pResult->Value = pSlot->Value;
			// 取走结果: 释放请求槽 (同一个请求号只成功一次)
			pSlot->State = SLOT_FREE;
			found = TRUE;
			break;
		}
	}

	return found;
}

///////////////////////////////////////////////////////////////////////////////
// 是否还有请求没完成
///////////////////////////////////////////////////////////////////////////////
BOOL CoeSdo::IsBusy()
{
	return (m_pending > 0) ? TRUE : FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// 分配请求槽: 先找空闲槽, 没有就回收最老的已完成槽 (结果还没被取走)
// @return 槽下标; -1 = 没有可用槽
///////////////////////////////////////////////////////////////////////////////
int CoeSdo::AllocSlotLocked()
{
	int i;
	int oldest = -1;
	ULONG oldestReq = 0xFFFFFFFFUL;

	for (i = 0; i < COE_SDO_MAX_REQ; i++) {
		if (m_slot[i].State == SLOT_FREE) {
			return i;
		}
	}

	// 全满: 回收最老的已完成请求 (只回收 DONE, 不会丢还没完成的请求)
	for (i = 0; i < COE_SDO_MAX_REQ; i++) {
		if (m_slot[i].State == SLOT_DONE && m_slot[i].ReqId < oldestReq) {
			oldestReq = m_slot[i].ReqId;
			oldest = i;
		}
	}
	return oldest;
}

///////////////////////////////////////////////////////////////////////////////
// 重置所有请求槽
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::ResetSlotsLocked()
{
	int i;

	m_pending = 0;
	for (i = 0; i < COE_SDO_MAX_REQ; i++) {
		m_slot[i].State = SLOT_FREE;
		m_slot[i].Status = COE_SDO_REJECT;
		m_slot[i].Logged = TRUE;
		m_slot[i].Wait = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////
// 让所有排队/在飞请求以失败结束 (端口建不起来时用)
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::FailPendingLocked(CoeSdoStatus status, ULONG adsResult)
{
	int i;

	for (i = 0; i < COE_SDO_MAX_REQ; i++) {
		Slot* pSlot = &m_slot[i];
		if (pSlot->State == SLOT_QUEUED || pSlot->State == SLOT_WAIT) {
			FinishLocked(*pSlot, status, adsResult, 0, 0);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// 该从站是否有请求在飞
///////////////////////////////////////////////////////////////////////////////
BOOL CoeSdo::SlaveInFlightLocked(ULONG slaveAddr) const
{
	int i;

	for (i = 0; i < COE_SDO_MAX_REQ; i++) {
		if (m_slot[i].State == SLOT_WAIT && m_slot[i].SlaveAddr == slaveAddr) {
			return TRUE;
		}
	}
	return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// 发一个排队请求 (ADS 异步请求; 数据缓冲是请求槽里的 Value, 确认前一直有效)
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::SendLocked(Slot& s)
{
	AmsAddr target;
	ULONG indexOffset;
	int   res;

	// 目标 = 主站 NetId + 该从站固定 EtherCAT 地址
	memset(&target, 0, sizeof(target));
	target.netId = m_masterNetId;
	target.port = (AmsPortNo)s.SlaveAddr;

	// IndexOffset = (index << 16) | subIndex
	indexOffset = ((ULONG)s.Index << 16) | (ULONG)s.SubIndex;

	if (s.IsWrite) {
		// 写: 低 dataLen 字节有效 (小端)
		res = AdsWriteReq(target, s.InvokeId, ADSIGRP_CANOPEN_SDO, indexOffset,
		                  s.DataLen, &s.Value);
	} else {
		// 读: dataLen = 期望的数据长度, 实际长度由确认回调给出
		res = AdsReadReq(target, s.InvokeId, ADSIGRP_CANOPEN_SDO, indexOffset, s.DataLen);
	}

	if (res != ADSERR_NOERR) {
		// 同步失败 (端口不对/目标不可达等), 直接出结果, 不占在飞名额
		FinishLocked(s, COE_SDO_ADSERR, (ULONG)res, 0, 0);
		return;
	}

	// 等确认
	s.State = SLOT_WAIT;
	s.Wait = 0;
}

///////////////////////////////////////////////////////////////////////////////
// 请求完成: 记录状态/数据并置为 SLOT_DONE
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::FinishLocked(Slot& s, CoeSdoStatus status, ULONG adsResult,
                          ULONG dataLen, ULONG value)
{
	// 只有还没完成的请求才减未完成计数 (同一个槽不会完成两次)
	if (s.State == SLOT_QUEUED || s.State == SLOT_WAIT) {
		if (m_pending > 0) {
			m_pending--;
		}
	}

	s.Status = status;
	s.AdsResult = adsResult;
	s.DataLen = dataLen;
	s.Value = value;
	s.Wait = 0;
	s.State = SLOT_DONE;
}

///////////////////////////////////////////////////////////////////////////////
// 生成请求号 (从 1 开始, 0 保留为"无效")
///////////////////////////////////////////////////////////////////////////////
ULONG CoeSdo::NewReqIdLocked()
{
	m_nextReqId++;
	if (m_nextReqId == 0) {
		m_nextReqId = 1;
	}
	return m_nextReqId;
}

///////////////////////////////////////////////////////////////////////////////
// 生成 invokeId (每个请求唯一, 迟到的确认不会误认)
///////////////////////////////////////////////////////////////////////////////
ULONG CoeSdo::NewInvokeIdLocked()
{
	m_nextInvokeId++;
	if (m_nextInvokeId == 0) {
		m_nextInvokeId = 1;
	}
	return m_nextInvokeId;
}

///////////////////////////////////////////////////////////////////////////////
// ADS 写确认: 只处理本工具自己的 invokeId
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::AdsWriteCon(ADS_CONST AmsAddr& rAddr, ULONG invokeId, ULONG nResult)
{
	int i;

	(void)rAddr;

	{
		CoeSdoGuard guard(m_lock);

		i = FindSlotByInvoke(invokeId);
		if (i < 0 || !m_slot[i].IsWrite) {
			// 不是本工具在等的写 (超时后迟到的确认等), 忽略
			return;
		}

		if (nResult == ADSERR_NOERR) {
			FinishLocked(m_slot[i], COE_SDO_OK, 0, m_slot[i].DataLen, m_slot[i].Value);
		} else {
			FinishLocked(m_slot[i], COE_SDO_ADSERR, nResult, 0, 0);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// ADS 读确认: 保存读回值 (最多 4 字节, 小端)
///////////////////////////////////////////////////////////////////////////////
void CoeSdo::AdsReadCon(ADS_CONST AmsAddr& rAddr, ULONG invokeId, ULONG nResult,
                        ULONG cbLength, PVOID pData)
{
	int i;

	(void)rAddr;

	{
		CoeSdoGuard guard(m_lock);

		i = FindSlotByInvoke(invokeId);
		if (i < 0 || m_slot[i].IsWrite) {
			return;
		}

		if (nResult != ADSERR_NOERR) {
			FinishLocked(m_slot[i], COE_SDO_ADSERR, nResult, 0, 0);
			return;
		}

		// pData 只在本次回调内有效 → 立刻按小端拷进 ULONG
		{
			ULONG value = 0;
			ULONG len = cbLength;

			if (len > sizeof(value)) {
				len = (ULONG)sizeof(value);
			}
			if (pData != NULL && len > 0) {
				memcpy(&value, pData, (size_t)len);
			}
			FinishLocked(m_slot[i], COE_SDO_OK, 0, len, value);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// 按 invokeId 找正在等待确认的槽
// @return 槽下标; -1 = 没有
///////////////////////////////////////////////////////////////////////////////
int CoeSdo::FindSlotByInvoke(ULONG invokeId) const
{
	int i;

	for (i = 0; i < COE_SDO_MAX_REQ; i++) {
		if (m_slot[i].State == SLOT_WAIT && m_slot[i].InvokeId == invokeId) {
			return i;
		}
	}
	return -1;
}
