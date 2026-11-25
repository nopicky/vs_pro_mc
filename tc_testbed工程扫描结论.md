# tc_testbed 工程扫描结论

> 扫描日期：2026-11-11
> 工程路径：`D:\work\w2026\vs_pro_mc\tc_testbed`
> 结论：这是一个**已跑通的对拖伺服测试台 C++ 实时模块**，不是送纸机 PLC 工程。

---

## 一、工程性质（一句话结论）

`tc_testbed` 是同事搭建的**对拖伺服测试平台**，用 TwinCAT 3 的 **C++ 实时模块**（非 PLC 工程）驱动两台伺服做频域/时域测试。它已经做到了"125 μs 周期 + EtherCAT + 两套伺服 + 多种实验"这个程度，是**方向 3（对拖平台）的现成基础**，但**不含送纸机工艺流程逻辑**。

---

## 二、目录结构

```
tc_testbed/
├── tc_testbed.sln           解决方案
├── tc_testbed.tsproj        TwinCAT 工程（周期/网卡/从站配置）
├── WASR_TC/                 C++ 实时模块（核心）
│   ├── WASR_TCMain.cpp      模块入口 / 周期任务
│   ├── WASR_TCDriver.*      驱动框架
│   ├── SpeedTestSequencer.* 实验序列器（核心状态机）
│   ├── MiniSpdLoop.h        上位机速度环（PI + 转矩滤波）
│   ├── CoeSdo.*             CoE SDO 读写工具
│   ├── ServoPlatformConfig.h 伺服/电机参数（编译期宏）
│   ├── WASR_ControlConfig.h 控制参数（转矩限幅、监视）
│   ├── TestScriptConfig.h   实验激励参数
│   ├── TestScanTables.h     扫频频点表
│   └── EncoderDebounce.h    编码器去抖
├── _Boot/                   已激活配置（说明工程曾被激活运行过）
└── _Repository/             已编译的 C++ 模块产物
```

`tc_measurement/`（同级的另一个目录）是 TwinCAT Measurement 工程，配套做 Scope 录波分析。

---

## 三、实时性配置（已达标）

| 项目 | 值 | 说明 |
|------|-----|------|
| 任务周期 | **125 μs** | `BaseTime="1250"`、`CycleTime="1250"`（单位 100 ns） |
| 实时核隔离 | 已配置 | `NonWinCpus="1"`、`PCoreAffinity="63"`、`ECoreAffinity="16320"` |
| 网卡 | **TwinCAT-Intel PCI Ethernet (Gigabit)** | 专用实时驱动网卡 |
| EtherCAT 主站 | `DcSyncTask="2"`、`DcSyncMode="3"` | 绑定 125 μs 任务，DC 同步 |
| 从站数量 | 4 段 EtherCAT 配置 | 总线上的从站 |
| DC 模式 | DC-Synchron | 从站间同步 |

**对照 125 μs 多轴同步目标**：硬件前提（Intel 网卡）、核隔离、DC 同步**全部已就位**。这说明同事已经把 125 μs 跑通的基础打好了。

---

## 四、伺服配置（对拖台架两套）

| 伺服 | 驱动器 | 电机 | 编码器 |
|------|--------|------|--------|
| Servo0 | 维控 VD5L | 默认 WD40M-01030S-D1（100W） | 23 bit |
| Servo1 | 汇川 SV630N（默认）/ SV680N | MS1H2-10C30CB（1.0kW） | 18 bit / 26 bit |

- 电机型号通过编译期宏 `WASR_S0_MOTOR` / `WASR_S1_MOTOR` 切换。
- 齿轮比约定写在 6091:01h/02h，与驱动器参数严格同步。
- 驱动/负载角色由 `WASR_EXPERIMENT_DRIVE_SIDE_SERVO` 指定。

---

## 五、已实现的实验能力（SpeedTestSequencer）

| 实验类型 | 说明 | 用途 |
|----------|------|------|
| StepResponse | 阶跃响应 | 时域动态特性 |
| WaveResponse | 定频波形（正弦/三角/梯形） | 频响测量 |
| VelocitySweep | 速度扫频（1~500 Hz） | 速度环频响 |
| TorqueSweep | 转矩扫频（10~2000 Hz） | 电流环/转矩环频响 |
| LoadDisturbScan | 负载侧扰动扫描 | **DOB/抗扰验证** |
| PosLoopWave | 位置环波形（CSP） | 位置环频响 |
| VelocityClosedLoopPRBS | 速度闭环 PRBS | 频响辨识 |

- 控制模式支持 CSV(9)/CST(10)/CSP(8) 切换（CiA 402）。
- `MiniSpdLoop` 提供了 CST 模式下的上位机速度环（PI + 转矩滤波 + 抗积分饱和）。
- `CoeSdo` 提供异步 CoE SDO 读写（读惯量比、刚性等级监视），已处理 EEPROM 磨损问题。

**关键结论**：这个测试台**已经具备了做 DOB / MFC 频响验证的完整实验手段**——扫频激励、扰动注入、PRBS 辨识、Scope 录波一应俱全。方向 3 的平台建设工作，很大程度已经被同事做完了。

---

## 六、与目标的差距

| 维度 | 现状 | 差距 |
|------|------|------|
| 125 μs 实时 | 已配好 | 无（已达标） |
| 对拖平台 | 已搭好（双伺服 + 实验序列器） | 无 |
| 频响/抗扰实验 | 已实现（扫频/扰动/PRBS） | 无 |
| **送纸机工艺流程** | **无** | **核心缺口** |
| 凸轮五次曲线算法 | 无（`SpeedTestSequencer` 只有波形/扫频，无凸轮） | 需新建 |
| AI 补偿算法 | 无 | 需新建 |
| CodeSys→TwinCAT 迁移 | 无 | 待 FAE 工程到位 |

---

## 七、建议的下一步

1. **先把这个测试台跑起来复现**：确认 125 μs + 双伺服 + 扫频在当前硬件上能复现，这是后续所有工作的基线。
2. **方向 3 基本可用**：对拖平台 + 实验序列器就是现成的验证工具，只需补"送纸工况模拟"（把印刷滚筒节拍映射成负载侧运动/转矩曲线）。
3. **方向 1 的新增工作**集中在：新建 PLC 工程（或扩展 C++ 模块）实现凸轮五次曲线 + 送纸工艺状态机，而不是从零搭测试台。
4. **等 FAE 的 CodeSys 工程**：拿到后按工艺流程做 TwinCAT 重建，本测试台的 `CoeSdo`、`MiniSpdLoop`、周期配置可直接复用。

---

## 八、风险提示

- `_Boot` 目录存在，说明工程曾被激活。用 junction 路径打开时 TwinCAT 激活配置写入 `_Boot` 可能异常，必要时复制为实体目录再打开。
- C++ 模块改参数需重新编译（`MiniSpdLoop`、`ServoPlatformConfig` 都是编译期宏）。
- 对拖/扫频时转矩限幅 `TORLIM=3000`（300% 额定）偏高，做 DOB 抗扰实验建议降到 2500 以下留裕量。
