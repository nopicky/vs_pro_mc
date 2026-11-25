# 驱动侧从 VD5L 切到 VD3E_1 —— 切换清单

> 结论先说：**只做「禁用 VD5L + 启用 VD3E_1」会失败**。电机大概率能进 OP、能显示使能，
> 但 `607A` 不动——因为模块的 IO 变量还全部链接在 VD5L 的 PDO 上，禁用从站**不会**自动改链接。
> 下面按「必须做 / 视情况做 / 千万别忘」三类列。

---

## 一、为什么禁用/启用不够（机制）

模块里驱动侧永远叫 `Servo0`，它的 14 个变量通过 **PDO 链接**指向某个物理从站：

```
Outputs^Servo0^Target_position  ──链接──►  维控VD5L^RxPDO^Target position
```

这套链接是**按从站名绑死的**。把 VD5L 设成 `Disabled` 只是停止它的数据交换，
链接文字仍然写着「维控VD5L」→ 模块写出去的值无人接收、读回来的是停更值（全 0 或旧值）。
于是：`6041 状态字 = 0` → 使能流程走不通；或 `6064` 不更新 → 模块算的指令基准是死的。

---

## 二、必须做（4 类，缺一不可）

### 1. 重链 14 条 PDO（最关键）

`Servo0` 与 VD5L 之间的 14 条链接，全部要改成 `维控VD3E_1^...`：

| 方向 | 变量（模块侧） | 从站侧（VD3E_1） |
|------|---------------|-----------------|
| 出 | `Outputs.Servo0.Controlword` | `RxPDO^ControlWord` |
| 出 | `Outputs.Servo0.Modes_of_operation` | `RxPDO^Modes of operation` |
| 出 | `Outputs.Servo0.Motor_revolutions` | `RxPDO^Motor revolutions` |
| 出 | `Outputs.Servo0.Shaft_revolutions` | `RxPDO^Shaft revolutions` |
| 出 | `Outputs.Servo0.Target_position` | `RxPDO^Target position` |
| 出 | `Outputs.Servo0.Target_velocity` | `RxPDO^Target velocity` |
| 出 | `Outputs.Servo0.Target_torque` | `RxPDO^Target torque` |
| 入 | `Inputs.Servo0.Status_word` | `TxPDO^statusWord` |
| 入 | `Inputs.Servo0.Position_actual_value` | `TxPDO^Position actual value` |
| 入 | `Inputs.Servo0.Velocity_actual_value` | `TxPDO^Velocity actual value` |
| 入 | `Inputs.Servo0.Torque_actual_value` | `TxPDO^Torque actual value` |
| 入 | `Inputs.Servo0.Torque_demand` | `TxPDO^Torque demand ` ← **结尾有空格** |
| 入 | `Inputs.Servo0.Error_code` | `TxPDO^Error Code` |
| 入 | `Inputs.Servo0.Following_error_actual_value` | `TxPDO^Following error actual value` |

> 手点容易漏，尤其 `Torque demand ` 那个尾空格。**这 14 行我可以直接改 tsproj**，比手点快且不会错。

### 2. 把 VD3E_1 分配到 EtherCAT 任务

我从工程里复制出来的 4 台 VD3E，tsproj 里**没有 `<SuTask>` 节点**（VD5L 有 `#x02010020`）。
没有任务分配 = PDO 不参与 125μs 周期交换 → 现象和"链接没改"一模一样。

在 XAE 里：双击从站 → EtherCAT 页 → 确认任务；或者我直接往 tsproj 补 `<SuTask>#x02010020</SuTask>`（与 VD5L 同值）。

### 3. 处理 VD5L 的「禁用 + 地址」

- **物理拔掉 VD5L 就必须设 Disabled**——否则主站找不到从站，整条总线进不了 OP，**VD3E 也一起挂**；
- 注意地址模式不同：VD5L 是**固定地址 1001**（`EtherCatAddr="1001"` `EtherCatAddrFixed="true"`），
  VD3E_1 是**自动地址（按拓扑位置）**。若物理接入位置与树中顺序不符，激活会报 `Slave not found` / `Invalid Working Counter`。
  → 解决：把 VD3E_1 也设成固定地址（推荐，和 VD5L 保持一致习惯），或主站 `Auto Config` 按实际拓扑重排。

### 4. 驱动器侧参数（VD3E 自己）

| 项 | 要求 | 不做的后果 |
|----|------|-----------|
| 6060 控制模式 | 由模块每拍写 8（CSP），无需手设 | — |
| 6091 齿轮比 | 由模块每拍写（1:1），VD3E 支持 6091 ✓ | 齿轮比不对 → 位移差倍数 |
| **位置环增益 / 刚性** | **必须重新整定** | **T 变了，20.3ms 超前失效**（见第四节） |
| 6065 跟随误差窗口 | VD5L 默认 524288；VD3E 可能更小 | 350 m/min 时报跟随误差脱机 |
| 60C2 插补周期 | ≤ 任务周期，建议 125μs 或 250μs | 位置指令被"踩台阶" |
| 电机参数 | 若换电机需重新辨识惯量 | 转矩% 显示错、增益不对 |

---

## 三、视情况做（代码侧）

| 项 | 要不要改 | 说明 |
|----|---------|------|
| `Driver_Servo_number` | **不改**，仍填 0 | 驱动侧仍是 `Servo0` 槽位，只是它的物理对象换了 |
| `WASR_S0_MOTOR` 宏 | 电机同型号（WE130M，17bit，1:1）就**不改**；换电机才改 | 影响每转指令单位（131072）、惯量、额定转矩 |
| `feedMmToUnit`（671） | 电机/齿轮比不变就**不用管** | 它由 `EncCntPerRev/GearNum/GearDen` 算出，宏对了它就对 |
| 上位机/Scope 路径 | **不变** | 仍读 `Servo0` / `Status.Feed*` |

> 换句话说：**只要 VD3E 带的是同一台 WE130M，代码一行都不用改**，纯配置切换。

---

## 四、千万别忘：T 要重扫

`fLeadTimeMs = 20.3` 是 **VD5L 那台**的位置环时间常数（`T ≈ 1/Kp`）。
VD3E 的默认刚性/位置环增益不同 → 换驱动器后这个数**必然变**，不重扫就会出现：

- T 给大了 → 轴跑到理论曲线前面（负误差、可能过冲）；
- T 给小了 → 又回到落后几十上百毫米。

**重扫方法（10 分钟，两次运行）**：

```
① fLeadTimeMs = 0，跑 350 一轮 → 读同步段 FeedIdealErr0 均值 E0（mm）
② T = E0 ÷ 5833 × 1000        （把"落后的距离"换成"时间"）
③ 填入 T，再跑一轮确认残差 <1mm
```

---

## 五、上台架的最短安全步骤

1. **只使能、不下指令**：`Enable=1` → `6041` 应为 `0x1637`（含 fault 位为 0）、`6064` 静止不漂；
2. **30 m/min 触发一次**：确认方向正确、净位移 ≈ **+456,273** 指令单位（680mm × 671）；
   - 方向反 → VD3E 的电机/编码器方向参数（不要改代码）；
   - 位移差 2 倍/几十倍 → 6091 或 `WASR_S0_MOTOR` 宏错；
3. **重扫 T**（见上）；
4. 再上 120 → 350，录 8 通道，和 VD5L 的数据做同口径对比。

---

## 六、建议的推进方式

改动只有两处文件级别的东西（都在 tsproj 里，不动代码）：

1. 14 条 Link：`维控VD5L^...` → `维控VD3E_1^...`
2. VD3E_1 补 `<SuTask>`，并设固定 EtherCAT 地址

**这两件我可以直接改，改完你只要在 XAE 里激活、整定驱动器、重扫 T。**
另外建议保留一份「切回 VD5L」的说明（把 14 行改回去即可），方便两台机来回比对。

---

## 七、4 台 VD3E 版（对拖）与「需要同步主站」报错

### 现象

启用 4 台 VD3E、禁用 VD5L 后点 Activate Configuration，弹：

> 设备'主站'需要同步主站（至少有一个变量与任务变量链接）

### 原因（不是总线问题，是"主站悬空"）

VD3E 虽然启用了，但**既没有 EtherCAT 任务分配、也没有任何 PDO 链接**；
Servo0 原来那 14 条链接指向的是**已被禁用**的 VD5L（禁用从站不参与数据交换）。
于是主站一条有效链接都没有 → 它不知道该同步到哪个任务 → 就是这句话。

所以顺序必须是：**先分配任务 + 建链接，再激活**；"启用 4 台 VD3E"本身不够。

### 4 台版要做的三件事（脚本已做：`fix_tsproj_4vd3e.py` → `fix_tsproj_allslots.py`）

| 项 | 内容 |
|----|------|
| 任务分配 | 4 台各补 `<SuTask>#x02010020</SuTask>`（与 VD5L 同任务），位置照抄 VD5L：第 3 个 `<Fmmu>` 之后、`<DcData>` 之前 |
| PDO 链接 | **每槽 14 条**，见下面「固定映射」表；同时删掉指向 VD5L(Servo0) 和汇川SV630N 的旧链接 |
| 地址 | **刻意不设固定地址**。固定地址要求从站 EEPROM 里已经是该地址，否则激活报 `Slave not found`；按拓扑自动分配即可。激活后到从站页看一眼实际地址 |

### 固定映射（一次接死，以后切场景只启停从站）

| 槽位 | 从站 | 用在哪 |
|------|------|--------|
| Servo0 | 维控VD5L | 本机单轴（`SiteProfile=0`） |
| Servo1 | 维控VD3E_1 | 对拖轴0（`SiteProfile=1`） |
| Servo2 | 维控VD3E_2 | 对拖轴1 |
| Servo3 | 维控VD3E_3 | 对拖轴2 |
| Servo4 | 维控VD3E_4 | 对拖轴3 |
| Servo5~7 | 留空（汇川 3 台暂不接） | — |

共 70 条链接。**切场景只剩两步**：

1. 启用要用的从站、禁用其余（VD5L 与 4 台 VD3E 互斥）；
2. `Enable=0`、`SigAmp=0` 时写 `SiteProfile`（0=本机单轴 / 1=对拖 4 轴）。

**为什么不能"同一槽位随场景换驱动器"**：TwinCAT 的 PDO 链接是静态 1:1，
一个模块变量只能指向一台从站，禁用设备不会改链接。现在的映射覆盖了现有全部场景；
将来 4 驱动 + 4 负载要 8 台 VD3E，加 VD5L 就是 9 槽，超出当前 8 槽 IO 结构 —— 得先扩槽位（tmc 升版本）。

> 角色为 NONE 的槽位即使从站还启着也安全：模块每拍把它的 Controlword 写 0，
> 驱动器停在 Switch-on-disabled。启用/禁用只影响总线扫描与启动时间。

### 已验证的前提（离线）

VD3E 的 PDO 条目名与 VD5L **完全一致**：RxPDO 8 / TxPDO 10，14 个必需项齐全，
连 `Torque demand ` 结尾那个空格都一样 —— 所以链接名可以照抄 VD5L 只换从站名。

### 激活后按这个顺序验收

1. 从站页：4 台 VD3E 都是 OP，无 `Slave not found` / `Invalid Working Counter`；
   记下各自的实际 EtherCAT 地址（应为 1001~1004）。
2. `SiteProfile=1`（Enable=0、SigAmp=0 时写）→
   `AxisCount=4`、**`AxisRole1~4=1`、`AxisRole0=0`**、`Driver_Servo_number=1`、
   `FeedMmToUnit0~3 ≈ 671`（**轴0 现在对应 Servo1**）。
3. `Enable=1`、SigAmp 保持 0 → Servo1~4 的 6041 都到 `0x0027`，6064 不动。
   **这一步过了才说明 70 条链接全对**，零运动风险。
4. `SigOffset=30` 低速，SigAmp 0→正 上升沿触发一次，看 4 轴 607A/6064 与 `FeedIdealErr0~3`。
5. 重扫 T（VD3E 的位置环常数 ≠ 20.3ms）：`fLeadTimeMs=0` 跑一轮读 `FeedIdealErr0` 均值 E0 →
   `T = E0 ÷ 5833 × 1000` → 回填再跑一轮。

> 注意两件事：
> 1. `fLeadTimeMs` 是**全局一个值**，4 轴共用。4 台 VD3E 若刚性/增益整定不一致，
>    超前补偿只能折中 —— 建议 4 台按同一套增益整定，否则后续要改成 `afLeadTimeMs[4]`。
> 2. Scope 里原来录 `Servo0.*` 的通道，对拖场景要改到 `Servo1~4`；`Status.Feed*0~3` 是按**轴号**的，不用改。

---

## 八、以后换驱动器型号（VD3E → VD5E / VD5L / 混机型）

### 先分清"换什么"——三层，只有第三层要动工程

| 换什么 | 改哪里 | 代价 |
|--------|--------|------|
| 辊径、减速比、同步段长、过送量、超前 T | `SiteConfig.afMmPerMotorRev` / `FeedConfig` | 重编译 |
| 电机（功率、编码器位数、齿轮比、惯量） | `MotorSpec.cpp` 加一行；站点表用 `SITE_SLOT_DRV_M(地址, MOTOR_xxx)` | 重编译 |
| **驱动器型号（从站对象变了）** | **tsproj 的 PDO 链接** | 重激活 |

前两层模块逻辑一行都不用改——这是当初用站点表/MotorSpec 表、不用 `#if` 分叉的原因。

### 换驱动器型号的操作顺序

1. **XAE 里换从站**：右键从站 → `Change EtherCAT Slave`（同族、PDO 布局相同时会**自动保留链接**，
   这时第 2~3 步可以跳过）；或者删掉旧从站、加新的（要装 ESI）；
2. **确认新从站有 EtherCAT 任务分配**（没有就补，脚本会补）；
3. **改 `wire_map.json` 一行**：把 `"1": "维控VD3E_1"` 改成 `"1": "维控VD5E_1"` 之类；
4. **跑脚本**：`python apply_links.py`（演练）→ `python apply_links.py --apply`（写入，先关 XAE）；
5. 电机不同则加 `MotorSpec` 表项 + 站点表改用 `SITE_SLOT_DRV_M`；
6. Rebuild All → Activate → **重扫 T**（换驱动器后位置环常数必变）。

`apply_links.py` 会按**从站实际的 PDO 条目名**自动匹配（转小写去空格后比对），
所以维控的 `TxPDO/RxPDO` 和汇川的 `Inputs/Outputs` 命名差异、`Torque demand ` 的尾空格都不用管；
缺项会明确列出并**拒绝写入**，不会生成半截链接集。

### 各从站的 PDO 覆盖度（换型前先查，`chk_pdo_coverage.py`）

| 从站 | Tx/Rx 条目 | 能否当 CSP 驱动轴 |
|------|-----------|------------------|
| 维控 VD5L / VD3E_1~4 | 19 / 16 | 齐全 ✓ |
| 汇川 SV680N | 63 / 54 | 齐全 ✓ |
| 汇川 SV630N_03716 | 46 / 43 | 齐全 ✓ |
| 汇川 SV630N_03713 | 45 / 41 | 缺 `Motor/Shaft revolutions`（仅 6091 齿轮比 PDO 下发无效，1:1 时无影响） |

### 注意：槽位号决定 SiteConfig，型号不影响

只要**槽位号不变**（VD3E 换 VD5E 仍在 Servo1~4），`SiteProfile=1` 那张表就不用改——
模块只关心"槽位 1~4 是驱动轴"，不关心接的是哪个型号。
换到别的槽位才需要加/改 profile。

---

## 九、模块 0.0.0.5：变量改名 + 独立送纸启动位

### 为什么改

1. **轴号 ≠ 槽位号**之后，`FeedFollowErr0` 这种名字会让人误以为是 Servo0。
   真实对应是"轴0 在 Servo1"（对拖场景），所以加 `Axis` 前缀 + 直接给出映射变量。
2. **SigAmp 兼作 Start 触发**语义别扭（数值型变量当开关，容易误碰），换成独立的 `bFeedStart`。

### 改名对照（旧 → 新）

| 旧（0.0.0.4） | 新（0.0.0.5） | 说明 |
|---|---|---|
| `FeedFollowErr0..3` | `Axis0FollowErr..Axis3FollowErr` | 跟随偏差，指令单位 |
| `FeedFollowErrMm0..3` | `Axis0FollowErrMm..` | 跟随偏差，mm |
| `FeedIdealErr0..3` | `Axis0IdealErr..` | 相对理论曲线偏差，mm（**评价超前补偿看这个**） |
| `FeedMmToUnit0..3` | `Axis0MmToUnit..` | mm→指令单位 换算，台架 ≈671 |
| — | **`Axis0Slot..Axis3Slot`**（新增） | **轴 n 现在在哪个槽位**（对拖应为 1/2/3/4） |
| — | **`bFeedStart`**（新增） | 送纸启动位：1=送纸/续送，0=停止；替代 SigAmp 触发 |
| `SigAmp` | 送纸实验里不再使用 | 其它实验（波形/扫频/位置环）含义不变 |

`Driver_Servo_number` 保留（= `Axis0Slot`，兼容旧上位机）。

### 上机顺序（这次必须多做一步）

1. 打开 C++ 工程 → **TMC Code Generator**（重新生成 `WASR_TC.tmc` / `WASR_TCServices.h`）；
2. **Rebuild All**；
3. 回 XAE：工程里 `AutoReloadTmc="true"`，此时会提示 TMC 变更 → 允许重载；
   若模块实例仍显示 0.0.0.4 或激活报版本不匹配，**把模块实例删掉重新插入一次**；
4. 激活后核对：`Status` 末尾应能看到 `Axis0Slot=1`、`bFeedStart`；`Axis0MmToUnit ≈ 671`；
5. Scope 里旧通道要按新名字重建（`Status.Feed*` → `Status.Axis*`）。



