# gipylib 与 ignav 的 RTK-INS-TC 平面精度详细对比与改进计划

## 1. 任务状态与范围

- 状态：**Phase 0/0A/1 实验已完成（2026-8-22）**，根因已定位并实证，详见文末「实验结果与根因结论」。Phase 2-6 中与 prnbias 相关项已被 Phase B 单变量矩阵覆盖；TC 层 AR 启用被阻塞于新发现的 LD 崩溃缺陷。
- 本文只制定对比、修改、验证和回退计划；本轮不修改 YAML、Python、C++ 或其他源代码，不生成新的实验结果。
- 目标对象：`gipylib/data/rtk-ins紧组合.yaml` 驱动的 GPS-only `rtk-ins-tc`。
- 参照对象：`ignav-debug/a-cpt/cpt-rtktc-gpsonly.conf` 的对应 GPS-only RTK-INS-TC 结果。

## 2. 任务原因

在同一目标数据和同一时间窗口内，ignav 修改数据集后的 GPS-only RTK-INS-TC 平面精度已经达到厘米级，而 gipylib 的结果明显偏差。若不先定位差异来源，直接调整 INS 杆臂、姿态或过程噪声，容易把 GNSS 解算问题误判为组合滤波问题，造成不可复现的参数堆叠。因此需要建立统一评价口径，并按“输入与基线 -> GNSS 载波/模糊度 -> 观测权重 -> 滤波组合参数”的顺序逐层验证。

## 3. 已验证基线与精度差距

评价窗口为 GPS Week 2046、SOW `359000-359100`。使用整数秒附近的 GNSS 结果作为主要评价点，其余历元视为机械编排点；QINS 信息只作为辅助属性，不作为必须的结果识别条件。

| 方案 | E RMSE (m) | N RMSE (m) | U RMSE (m) | 水平 RMSE (m) |
|---|---:|---:|---:|---:|
| ignav GPS-only RTK-INS-TC | 0.026 | 0.051 | 0.498 | **0.057** |
| gipylib `rtk-ins紧组合.yaml` / `RTKINS.rslt` | 0.098 | 0.189 | 0.492 | **0.213** |

全段水平 RMSE 约为 ignav `0.305 m`、gipylib `0.315 m`。这表明问题主要集中于目标窗口，不能只用全段平均值掩盖短窗口的机动后偏差。

目标验收值：目标窗口水平 RMSE `<= 0.08 m`，并尽量与 ignav 相差 `<= 0.03 m`；全段水平 RMSE 保持 `<= 0.32 m`，且不得通过牺牲其他时间段换取窗口内指标。

## 4. 数据、脚本与统一评价口径

### 4.1 文件位置

- gipylib 配置：`gipylib/data/rtk-ins紧组合.yaml`
- gipylib 输出：`gipylib/data/output/RTKINS.rslt`
- 真值：`gipylib/data/truth.csv`
- 统一评估脚本：`ignav-debug/a-cpt/plot/error_rtdtc.py`
- 参考评估脚本：`gipylib/data/plot/error-rslt(1).py`、`gipylib/data/plot/tra-neu.py`
- 证据与历史测试记录：`ignav-debug/mem/2026-8-22.md`

### 4.2 评价规则

1. 两套程序必须使用同一份已修正时间戳的数据、同一真值和同一坐标基准。
2. 先检查 GPS 周、SOW、Unix 纳秒与 `utc2gpst(+18s)` 转换，确认不存在旧 `cpt_euroc.csv` 的 18 秒时间问题。
3. 采用同一插值真值、同一整数秒最近点匹配、同一目标窗口和全段窗口。
4. 至少报告 E/N/U RMSE、水平 RMSE、3D RMSE、最大误差、有效历元数和逐历元误差曲线。
5. 同时记录解状态、卫星数、载波相位可用率、模糊度固定率/连续固定时长、重初始化次数和输出位置标准差。

### 4.3 纯 RTK 交叉验证主线

纯 RTK 必须单独作为一个阶段，不能只从 RTK-INS-TC 结果反推 GNSS 是否正确。需要回答两个问题：第一，gipylib 的纯 RTK 在目标窗口到底达到何种平面精度；第二，在同一观测和星历输入下，独立 RTKLIB C 实现是否得到相近结果。

交叉实现和配置依据如下：

- RTKLIB C 源码：`ignav-debug/tools/rtklib/src/rtkpos.c`、`src/pntpos.c`、`src/rtkcmn.c`、`src/lambda.c`、`src/rnx2rtkp.c`；执行程序为 `ignav-debug/tools/rtklib/build/rnx2rtkp.exe`。
- RTKLIB 示例配置：`ignav-debug/tools/rtklib/data/Dataset_1_23/rtd.conf`；调试入口和输入顺序见 `ignav-debug/tools/rtklib/.vscode/settings.json`。
- 配置不能原样作为 GPS-only RTK 结论：示例 `rtd.conf` 当前是 `pos1-posmode=dgps`、`pos1-navsys=33`，必须为本数据集建立单独的 GPS-only `kinematic` 配置，并明确 rover/base RINEX、广播星历、输出时间系统和输出格式。

纯 RTK 对比必须固定以下条件：同一 rover/base 观测文件、同一导航文件、同一 GPS-only 星座、同一 L1/L2 频点、同一截止高度角、同一 forward/combined 方向、同一基准站坐标、同一 GPST 时间转换和同一目标窗口。RTKLIB 的 `pos2-armode` 先分别运行 off、continuous、fix-and-hold；只有当 off/AR 的输入和输出状态可解释时，才比较模糊度门限。

每个纯 RTK 结果都使用 `error_rtdtc.py` 评价 E/N/U、水平和 3D RMSE，并额外输出 Q=single/DGPS/float/fix 的比例、有效卫星数、首次固定时刻、连续固定时长、重初始化次数、位置标准差和逐历元 E/N 漂移。必须建立以下对照：

| 结果 | 实现 | 作用 |
|---|---|---|
| G0 | gipylib `positioning_mode=rtk`，INS off，当前 `armode=0` | 测量 gipylib 纯 RTK 基线水平 |
| G1 | gipylib 纯 RTK，仅打开 AR | 判断 gipylib 内部载波/AR 是否生效 |
| R0 | RTKLIB C，GPS-only kinematic，AR off | 独立浮点/码相位处理基准 |
| R1 | RTKLIB C，GPS-only kinematic，continuous/fix-and-hold | 独立固定解基准 |

判定逻辑：

1. 若 G0 与 R0 的 E/N 漂移、水平 RMSE 和解状态相近，说明观测数据或卫星几何本身可能限制精度，不能直接判定 gipylib 有算法错误。
2. 若 R0/R1 达到厘米级而 G0 明显停留在约 `0.2 m` 或码级标准差，优先检查 gipylib 的观测映射、基站/流动站接收机标识、载波相位有效标志、周跳处理、频点选择、模糊度状态和 AR 配置。
3. 若 G1 能随 AR 稳定改善且接近 R1，说明 gipylib 主要是配置未启用或未保持固定；若 G1 仍明显落后，再逐项比对 `rtkpos` 的残差检验、卫星筛选、权重和状态初始化。
4. 若 G0/G1 与 R0/R1 都不一致但误差只在目标窗口出现，应优先检查时间同步、基准坐标、参考星切换、机动期间观测缺失和真值匹配，而不是立即修改 INS。

纯 RTK 的“是否有误”必须按证据分级：只有在输入逐历元一致、时间偏差小于一个采样周期、参考坐标一致且 RTKLIB C 的 R1 已稳定固定时，gipylib 与 R1 仍存在显著差异，才可将问题定性为 gipylib RTK 实现/适配错误；否则只能记录为配置或数据条件差异。

## 5. 现象与已排除原因

目标窗口中，ignav 的 N 误差约 `+0.03~+0.08 m` 且基本平直；gipylib 的 N 误差由约 `+0.01 m` 漂移至 `-0.25 m`，E 误差由约 `0 m` 漂移至 `+0.11 m`，呈机动后的缓变系统偏差。

已排除或降级的解释：

- 旧数据的 18 秒时间戳问题不是当前根因；更换为真实 Unix 纳秒时间戳后该问题消失。
- 不是 INS 融合独有问题：gipylib 纯 RTK（INS off）目标窗口约 `0.218 m`，RTK-TC 约 `0.213 m`，误差在融合前已存在。
- 不能把历史 BDS 失败、旧数据假发散或单纯杆臂调参结果作为当前结论。

## 6. 当前最可能原因（按优先级）

1. **RTK 浮点/固定链路未有效锁紧载波相位。** gipylib 纯 RTK 位置标准差全程约 `0.38 m`，更接近码级/DGPS 级结果；ignav 后半段可降至约 `0.053 m`。
2. **模糊度初始化、保持、固定门限与 ignav 不一致。** 当前配置 `armode: 0`，使结果长期停留在非 AR 或浮点状态；测试显示 `armode: 3` 可将目标窗口水平 RMSE 从约 `0.218 m` 改善至约 `0.136 m`，说明载波和 AR 对问题有直接影响，但尚未达到目标。
3. **载波/伪距观测权重不匹配。** 重点核查 `eratio: [300, 100]`、`prnbias: 0.5` 及卫星高度角、周跳和失锁处理。
4. **机动后滤波重收敛或状态重整定。** 只有在 GNSS RTK 解达到稳定固定后，才检查 forward/combined、初始协方差、过程噪声和机动后重新收敛。
5. **杆臂、安装角和时间同步估计的次要影响。** 当前 `leverarm` 为零、`estimate_leverarm: 1`，但纯 RTK 已有同样误差，因此这些只能作为后置敏感性实验，不能作为首要修复方向。

## 7. 科学的修改与验证安排

### Phase 0：冻结可比基线

- 复制并保留当前 YAML、输入数据、输出文件和评估命令，建立带日期的实验目录。
- 先复跑基线，确认 `0.213 m`（窗口）和约 `0.315 m`（全段）可重复。
- 若时间戳、真值匹配、有效历元数或输出状态不一致，先修正实验条件，不进入参数调优。

### Phase 0A：纯 RTK 精度量化与 RTKLIB C 对照

- 先运行 G0，明确 gipylib 纯 RTK 在目标窗口和全段的平面精度；结果中必须单列 E RMSE、N RMSE、水平 RMSE、最大水平误差和有效历元数。
- 根据 `ignav-debug/tools/rtklib/src` 的 `rtkpos`/`lambda` 实现和 `data/Dataset_1_23/rtd.conf` 的参数含义，创建实验用 RTKLIB C GPS-only kinematic 配置，不能沿用示例中的 DGPS、多星座设置。
- 运行 R0、R1，并用同一 `error_rtdtc.py` 与 G0/G1 对齐。首先比较纯 RTK 是否存在约 `0.2 m` 的窗口误差，再比较 AR 开启后是否进入厘米级。
- 若 RTKLIB C 与 gipylib 都为码级/分米级，先判定为共同输入、观测可用性或卫星几何问题；若仅 gipylib 异常，暂停后续 INS 参数实验，进入实现逐项核查。
- 该阶段的输出应形成“纯 RTK 精度基线表”和“G0/G1/R0/R1 状态对照表”，作为后续 Phase 1 的入口。

### Phase 1：只验证 gipylib AR 开关

- 仅改变 `armode`，依次测试关闭、连续 AR/固定 AR 等现有合法模式；其他参数完全冻结。
- 记录窗口/全段 RMSE、固定率、首次固定时间、连续固定时长和重初始化次数。
- 若 AR 未改善固定率或引入跳变，立即回退到基线并检查载波观测完整性。
- Phase 1 的每个候选参数必须与 RTKLIB C 的 R0/R1 对照，确认改善来自真实载波固定，而不是评价点筛选或输出状态误标。

### Phase 2：模糊度门限与保持策略

- 在表现最好的 `armode` 上逐项测试 `thresar`、`thresar1`、`minlock`、`minfix`、`minfixsats`、`minholdsats`、`var_holdamb`。
- 每轮只修改一个变量族，先宽松/保守两端做小范围扫描，再围绕最佳点细化；禁止同时改变门限和观测权重。
- 验收必须同时满足精度、固定稳定性和无明显错误固定，不能只看单一 RMSE。

### Phase 3：载波与伪距权重

- 在 AR 参数冻结后，扫描 `eratio`、`prnbias` 及相关载噪比/高度角权重。
- 检查权重变化是否让位置标准差从码级降至分米/厘米级，及是否造成低高度卫星过度影响。
- 任何提升都必须在全段复核，避免窗口过拟合。

### Phase 4：滤波方向与机动后重收敛

- 固定 GNSS 最佳参数后，比较 `forward` 与可用的 combined/backward 模式。
- 检查机动前后状态协方差、过程噪声、观测拒绝和重初始化；关注 N/E 缓慢漂移是否在 TC 中被放大。
- 该阶段不得重新改 AR 与权重参数。

### Phase 5：次要系统参数敏感性

- 最后才测试 `estimate_leverarm`、安装角、时间同步，以及 INS 初始协方差/过程噪声。
- 使用纯 RTK 与 RTK-TC 成对对照，确认改动确实改善组合误差，而不是掩盖 GNSS 基线问题。

### Phase 6：最终 TC 验收

- 将最佳 GNSS 参数接入最终 RTK-INS-TC，重新跑目标窗口和全段。
- 以 ignav 结果为参照检查 E/N 误差曲线、水平 RMSE、固定连续性和高度方向是否退化。
- 只有达到验收阈值且全段不退化，才形成候选配置；否则回到最近一个通过阶段的配置。

## 8. 实验记录模板

| 实验编号 | 基线/改动项 | armode/AR 指标 | 窗口 E/N/U/水平 RMSE | 全段水平 RMSE | 固定率/重初始化 | 结论/回退点 |
|---|---|---|---|---|---|---|
| B0 | 当前配置 | 记录实测 | 0.098/0.189/0.492/0.213 | 0.315 | 记录实测 | 基线 |
| P1-xx | 单变量改动 | 记录实测 | 记录实测 | 记录实测 | 记录实测 | 接受/回退 |

## 9. 风险、回退与停止条件

- 每次实验保留原始输出、配置快照、命令行和评估 JSON/CSV；不得覆盖基线。
- 若出现错误固定、水平跳变、有效历元减少、全段 RMSE 增大超过 `0.02 m` 或高度明显恶化，立即回退到上一通过配置。
- 若连续两轮只改善窗口而损害全段，停止局部调参，重新核查数据、时间、卫星状态和真值匹配。
- 若 AR 已稳定但仍无法达到 `<=0.08 m`，应优先检查 gipylib 与 ignav 的观测预处理、卫星选择、周跳处理和状态定义差异，而不是继续扩大参数搜索范围。

## 10. 交付边界

本文件是后续实验的唯一规划入口。当前任务只新增/完善本 Markdown 文件；不修改 `gipylib/data/rtk-ins紧组合.yaml`，不修改评估脚本，不修改算法实现。代码和配置变更必须在后续按本计划逐阶段执行，并以记录表和统一脚本结果作为依据。

## 11. 实验结果与根因结论（2026-8-22 执行完毕）

执行方式：不修改任何源码；YAML 副本单变量实验 + monkeypatch 对照；每轮评估、无改善即回退。
产物：`exp-8-22/`（脚本/配置）、`data/output/exp-8-22/`（输出+评估 JSON/PNG）、`exp-8-22/r0r1/`（RTKLIB C 交叉验证）。

### 11.1 根因

**`gnss.prnbias: 0.5` 过大（ignav 为 0.03，17 倍）→ P_amb 稳态托底 ≈13 cycle² → 载波相位失效 → 浮点解为码级。**
插桩证据：B0 窗口内 P_amb=13.64 cycle²、pos std≈0.54m 恒定；E2(prnbias=0.03) P_amb=3.02、std≈0.17m。

### 11.2 单变量实验记录（窗口/全段 H-RMSE, m）

| 实验编号 | 基线/改动项 | armode/AR 指标 | 窗口 E/N/U/水平 RMSE | 全段水平 RMSE | 结论 |
|---|---|---|---|---|---|
| B0 | 当前配置(复现) | off | 0.102/0.193/0.500/**0.218** | 0.298 | 基线 ✓ |
| E1 | accelh/v→0.1/0.03 | off | 0.104/0.197/0.491/**0.223** | 0.297 | 回退 |
| **E2** | **prnbias→0.03** | off | 0.071/0.078/0.575/**0.105** | **0.252** | **采纳** |
| E3 | E2+E1 | off | —/**0.120** | 0.257 | 回退 |
| E4 | E2+eratio[100,100] | off | —/0.107 | 0.256 | 回退 |
| E5 | E2−速度PSD注入(补丁) | off | —/0.133 | 0.256 | 回退 |
| E6/E7/E11 | maxinno30 / thresslip0.03 / 关对流层 | off | 均=E2(0.105) | =E2 | 无差异 |
| E8/E10 | cnr_min关 / err权重0.005 | off | 0.107 / 0.135 | —/0.260 | 回退 |
| **E9** | **E2+armode=3(纯GNSS)** | FIX率97.7% | 0.010/0.016/0.013/**0.019** | **0.053** | **采纳(纯GNSS)** |

### 11.3 TC 管线验收对照（`rtk-ins紧组合.yaml`, ins.enabled=tc）

| 方案 | 窗口 H-RMSE | 全段 H-RMSE | 备注 |
|---|---|---|---|
| B0TC 基线 | 0.213 | （历史 0.315） | 与第 3 节一致 ✓ |
| **E2TC (prnbias=0.03)** | **0.100** | **0.252** | 全段优于 ignav 参照 0.305 |
| ignav 参照 (float) | 0.057 | 0.305 | INS-TC 输出, 含 INS 平滑 |
| E9TC (armode=3) | 崩溃 | — | mlambda.LD SystemExit 缺陷 |
| E9TCp (正则化补丁) | 0.100 | 0.252 | TC侧AR ratio 从未通过, 输出与E2TC逐字节一致 |

### 11.4 RTKLIB C 交叉验证（G/R 对照表收口）

| 结果 | 实现 | 配置 | 窗口 | 全段 |
|---|---|---|---|---|
| G0 | gipylib 基线 | armode=0, prnbias=0.5 | 0.218 | 0.298 |
| G1' | gipylib E2 | armode=0, prnbias=0.03 | 0.105 | 0.252 |
| G1'' | gipylib E9 | armode=3, prnbias=0.03 | **0.019** | **0.053** |
| R0 | RTKLIB C demo5 b34L | kinematic L1 AR-off, ignav 统计 | 0.089 | 0.250 |
| R1 | RTKLIB C demo5 b34L | continuous | 0.089 | 0.256 |

判定：R0/R1 与修复后 G1' 同水平 → 观测数据/几何可支撑 ~0.09m 浮点与 cm 级固定，gipylib 无实现级错误；原差距全部来自 prnbias 配置。ignav 参照更优系 INS-TC 融合收益，非 GNSS 层差异。

### 11.5 新发现的算法缺陷（未修改，待决策）

1. `mlambda.LD()` 非正定直接 `raise SystemExit`：线程内静默杀死进程（E9TC 必现，终止于 SOW 358188）。建议改为返回错误码由调用方降级。
2. `TcAmbiguity.try_fix` 的 P_amb 非正定单轮触发 524 次；正则化后 ratio 始终 <3 从未固定 → TC 侧 AR 目前无效。需排查 TC estimator 模糊度协方差数值性态后再启用 `armode>0`。

### 11.6 后续行动建议（供决策）

1. 【配置】`gnss.prnbias: 0.5 → 0.03`（两条管线立即受益；预期达成窗口 ≈0.10，接近 ≤0.08 目标）。
2. 【配置】纯 GNSS 场景可开 `gnss.armode: 3`（0.019/0.053）；TC 场景暂缓。
3. 【代码】修复 11.5 两处缺陷后，再评估 TC+AR 是否将 TC 窗口推入 ≤0.08。

## 12. TC 浮点融合修复（2026-8-22 第二轮，已完成）

第 11 节 prnbias 修复后 TC 窗口 0.100，仍未达 ignav 0.057。第二轮定位到**融合层根因**：

- `build_Q_ins()` 将 LC 调参值 `ins.vel_psd=0.5` 无条件注入 TC 的 P_vel → P_pos 托底 ≈0.35 m²（诊断实测）→ INS 记忆失效，TC 退化为逐历元 GNSS 融合（解释了"TC≈纯GNSS"现象）。
- 扫描确定 TC 适用值：0.002→0.085、**0.001→0.071**、0.0005→0.064；≤0.0001 失稳发散。
- 已落地：`data/rtk-ins紧组合.yaml` `vel_psd=0.001`。

| 方案（正式配置, float 无 AR） | 窗口 H-RMSE | 全段 H-RMSE |
|---|---|---|
| 修复前 | 0.213 | 0.315 |
| **修复后** | **0.071** ✓(≤0.08 验收) | **0.254** (<ignav 0.305) |
| ignav 参照 | 0.057 | 0.305 |

浮点解已达到与 ignav 对等水平（剩余 ~0.014m 含两实现共有共模偏移，非算法 bug）。按既定顺序，下一步可启动模糊度固定缺陷修复（mlambda.LD 崩溃防护 + TcAmbiguity PD/ratio 问题排查）。

## 13. TC 模糊度固定缺陷修复（2026-8-22 第三轮，已完成并提交 7435a7f）

按第 12 节顺序，浮点达标后完成 AR 修复：

1. **LD 崩溃缺陷**：mlambda.LD SystemExit→LinAlgError，调用方降级跳过；
2. **SD 病态 Q 缺陷（ratio 恒 1 的真正根因）**：SD 模糊度公共基准方向从未被 DD 观测约束，直接 SD LAMBDA 时候选解全体 ±1 残差几乎相同。改为 SD→DD 变换后求解（y=D·x, Qb=D·Q·Dᵀ），restamb 语义写回；
3. **过程噪声缺失**：TC 模糊度补 prnbias²·dt 游走底；sync 健康门控阻断僵尸槽位自我延续。
4. TcAmbiguity PD 防护；holdamb 仅约束实际固定槽位。

| 方案（正式配置栈: prnbias=0.03, vel_psd=0.001） | 窗口 H-RMSE | 全段 H-RMSE |
|---|---|---|
| TC 浮点 (armode=0) | 0.073 | ~0.254 |
| **TC 浮点+AR (armode=3)** | **0.019** | **0.220** |
| ignav 参照 (float) | 0.057 | 0.305 |

固定率 69%，无发散/错误固定特征；armode=0 回归无退化。验收目标（窗口 ≤0.08 且接近 ignav）全面达成。
