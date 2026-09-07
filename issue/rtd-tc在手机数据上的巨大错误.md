# 手机数据 RTD-TC 巨大误差分析与后续工作计划

## 1. 任务原因与问题定义

使用 `gipylib/data/rtdtc.yaml` 时，RTD-INS 紧组合结果可以达到约 2 m 以内的定位误差；切换到 `gipylib/phone/rtdtc.yaml` 后，手机 RTD-TC 结果相对纯 RTD 明显恶化，出现数百米级误差。用户已手动确认手机原始数据不存在时间戳错误，因此时间戳不再是首要假设，但仍需确认评估程序没有重复转换时间系统。紧组合的基本要求是：在同一数据和同一评价口径下，TC 不应无故显著劣于其输入的纯 GNSS RTD；因此必须区分评估口径、数据特性、配置适配和 INS 融合实现。

本文件只制定调查和修复计划，不修改任何 Python、C/C++、YAML、RINEX 或输出结果文件。

## 2. 当前对照对象和评定文件

### 2.1 配置与原始数据

- CPT 对照配置：`gipylib/data/rtdtc.yaml`
- 手机配置：`gipylib/phone/rtdtc.yaml`
- 手机流动站观测：`gipylib/phone/phone.25o`
- 手机基站观测：`gipylib/phone/phone_base.25o`
- 手机广播星历：`gipylib/phone/nav.25n`
- 手机 IMU：`gipylib/phone/phone_imu_euroc.csv`
- 手机已有纯 RTD 输出：`gipylib/phone/output/RTD.pos`
- 手机已有 RTD-TC 输出：`gipylib/phone/output/RTDTC.rslt`

### 2.2 评估脚本与图表

- 首要统一评估脚本：`ignav-debug/a-cpt/plot/error_rtdtc.py`
- 轨迹对比脚本：`ignav-debug/a-cpt/plot/tra_neu.py`
- gipylib 手机局部评估脚本：`gipylib/phone/eval_enu.py`、`gipylib/phone/error_rslt.py`
- 手机已有图：`gipylib/phone/plot/rtd-tc.png`
- 真值/参考数据（若时间和坐标基准确实覆盖手机数据）：`gipylib/data/truth.csv`
- 手机运行日志：`gipylib/phone/rtdtc.stdout.log`、`gipylib/phone/rtdtc.stderr.log`、`gipylib/phone/rtd_trace_window.log`
- ignav 手机配置：`ignav-debug/phone/rtdtc.conf`
- ignav 手机输出：`ignav-debug/phone/output/rtdtc.rslt`
- ignav 手机参考真值：`ignav-debug/phone/mate40ref.kf`
- ignav 手机评估记录/脚本：`ignav-debug/phone/analyze_error.py`、`ignav-debug/phone/error_analysis_output.txt`、`ignav-debug/docs/superpowers/plans/2026-08-20-rtdtc-gps-evaluation.md`
- CPT 数据集跨实现记录：`ignav-debug/issue/8-21.md`（其中已记录 gipylib 与 ignav RTDTC 的同窗口对比）

评定文件必须同时包含纯 RTD 和 RTD-TC，不能只评估 `RTDTC.rslt`。若手机没有与 `truth.csv` 同一时间、同一坐标基准的真值，必须先明确只能做 RTD-TC 相对 RTD 的一致性评估，不能把相对误差冒充绝对定位精度。

## 3. 必须先完成的评估脚本核对

这是后续工作的硬性前置条件。任何配置或算法修复前，先逐项审计并用小样本手算核对：

1. **时间系统**：手机原始时间戳已由人工检查确认无误；后续只需确认 `RTD.pos`、`RTDTC.rslt`、手机 IMU 和真值在评估入口的 GPS week/SOW、GPST/UTC、Unix 秒及 `+18 s` 规则一致，排除脚本重复转换，而不再把原始数据时间戳作为首要修复方向。
2. **文件格式**：核对 `RTD.pos` 的 LLH/XYZ、`RTDTC.rslt` 的列定义、Q/Qins 列位置、有效行筛选和注释跳过逻辑；确认没有把 XYZ 当 LLH 或把高度单位误读。
3. **坐标基准**：确认纯 RTD 与 TC 是否都以同一基站坐标、同一 ECEF/LLH 基准输出；检查 ENU 原点是否来自真值首点而非不同结果首点。
4. **时间匹配**：使用整数秒附近最近点作为主要 GNSS 评价点，同时保留全部 IMU 机械编排点；报告匹配误差、有效历元数和未匹配比例。
5. **误差计算**：独立复算 E/N/U、水平误差和 RMSE，检查 East/North 顺序、Up 符号、度弧度转换和 WGS84 参数。
6. **异常点策略**：分别输出全量、整数秒点、纯 RTD Q=有效点和 TC GNSS 更新点；禁止用删除数百米点的方式得到“改善”。
7. **基线自洽性**：先验证评估脚本能从 `RTD.pos` 重现已知纯 RTD 精度，再验证将同一 RTD 文件复制为等价输入时脚本结果不变。

脚本核对的交付物应是：输入行数/有效行数、时间范围、时间偏差统计、坐标解析样例、ENU 原点、E/N/U/水平 RMSE 及一张逐历元误差曲线。若这些项目不能复现，暂停根因判断。

## 4. 两数据集、两实现的反差对比

本问题不能只比较“gipylib 手机 RTD-TC 与纯 RTD”，必须建立同一评价脚本下的四象限矩阵：

| 数据集 | gipylib RTD-TC | ignav RTD-TC | 当前事实/待核对内容 |
|---|---|---|---|
| `gipylib/data` CPT 数据 | 用户观察为约 2 m 以内，且优于 ignav；历史 `issue/8-21.md` 某窗口记录约 `0.406 m` | 用户观察明显劣于 gipylib；历史 `issue/8-21.md` 同一记录约 `390 m` | 这些数字可能对应不同窗口/脚本，必须使用同一 truth、窗口、整数秒点和坐标基准复核 |
| `gipylib/phone` 手机数据 | 数百米级，明显劣于纯 RTD | 用户已观察约 10 m 级 | 必须使用 `mate40ref.kf` 与同一脚本重算，确认“数百米”和“10 m”是同一统计量 |

该反差说明不能简单归因于“gipylib 的 TC 算法整体错误”：如果同一实现对 CPT 优于 ignav、对手机却大幅落后，问题更可能是手机数据特性触发了某个实现/配置边界，或两套评估链路并不等价。后续必须同时检查：

1. 两实现是否消费完全相同的 rover/base RINEX、星历、基站坐标、GPS/BDS/GAL 选择和 L1 观测；
2. 两实现是否使用相同的 INS 输入、RFU/FRD 解释、初始对准、杆臂和状态初值；
3. 两实现的 RTD 质量是否相当，TC 是否在相同历元收到有效 GNSS 更新；
4. 两套评估是否使用同一手机真值、同一时间窗、同一 LLH/ECEF 解析和同一整数秒/全历元规则；
5. 手机数据中的低成本 IMU 噪声、卫星遮挡、周跳和多路径是否让某个滤波器的权重/创新门限失效，而 CPT 数据没有触发该问题。

## 5. 已知配置差异与可能原因

以下是候选原因，当前均不能在未完成评估脚本核对前定性为最终根因。

### 5.1 手机 IMU 坐标系或入口转换不一致（高优先级）

手机配置使用 `phone_imu_euroc.csv`，并声明 IMU 坐标为 RFU。原始时间戳已人工确认无误，但仍需确认入口没有额外改变时间；更重要的是检查 RFU/FRD 转换方向、陀螺/加速度单位和 ignav `ins-imucoors=2` 的语义是否与 gipylib 完全一致。手机数据可能触发该差异，而 CPT 数据没有触发。

### 5.2 GNSS 与 IMU 时间覆盖/采样率错位

手机 IMU 可能存在起始时间、结束时间、重复时间戳、非 100 Hz 间隔或中断；配置固定 `data_rate: 100`。若 GNSS 更新落在 IMU 覆盖之外，TC 可能长时间机械编排或使用错误的最近点。需要统计 IMU `dt`、GNSS-IMU 最近邻差、首尾覆盖和每次更新间隔。

### 5.3 手机初始姿态与动态对准不充分

手机配置把 `initial_att_std_si` 放宽到 `[0.05, 0.05, 0.5]` rad，`dynamic_speed_threshold` 改为 `4.0 m/s`，而 CPT 配置为较小姿态不确定度和 `3.0 m/s`。低成本手机 IMU 的 yaw、加速度计偏置和运动激励可能无法满足动对准条件；初始航向错误会在 TC 中投影为巨大水平误差。

### 5.4 过程噪声与量测协方差严重不匹配

手机配置使用 `gyro_psd=1e-5`、`accel_psd=1e-2`、`gyro_bias_psd=1e-10`、`acce_bias_psd=1e-4`，与 `data/rtdtc.yaml` 的 CPT 级参数相差多个数量级；同时手机 `pos_psd=0.0`、`vel_psd=1.0`。过大的过程噪声会让错误 IMU 状态获得过大的卡尔曼增益，过小的位置随机游走又可能导致协方差锁死。必须查看 TC 创新、P 对角线、K 增益和反馈状态，而不能只凭 RMSE 调参。

### 5.5 基站坐标格式或数值错误

手机配置显式设置 `rb_format: xyz` 和 ECEF 基站坐标；若坐标与 `phone_base.25o` 头部不一致、坐标顺序错误、单位错误或适配器未正确读取 `rb_format`，RTD 与 TC 可能使用不同参考框架。纯 RTD 误差较小而 TC 数百米，仍不能排除 TC 初始化直接采用错误 `rb` 的可能。

### 5.6 手机观测频率映射和星座选择

手机启用 `GPS/GAL/BDS`，并将 BDS 第一频率设为索引 6（`1561.098 MHz`），因为 RINEX 使用 `C2I`。若解析器、`freq_ix0` 或 BDS `C2I` 映射不一致，TC 使用的双差观测可能缺失、错频或卫星集合与纯 RTD 不同。已有 `gipylib/phone/test_phone_rinex.py` 专门检查该风险，后续必须把观测计数和每系统卫星集合列入评估。

本项升级为当前最高优先级假设：需要直接分析 RTKLIB C 对同一手机 RINEX 的观测类型、频率索引和优先级选择，不能只比较配置中的频率数值。RTKLIB 的关键参考实现为：

- `ignav-debug/tools/rtklib/src/rinex.c::set_index()`：按系统遍历 RINEX 观测类型，将 `obs2code()` 映射到内部 code，使用 `code2idx()` 得到频点，再通过 `getcodepri()` 选择每个频点优先级最高的观测码；未选中的同频码不进入主频观测槽。
- `ignav-debug/tools/rtklib/src/rtkcmn.c::code2freq_*()`：把内部观测码转换为实际载波频率。当前源码明确将 BDS `C2I` 的频率字符 `2` 映射为 `FREQ1_CMP`，因此必须以当前源码实测结果为准，不能沿用“C2I 一定无法识别”的旧结论。
- `ignav-debug/tools/rtklib/src/rtkpos.c`：使用已选的 `obs.L/P/code` 和 `sat2freq()` 进入单点/双差残差、周跳、权重和状态更新；`nf` 只决定参与的主频数量，不等价于“按数组下标盲选频率”。

规划新增脚本（后续实现，不在本轮创建）：`gipylib/phone/analyze_rtklib_frequency.py`。脚本输入手机 rover/base RINEX 和 RTKLIB 配置/trace，输出每系统每观测码的 `obs2code -> code2idx -> getcodepri -> pos -> code2freq/sat2freq` 映射、每历元有效载波/伪距数量、最终进入 `nf` 主频槽的卫星集合，以及与 gipylib 实际 `freq_ix0/freq_ix1` 选择的差异。必要时同步增加一个只读诊断入口，分别打印纯 RTD 和 RTD-TC 消费的观测索引，不能用最终位置误差反推频率选择。

### 5.6A 频率选择配置开关设计

后续配置文件（至少 `gipylib/phone/rtdtc.yaml`，并评估是否推广至 `data/rtdtc.yaml` 和其他 GNSS 配置）规划增加：

```yaml
select_fre: off   # off: 按 RTKLIB 观测码/频点/优先级自动选择；on: 强制采用 freq_ix/freq_table/dfreq_glo
```

- `select_fre: off`（建议默认值）：不把 `freq_ix0`、`freq_ix1`、`freq_table`、`dfreq_glo` 当作手机观测挑选的唯一依据；先按 RTKLIB 的 RINEX 观测码、系统频点、优先级和可用性选择，再将选中的实际频率传入 RTD/TC。
- `select_fre: on`：明确表示用户承担频率映射责任，强制从配置读取 `freq_ix0`、`freq_ix1`、`freq_table` 和 `dfreq_glo`；启动时必须校验频率索引、观测码实际频率、GLONASS FCN 和 rover/base 一致性，发现不一致应停止而不是静默回退。
- 开关只控制“频率/观测槽选择”，不改变 `nf`、星座开关、周跳检测、AR 或滤波权重；每次运行必须在日志中记录有效模式和最终映射。
- 兼容性策略：旧配置缺少 `select_fre` 时按 `off` 处理，避免旧手机配置继续强制使用可能错误的手动索引；只有专项复现实验才显式设为 `on`。

### 5.7 纯 RTD 与 TC 实际消费的 GNSS 结果不同

配置注释称 TC 消费同一套原始观测，但需要验证代码路径：纯 RTD 的位置、状态质量、卫星数、协方差和 TC 的原始观测更新是否使用同一历元、同一筛选和同一基站插值。若 RTD 的 2 m 结果来自有效码双差，而 TC 因 Q 状态、量测列或降级逻辑错误使用了异常观测，TC 会明显劣化。

### 5.8 杆臂、安装角和反馈状态问题

手机配置关闭 `estimate_leverarm`，杆臂为零；若手机天线与 IMU 实际分离明显，快速运动时零杆臂会产生系统误差，但通常不足以单独解释数百米。姿态反馈符号、ECEF/导航系 H 矩阵和状态单位错误则可能放大该误差，应在前述输入和脚本核对后检查。

## 6. 科学的后续工作安排

### 6.0 所有修改的统一收益判定协议（强制）

后续任何修改都必须先有可复现基线，再用同一精度评估脚本判断收益；没有评估结果不得进入下一轮，也不得凭轨迹图、日志片段或单个历元改善宣布成功。

#### 时间戳记号规范

所有实验记录同时保留两类时间戳，禁止混用：

- **数据历元时间 `Tdata`**：使用 `GPS week + SOW`（必要时同时记录 GPST/UTC 和 Unix 秒），用于标记观测、IMU、评估窗口、首次发散和首次恢复的实际数据时刻；手机原始时间戳已人工确认无误，但评估输出仍必须记录转换后的 `Tdata`。
- **实验操作时间 `Texp`**：使用本地时间 `YYYY-MM-DD HH:MM:SS+08:00`，用于标记配置快照、命令执行、评估完成、回退和向用户请示的实际操作时刻。

统一记号如下：`B0[Texp]` 表示基线建立时间，`CAND-xx[Texp]` 表示候选修改时间，`EVAL-xx[Texp]` 表示评估完成时间，`RB-xx[Texp]` 表示回退时间，`ASK-xx[Texp]` 表示请示时间；误差曲线和结果表中的 `first_bad[Tdata]`、`first_good[Tdata]` 分别表示首次恶化和首次恢复的数据历元。每个输出目录、配置快照、日志、CSV/JSON 汇总和报告标题都应包含实验编号及 `Texp`，避免同名结果覆盖或无法追溯。

时间记录最低要求：记录 `Tdata_start/Tdata_end`、评估窗口、`first_bad[Tdata]`、`first_good[Tdata]`、`Texp_before`、`Texp_after`；若触发回退或请示，还要记录 `Texp_rollback`/`Texp_ask` 及对应的上一通过版本。

1. **修改前基线**：冻结上一轮通过的配置、输入文件、命令、脚本版本和输出目录；先运行纯 RTD，再运行 RTD-TC。记录同一时间窗下 E/N/U RMSE、水平 RMSE、最大水平误差、有效历元数、GNSS 更新率、首次发散时刻和状态/解质量统计。
2. **单变量修改**：一次只修改一个变量或一个严格相关的变量族（例如仅 `select_fre` 模式，或仅一组过程噪声），不得同时改变频率选择、初始姿态和滤波权重。
3. **修改后评估**：使用完全相同的 `error_rtdtc.py`/手机评估适配、真值、时间窗、整数秒选择规则、坐标基准和异常点策略；手机仍严格先评估纯 RTD，再评估 RTD-TC。
4. **收益条件**：至少满足目标窗口水平 RMSE、全段水平 RMSE、最大误差和有效更新率均不恶化；若某一指标改善但另一关键指标恶化，不得判定为收益。对无绝对真值的手机实验，必须同时满足 RTD-TC 相对 RTD 的误差差值、漂移/跳变和更新连续性要求。
5. **恶化处理**：任一关键指标恶化、出现新的百米级跳变、有效 GNSS 更新明显减少、错误固定/错误观测增加，立即废弃该候选并回退到上一通过配置；保留失败输出和差异报告，但不得在失败候选上继续叠加修改。
6. **通过门槛**：只有“修改前/修改后对照表”明确显示收益，且纯 RTD 没有退化、RTD-TC 没有新增发散，才能把该配置标为新的基线并进入下一阶段。

### 6.0A RTKLIB 参考策略恶化时的请示停点

RTKLIB C 的观测挑选、频率映射或参数策略只是参考，不是可以无条件覆盖 gipylib 的最终答案。若完成逐观测对齐后，gipylib 按 RTKLIB 策略（包括 `select_fre: off` 自动选择）运行，结果反而比上一通过基线恶化，必须执行以下停点：

- 立即回退到上一通过的 gipylib 配置，保留 RTKLIB 对照结果、映射差异表、纯 RTD/RTD-TC 评估 CSV/JSON、日志和首次恶化历元；
- 不得自行改成 `select_fre: on`、继续调 INS 噪声或把恶化归因于手机数据质量；先确认 RTKLIB C 与 gipylib 的输入观测、参考站坐标、频点物理含义、周跳/粗差筛选和输出状态真的一致；
- 在规划文件中标记为“**需用户请示，留后续处理**”，向用户报告：采用的 RTKLIB 策略、修改前后纯 RTD 与 RTD-TC 指标、恶化幅度、首次恶化时间、观测映射差异和回退版本；
- 未获得用户下一步指示前，只允许做只读分析和报告补充，不再提交新的算法/配置候选。

该停点适用于“参考代码结果也恶化”的情况：它可能说明 RTKLIB 的观测选择并不适合当前数据、两套实现并非等价、或评估口径仍有差异，不能用“参考实现如此”替代收益证据。

### Phase 0：冻结并复现现象

- 保留 `phone/output/RTD.pos`、`RTDTC.rslt`、日志和当前配置快照。
- 明确纯 RTD 的“2 m 以内”和 TC 的“数百米”分别对应的时间窗口、统计量、有效点和坐标基准。
- 不改变任何参数，不覆盖原始结果。

### Phase 1：评估脚本审计与独立复算

- 先按第 3 节逐项核对 `error_rtdtc.py`、`eval_enu.py`、`error_rslt.py` 的时间、列、坐标和采样点选择。
- 用少量手工选定历元复算 RTD/TC 的 ECEF/LLH 到 ENU 误差。
- 统一输出纯 RTD、TC GNSS 更新点和 TC 全部机械编排点三组统计；脚本不通过则停止后续修改。

### Phase 1A：ignav 手机 RTD-TC 对等复算

- 使用 `ignav-debug/phone/rtdtc.conf`、`phone/output/rtdtc.rslt`、`phone/mate40ref.kf` 和相应评估脚本，复现约 10 m 级结果，并记录完整统计定义。
- 将 gipylib `RTDTC.rslt` 转换/解析到同一输出语义，使用同一真值、时间窗、整数秒点规则和 ENU 原点重新评估；不得直接比较两套程序各自生成的图像或 summary。
- 对 ignav 与 gipylib 同时评估纯 RTD 输入、GNSS 更新率、Q/Qins 状态、首次发散时刻和创新/协方差可用性，建立“CPT 优势反转、手机劣势反转”的证据表。
- 若统一复算后反差消失，优先修复评估口径；若反差保留，再进入 Phase 2 的数据流和配置差异定位。

### Phase 2：输入与数据流一致性

- 对比 RINEX 头、基站坐标、GPS 周/SOW、IMU 首尾时间和采样间隔。
- 检查 BDS `C2I` 解析、GPS/GAL/BDS 每历元卫星集合、载波/伪距有效率、周跳标志和 base 插值。
- 确认 TC 使用的 GNSS 量测与纯 RTD 的历元一一对应，并记录降级、拒绝和重初始化次数。

### Phase 2A：RTKLIB 观测选择复现与频率策略对照

- 先运行 `analyze_rtklib_frequency.py`，对 rover/base 的 RINEX 头和选定历元输出 RTKLIB C 的实际观测选择；特别核对 GPS `1C`、Galileo `1C`、BDS `2I/C2I`、GLONASS 频点和同频多码优先级。
- 对比三条链路：RTKLIB C 纯 RTD、gipylib 纯 RTD、gipylib RTD-TC。每条链路记录同一历元的卫星集合、主频索引、载波/伪距是否存在、实际 Hz 频率、双差对数和 rejected observation 类型。
- 先以候选副本运行 `select_fre: off`，完成“修改前纯 RTD → 修改后纯 RTD → 修改前 RTD-TC → 修改后 RTD-TC”的完整评估，再决定是否保留；之后才可在独立副本中以 `select_fre: on` 复现实验中的手动 `freq_ix/freq_table/dfreq_glo`。禁止在未输出映射差异表和收益对照表前直接调 INS 参数。
- 若 off 模式与 RTKLIB C 的观测集合一致而 on 模式不一致，优先判定为手动频率配置问题；若两种模式都不一致，进入 RINEX 解码、code2freq、观测槽布局和 rover/base 配对核查。
- 若 off 模式虽与 RTKLIB C 一致但纯 RTD 或 RTD-TC 精度恶化，执行 6.0A 停点，回退并向用户请示，不得自动选择 on 模式掩盖结果。

### Phase 3：最小化 TC 诊断

- 手机数据必须严格按“先纯 RTD、再 RTD-TC”的顺序运行：先固定 `select_fre` 策略，生成纯 RTD 输出并完成观测集合/平面精度评估；纯 RTD 通过后，使用同一观测选择策略运行 RTD-TC，再比较 TC GNSS 更新点和全部机械编排点。
- 在不改算法的前提下，设计对照运行：纯 RTD、TC 但固定/冻结 IMU 状态、TC 仅 GNSS 更新、TC 完整传播。
- 比较每次量测更新前后的位置、速度、姿态、创新、协方差和反馈增量，定位数百米误差首次出现的历元。
- 先验证时间/坐标/观测状态，再判断是初始化、机械编排还是量测更新导致。

### Phase 4：单变量修复实验

按以下顺序，一轮只改变一个变量族，并保留基线回退：

1. RTKLIB 观测码/频点自动选择与 `select_fre` 开关；
2. 基站坐标格式与 RINEX 头一致性；
3. 手机 BDS/频率映射及星座筛选；
4. IMU 坐标转换、初始对准阈值、姿态协方差和初始偏置；
5. 过程噪声与量测协方差；
6. 杆臂、时间同步估计和 H/反馈相关参数。

每轮必须同时评估纯 RTD 和 TC，记录水平 RMSE、最大误差、首次发散时刻、GNSS 更新率、创新拒绝率和状态协方差。任何“TC 比纯 RTD 好”但全段或更新点数量明显减少的结果均不得接受。

每轮实验必须填写以下最小记录；“收益”栏只有在第 6.0 节条件全部满足时才可填写“通过”：

| 实验编号/时间记号 | 修改前基线 | 单变量修改 | 纯 RTD 前/后水平 RMSE | RTD-TC 前/后水平 RMSE | 最大误差/更新率变化 | 是否回退 | 备注/请示状态 |
|---|---|---|---|---|---|---|---|
| B0[Texp] | 当前通过配置 | 无 | 记录 | 记录 | 记录 | 否 | 记录 `Tdata_start/end` |
| CAND-xx[Texp] / EVAL-xx[Texp] | B0 或最近通过版本 | 记录 | 记录 | 记录 | 记录 `first_bad/first_good[Tdata]` | 通过/回退 | 若 RTKLIB 参考恶化，标记“需用户请示，留后续处理” |

### Phase 5：验收标准

- 评估脚本通过时间、格式、坐标和独立复算核对。
- 手机 TC 不得出现无解释的百米级跳变或持续漂移。
- 在同一评价窗口、同一有效点规则下，TC 水平误差应回到与纯 RTD 同量级；若目标仍无法达到，必须给出可复现的传感器/观测限制证据。
- 任何修复不能牺牲有效 GNSS 更新率、扩大高度误差或引入错误固定。

## 7. 风险与回退规则

- 不得把评估脚本中的时间偏移、列号修正或异常点过滤与算法修复混在同一提交/实验中。
- 所有候选运行使用独立输出目录和配置副本；每次候选修改前后都必须先跑统一精度评估，并写入 `Texp_before/Texp_after` 和数据历元范围，失败或恶化立即回退到最近一个通过基线并记录 `RB-xx[Texp]`。
- 若纯 RTD 本身的评估不能复现 2 m 结论，先修复评估口径，不进入 TC 根因判断。
- 若 TC 在第一次 GNSS 更新前已产生百米误差，优先查时间、初始位置/姿态和机械编排；若首次更新后才跳变，优先查 H、R、创新门限、观测列和反馈符号。
- 若 RTKLIB 参考策略在输入和评估均已对齐后仍导致恶化，按 6.0A 标记“需用户请示，留后续处理”，记录 `ASK-xx[Texp]`、`first_bad[Tdata]` 和回退版本，回退后停止自主修改，等待用户决定是否继续深入。

## 8. 本轮交付边界

本轮仅新增本规划文件，记录问题现象、评定文件、候选原因和科学验证顺序；不修改 `phone/rtdtc.yaml`、`data/rtdtc.yaml`、评估脚本、算法代码、数据文件或已有输出。

## 9. 执行记录（2026-08-24，Phase 0/1/2A 部分 + Phase 3/4 完成）

### Phase 0：现象量化（统一 mate40ref.kf 参考核对）

| 输出 | H med | H RMSE | V rms | 备注 |
|---|---:|---:|---:|---|
| 纯 RTD.pos | 2.58 | 3.58 | 6.64 | 用户"2m 以内"≈中位口径 ✓ |
| RTDTC.rslt(8-7 旧) | 4.52 | 6.34 | 6.92 | **无数百米段**（全程 3-8m）|
| ignav rtdtc.rslt | 8.43e6 | — | — | 该文件损坏/口径不符，不可用 |

修正后的问题定义：不是"数百米"，而是 **TC 更新点比其输入纯 RTD 差 2.5 倍**
（更新点 H med 6.45 vs RTD 2.58；同历元 TC−RTD 差异中位 7-9 m）。

### 根因链（诊断实证）

1. 更新间垂直漂移 +3~5.5 m/s、创新范数中位 **51 m**（正常 ~3m）；
2. 更新前状态误差中位 27m 时大量码 DD 创新逼近 maxcode=30 门限被拒 → 修正受限，
   卡在 ~27m 误差平台（maxcode→60 实证改善：最差窗 28.5→16.2）；
3. 姿态在传播中发散（pitch ±20°/yaw ±100° 差异）：RTD-TC 码差量测对姿态零可观测
   （零杆臂 H[att]=0）+ 手机 MEMS 陀螺零偏 ~50-80°h；重力泄漏 g·sinθ ≈ 3+ m/s² 解释漂移率；
4. **BDS 路径额外毒化**（GPS+BDS 组合 V_rms=95m vs GPS+GAL 10.4m）；C2I 解码本身正常
   （槽位 0 有完整载波/伪距），系统性偏差来源待每卫星残差 dump 定位。

### Phase 4 候选记录

| 编号 | 改动 | 全段 H-RMSE | 最差窗 | 判定 |
|---|---|---:|---:|---|
| B0-phone[Texp=08-24 11:49] | 当前代码+当前 yaml 复现 | 14.454 | 28.544 | 可复现基线 |
| P1 | gyro_bias_psd→2.61e-9(对齐 ignav) | 14.384(V 36→36) | — | 回退 |
| P2 | vel_psd 1.0→0.05 | 15.196 | — | 回退 |
| P3 | accel_psd→2.60e-3(对齐 ignav) | 14.481 | — | 无效 |
| P4 | vel_psd→0.0003(CPT 值) | 14.909 | 33.5 | 回退 |
| BRK-P2 | NHC 开启 | 14.495 | — | 无效（触发 1206 次但 v 系假设轴与未知安装方向不符）|
| BRKP3/P4 | pos_psd/maxcode 组合 | 14.479 / **11.111** | 28.6/**16.2** | maxcode 有效 |
| **BRK-P5** | **gnss_t 去 BDS** | **7.691** | **12.437** | **主要修复** |
| **BRK-P6** | **去 BDS + maxcode=60** | **7.718** | **11.970** | **采纳** |
| BRKP11 | 一次性 yaw 对齐 + CPT vel_psd | 14.192 | 34.3 | 无持续收益 |

### 已采纳（phone/rtdtc.yaml）

- `gnss_t: ["GPS","GAL"]`（BDS 暂停，待残差专项）
- `maxcode: 60.0`

### FINAL-phone 验收[Texp=2026-08-24]

| 输出 | H med | H RMSE | H max | V rms |
|---|---:|---:|---:|---:|
| 改前可复现基线 | 11.29 | 14.45 | 61.1 | 35.97 |
| **改后正式输出** | **5.83（−48%）** | **7.72（−47%）** | **35.0** | **10.31（−71%）** |
| 纯 RTD 输入参照 | 2.58 | 3.58 | 28.8 | 6.64 |
| 8-7 历史(条件不明) | 4.52 | 6.34 | 43.0 | 6.92 |

回归测试 147 passed / 1 failed（既有无关）；纯 GNSS(RTD) 输出不受影响（改动仅 gnss_t 星座集与码门限，
RTD 自身评估前后一致）。

### 遗留与请示

1. **ASK-A**：8-7 输出曾达 6.34m 且含 BDS——当时的 yaml 参数组合无法复原（yaml 被 gitignore 无历史）。
   若用户留存当日配置可提供对照。
2. **ASK-B**：BDS 恢复需要先实现每卫星相位/伪距残差 dump 与 udbias/ddres 细节比对（新专项）。
3. **ASK-C**：根治垂直通道需姿态可观性——候选：RtdTcMeas 增加 Doppler 速度量测、或 v 系安装角标定后启用 NHC。
   三者均为 src/core 改动，待指示。

## 10. 最新 GPS+GAL RTD-TC 复算（2026-09-06）

根据当前已采纳的手机策略，`phone/rtdtc.yaml` 已统一为：

- `gnss_t: ["GPS", "GAL"]`，暂停存在系统性偏差的 BDS C2I；
- `maxcode: 60.0`；
- `ins.feedback_pos_enable: off`，采用 ignav 式位置立即反馈；`feedback_pos_fraction: 0.88`、
  `feedback_pos_smoothing_s: 1.0`、`feedback_pos_smoothing_mode: transverse` 仅作为 on 时参数保留。

按该配置重新运行 `src/main.py phone/rtdtc.yaml`，`phone/output/RTDTC.rslt` 共 `525743` 个高频历元，
Qins=0/2/3 为 `18/520533/5192`，GNSS Q=4/5 为 `525608/135`。使用 `phone/error_rslt.py` 的
固定窗口（Week 2382，SOW 115665--115765，Qins=3 统计）得到 H RMSE `4.211 m`、U RMSE
`7.325 m`、3D RMSE `8.449 m`；使用 `phone/eval_enu.py` 对全文件整数秒真值匹配得到 H RMS
`6.055 m`、U RMS `10.034 m`、3D RMS `11.719 m`，最大 3D 误差 `64.245 m`。

已重新生成 `phone/plot/error_plot.png` 和 `phone/plot/tra-mech.png`；后者已参考
`data/plot/tra-neu.py` 改为纯散点显示，真值高频内插点为绿色小点，RTD-TC 机械编排点为红色小点，
Qins=3 更新点为红色大点。`phone/eval_enu.py` 为统计脚本，不单独生成图片。
