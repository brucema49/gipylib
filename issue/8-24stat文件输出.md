# gipylib trace/stat 文件输出与 ignav 兼容指导规划

## 1. 任务范围与约束

本文件指导 gipylib 增加与 ignav/RTKLIB 可对照的 `trace` 和 `stat` 文件输出能力，服务于 `issue/8-24浮点解协方差差距.md` 的浮点协方差、残差、参考星和状态反馈分析。

本轮只新增本规划文件，不修改 Python、C/C++、YAML、数据、结果文件或评估脚本。后续执行必须先按本文完成设计确认和输出回归，再逐步修改实现；任何一轮输出能力或配置修改都必须先做精度评估，结果恶化立即回退。若直接对齐 ignav 语义后精度、稳定性或状态解释恶化，保留证据并记录 `ASK-STAT[Texp]`，向用户请示后暂停该方向。

输出目标采用“分层兼容”方案：

1. 基础 `stat` 记录使用 ignav/RTKLIB 的记录名和字段顺序，便于现有解析工具读取；
2. 无法与 ignav 状态一一等价的协方差、增益、残差和事件使用 `GIPY_*` 前缀扩展记录；
3. `trace` 负责连续调试日志，`stat` 负责结构化逐历元记录，二者不互相替代；
4. 不复制 ignav 已知的 `$ABIAS` 字段引用错误和 `$INSTA` 记录缺少换行的问题，兼容差异必须在文件头声明。

## 2. 现状与问题

### 2.1 当前 gipylib 输出能力

- `src/log/solution_writer.py` 输出纯 GNSS `.pos`；
- `src/log/rslt_writer.py` 输出组合导航 `.rslt`，包含位置、速度、姿态和部分标准差；
- `src/log/trace_file_writer.py` 已能按 `output.trace_level` 重定向 rtklib-py `trace()`，并将 Unix 时间转换为 GPS week/SOW；
- `src/log/trace_writer.py` 能输出部分 INS 状态调试信息，但尚未与统一 stat 文件生命周期和字段协议连接；
- `src/core/tc/tc_integration.py` 已有 `tc.diagnostics_path` CSV，记录部分更新前后协方差、增益、创新、模糊度和反馈；
- `skills/logANDoutput.md` 将 INS 状态输出列为预留，现有 `.pos/.rslt/.trace` 没有统一的 stat writer 和模式级输出契约。

### 2.2 ignav 参考输出

ignav 配置中：

- `file-file` 指定 trace 输出路径；
- `file-solstatfile` 指定 stat 文件路径；
- `out-outstat=1` 输出状态，`out-outstat=2` 输出状态和残差；
- `out-solformat=4` 表示 stat 格式，`outstr1-format=6` 表示 INS 结果格式；
- `ins-updint=1` 表示按 IMU 内部传播状态，stat 通常仍按解算历元输出。

已核对的基础记录包括：

| 记录 | 作用 | 参考字段 |
|---|---|---|
| `$POS` | 纯 GNSS 位置状态 | week、tow、quality、ECEF 位置、标准差/协方差相关字段 |
| `$VELACC` | 纯 GNSS 速度/加速度 | week、tow、quality、速度、加速度 |
| `$CLK` | 接收机钟状态 | week、tow、质量、钟偏及相关状态 |
| `$SAT` | 每颗卫星观测状态 | 卫星、仰角、方位角、残差、有效标志、固定标志、周跳和拒绝计数 |
| `$POSI` | INS ECEF 位置 | week、tow、INS 状态、位置及可选 float/fix |
| `$VELACCI` | INS ENU 速度和加速度 | week、tow、状态、E/N/U 速度和加速度 |
| `$ATT` | INS 姿态 | week、tow、状态、roll/pitch/yaw，单位 deg |
| `$GBIAS` | 陀螺零偏 | week、tow、状态、三轴零偏，单位 rad/s |
| `$ABIAS` | 加计零偏 | week、tow、状态、三轴零偏，单位 m/s² |
| `$INSTA` | INS 更新状态 | week、tow、`ista` 状态码 |

ignav 源码中 `$ABIAS` 的历史输出存在将 y/z 字段引用为 gyro bias 的缺陷，gipylib 不能复制该语义错误；ignav 某些 stat 文件中 `$INSTA` 后无换行导致下一条 `$POSI` 粘连，gipylib 必须保证每条记录独占一行，并在兼容说明中标注该差异。

## 3. 目标与非目标

### 3.1 目标

1. `off`、`lc`、`tc` 三种模式都有明确的 trace/stat 开关、路径、文件名、时间和覆盖语义；
2. 纯 GNSS 能输出可被 `error_rtdtc.py`、RTKLIB/ignav 解析链使用的基础 GNSS stat；
3. `lc/tc` 能输出 ignav 风格 INS 状态 stat，并标记更新点、传播点和初始化/重启状态；
4. `tc` 能输出量测残差、`P/K/S`、模糊度、参考星、周跳、降级和反馈事件，支撑浮点协方差差距分析；
5. trace 具备 0-3 级稳定语义，记录配置、输入、生命周期、异常和关键解算阶段；
6. 输出字段明确标注时间系统、坐标系、单位、状态码、版本和参数化边界；
7. 输出异常关闭、重复运行、flush、文件损坏和大文件增长均有可执行规则。

### 3.2 非目标

- 不在本任务中改变 GNSS 观测筛选、TC 量测方程、协方差传播、模糊度固定或 INS 反馈算法；
- 不把 gipylib 的 SD/DD 模糊度状态强行伪装成 ignav 的同名状态；
- 不以 stat 输出替代 `.pos/.rslt` 主结果，不改变现有评估脚本默认输入；
- 不为兼容历史解析器复制 ignav 的字段错误、缺少换行或未声明的单位转换。

## 4. 输出架构指导

### 4.1 统一输出会话

在主运行生命周期创建一个 `OutputSession` 概念，管理 trace writer、stat writer 和可选诊断扩展 writer；具体类名可沿用项目现有 `WriterBase` 风格，但职责必须分离：

1. `open()`：创建目录、写文件头、写 schema 版本和运行元数据；
2. `write_event()`：接收统一事件对象，按模式和级别分发；
3. `flush()`：按配置或关键边界刷盘；
4. `close()`：正常结束时写结束记录并关闭所有文件；
5. 异常退出时尽力写 `$GIPY_END`/`GIPY_RUN_ABORT`，不能阻塞主解算线程。

主输出 `.pos/.rslt` 与 `trace/stat` 必须共享同一 `output_dir` 和同一实验 stem，但不能共用文件句柄。默认命名建议：

| 模式 | 主文件 | trace | stat |
|---|---|---|---|
| `off` | `gnss_filename`，如 `RTK.pos` | `RTK.trace` | `RTK.stat` |
| `lc` | `rslt_filename`，如 `RTKLC.rslt` | `RTKLC.trace` | `RTKLC.stat` |
| `tc` | `rslt_filename`，如 `RTKINS.rslt` | `RTKINS.trace` | `RTKINS.stat` |

文件 stem 必须来自配置解析后的实际主文件名，不能依赖固定字符串；实验目录和文件名仍需遵循 `Texp` 及实验编号规则，禁止覆盖正式基线产物。

### 4.2 推荐配置语义

本轮不修改配置，只在规划中固定后续配置接口：

```yaml
output:
  output_dir: "data/output"
  gnss_filename: "RTK.pos"
  rslt_filename: "RTKINS.rslt"
  trace_level: 0              # 0=off, 1=info, 2=detail, 3=debug
  trace_enabled: true         # false 时强制不生成 trace
  trace_filename: ""          # 空值=主文件 stem + .trace
  stat_level: 0               # 0=off, 1=state, 2=state+residual, 3=state+residual+covariance
  stat_filename: ""           # 空值=主文件 stem + .stat
  stat_rate: "update"         # update=GNSS/约束更新点, imu=每个IMU点, second=整数秒附近
  stat_flush: "update"        # never/update/second/event
  overwrite: false             # false 时拒绝覆盖已有实验文件
```

配置校验必须拒绝未知的级别、速率、flush 模式和空文件名；`trace_enabled=false` 或 `trace_level=0` 时不得创建空 trace；`stat_level=0` 时不得创建空 stat。默认 `stat_rate=update`，用于与 ignav 的按解算历元对照；只有诊断协方差传播时才显式选择 `imu`。

### 4.3 时间与坐标协议

- 内部事件时间统一为 Unix 秒；文件记录统一输出 GPS week/SOW，除非配置明确选择 datetime；
- 所有记录头必须声明 `time_system=GPST`、SOW 精度和是否经过 UTC→GPST 转换；
- `$POSI` 使用 ECEF m；`$VELACCI` 使用 ENU m/s 和 m/s²；`$ATT` 使用 FRD roll/pitch/yaw deg；
- `$GBIAS` 使用 body-frame rad/s；`$ABIAS` 使用 body-frame m/s²；
- `P` 的单位按状态块声明：位置 m²、速度 m²/s²、姿态 rad²、gyro bias (rad/s)²、accel bias (m/s²)²；
- `P/K/S` 扩展记录必须带状态块索引、行列顺序和单位，不能只输出无标签矩阵数字；
- E/N/U 与 FRD 的方向约定必须写入文件头，并与 `.rslt` 的现有约定一致。

## 5. trace 文件指导

### 5.1 trace 级别

| 级别 | 内容 | 允许频率 |
|---|---|---|
| 0 | 关闭 | 不创建文件 |
| 1 | 启动/结束、配置摘要、输入范围、初始化、降级、重启、错误和关键解状态 | 事件级 |
| 2 | GNSS 历元、IMU/更新边界、观测数、残差统计、参考星、接受/拒绝原因、状态转换 | 历元级 |
| 3 | `F/Phi/Q`、`P` 健康度、`H/R/S/K` 摘要、模糊度同步、逐卫星诊断和异常前后快照 | 调试级，允许采样/限流 |

等级必须是内容过滤，不得改变滤波算法、观测顺序或随机数；trace 输出异常不能让解算线程崩溃。现有 `TraceFileWriter` 的 rtklib-py 重定向能力保留，新增应用层事件必须通过同一 writer 写入，避免一个运行生成多个不一致时间格式的日志。

### 5.2 trace 行格式

每行至少包含：

```text
<level> <week> <sow> <mode> <event> <message>
```

文件头声明 `schema=GIPY_TRACE_V1`、代码版本、配置路径/哈希、输入哈希、状态维度、时间系统和坐标系。`event` 使用稳定枚举，例如 `RUN_START`、`GNSS_EPOCH`、`IMU_PROP`、`TC_UPDATE_PRE`、`TC_UPDATE_POST`、`AMB_SYNC`、`REFSAT_SWITCH`、`SLIP_RESET`、`DEGRADE`、`REBOOT`、`COV_HEALTH`、`RUN_END`、`RUN_ABORT`。自由文本放在末尾，不能让解析器依赖中文标点或可变字段数量。

### 5.3 trace 必须记录的边界

1. 启动：配置摘要、输入文件、数据时间范围、输出路径和 trace/stat 级别；
2. 初始化：初始化开始/成功/失败、初始位置/速度/姿态质量和初始协方差摘要；
3. GNSS：历元时间、模式、解状态、卫星数、相位/码数、参考星和残差摘要；
4. TC：更新前后状态、更新是否接受、反馈范数、降级和重启原因；
5. 模糊度：初始化、同步、周跳/失锁、参考星切换和清零原因；
6. 数值健康：非有限值、非正定、Cholesky 失败、条件数超限和修补动作；
7. 结束：正常结束或异常中止、处理历元数、输出文件大小和最后一个有效时间。

## 6. stat 文件指导

### 6.1 文件头与记录规则

stat 文件头必须包含：

```text
# GIPY_STAT_V1
# time_system=GPST time_precision=0.001
# mode=off|lc|tc state_rate=update|imu|second
# position_frame=ECEF velocity_frame=ENU attitude_frame=FRD
# units: pos=m vel=m/s acc=m/s^2 att=deg gbias=rad/s abias=m/s^2
# qins: 0=GNSS-only/uninitialized 1=mechanization 2=mechanization+P propagation 3=measurement update
# compatibility: ignav-basic-records=yes gipy-extensions=GIPY_*
```

每条记录必须独占一行、以换行结束；字段缺失使用明确 `nan`/`-` 约定并在头部声明，不能使用列数变化表示缺失。记录顺序按时间稳定排序，同一历元的基础状态记录先于扩展记录。

### 6.2 `off` 纯 GNSS 模式

`stat_level>=1` 时输出 `$POS`、`$VELACC`、`$CLK` 和按 `stat_level>=2` 输出的 `$SAT`/`GIPY_RES`。必须包含：

- GPS week/SOW、质量状态、卫星数、位置和速度；
- ECEF 位置协方差或 ENU 标准差及非对角相关项；
- 钟差/钟漂状态（若实现可提供）；
- 每颗卫星的有效标志、仰角、方位角、相位/伪距残差、周跳、lock/outage/reject 计数和频点；
- 纯 GNSS 不能输出伪造的 `$ATT/$GBIAS/$ABIAS/$INSTA`，未初始化 INS 字段不得填零冒充有效状态。

### 6.3 `lc` 和 `tc` INS 基础记录

`stat_level>=1` 且 INS 初始化成功时输出：

- `$POSI`：INS ECEF 位置和质量/更新属性；
- `$VELACCI`：ENU 速度与加速度；
- `$ATT`：FRD 姿态 deg；
- `$GBIAS`：三轴 gyro bias rad/s；
- `$ABIAS`：三轴 accel bias m/s²；
- `$INSTA`：初始化、机械编排、更新、降级和重启状态码。

每组基础记录必须携带相同的 week/SOW 和状态码；`qins` 只能表示输出属性，更新点选择仍使用显式 `update_flag` 或等价的整数秒规则。若 INS 尚未初始化，只输出 GNSS 基础记录和 `GIPY_EVT,INIT_WAIT`，不得写零姿态和零偏作为有效估计。

### 6.4 TC 扩展记录

`tc` 且 `stat_level>=2` 时输出以下 `GIPY_*` 记录，记录名和字段顺序一旦确定必须版本化：

| 记录 | 内容 |
|---|---|
| `GIPY_EPOCH` | week、SOW、`update_flag`、qins、TC mode、GNSS quality、n_sv、n_phase、n_code、参考星、降级状态 |
| `GIPY_RES` | 先验/后验 residual 范数、相位/伪距分项 RMS、接受/拒绝原因、post-fit 标志 |
| `GIPY_COV` | `P_pos/P_vel/P_att/P_gbias/P_abias` 对角、关键交叉协方差范数、最小特征值、条件数、对称误差 |
| `GIPY_INNOV` | innovation 范数、`S` 对角摘要、NIS/门控阈值、有效观测数和拒绝数 |
| `GIPY_GAIN` | `K_pos/K_vel/K_att/K_gbias/K_abias/K_amb` 范数及反馈向量 |
| `GIPY_AMB` | 模糊度槽位卫星/频点、状态排序、初始化/清零、方差、相关性、周跳/失锁、参考星变化 |
| `GIPY_SAT` | 每颗卫星、频点、观测类型、仰角/方位角、phase/code residual、valid/fix/slip/lock/outage/reject |
| `GIPY_EVT` | 初始化、参考星切换、周跳、降级、reboot、数值异常、文件异常和回退相关事件 |

`stat_level=3` 才允许输出逐状态或逐卫星矩阵明细；默认只输出摘要，避免 100 Hz 全矩阵造成不可控文件增长。矩阵明细必须能由 `state_index` 或记录头恢复行列含义，不能只写 numpy 展平数组。

### 6.5 更新点、传播点与采样

- `stat_rate=update`：每个有效 GNSS/约束更新点输出一组状态和协方差；用于与 ignav `.stat` 对照；
- `stat_rate=second`：输出最接近整数秒的状态，明确 `update_flag`，不得用 Qins 代替更新识别；
- `stat_rate=imu`：每个 IMU 点输出基础状态，扩展协方差和矩阵只能按采样配置限流；
- TC 量测更新必须成对记录 `TC_UPDATE_PRE`/`TC_UPDATE_POST` 事件和相应 `GIPY_COV/GIPY_GAIN/GIPY_RES`，以便区分更新跳变与传播漂移；
- 同一时间的 GNSS 纯解、TC 更新和传播状态不能覆盖，使用 `source=gnss|tc_pre|tc_post|prop` 或等价字段区分。

## 7. 与 ignav 的字段映射与兼容边界

### 7.1 可直接对照字段

以下字段在统一时间、坐标和单位后可直接对照：

- `$POSI` 的 ECEF 位置；
- `$VELACCI` 的 ENU 速度和加速度；
- `$ATT` 的 FRD roll/pitch/yaw；
- `$GBIAS/$ABIAS` 的三轴状态，但必须确认单位和字段来源；
- `$INSTA` 与 `qins/update_flag` 的状态转换；
- `$POS/$VELACC/$SAT` 的 GNSS 质量、卫星和残差基础字段。

### 7.2 不能直接等价的字段

- gipylib 的完整 EKF `P` 与 ignav 的 `insstate_t.P` 可能状态顺序、误差定义和坐标系不同；先输出状态索引映射，再比较块级指标；
- gipylib TC 模糊度槽位可能采用卫星主序，rtklib/ignav 可能采用频率主序或 DD 重参数化；只能在共同基底变换后比较；
- `qins` 是 gipylib 输出属性，不是 GNSS 更新真值；`update_flag` 才是更新识别字段；
- ignav `$ABIAS` 历史实现的 y/z 引用错误不是兼容目标；gipylib 输出必须以真实加计零偏为准；
- ignav 某些 `$INSTA` 换行缺陷不是兼容目标；gipylib 文件必须可逐行解析。

## 8. 分阶段实现与验证顺序

### Phase S0：冻结输出回归基线

1. 保存当前 `output` 配置、主输出 `.pos/.rslt`、代码版本、输入哈希和现有 `trace_level` 行为；建立 `S0-STAT[Texp]`。
2. 在 `off/lc/tc` 各运行一次默认 `trace_level=0/stat_level=0`，确认主结果字节数、历元数、精度和解状态不变；空输出文件不得生成。
3. 明确 `Tdata_start/end`、处理线程、输出 stem 和异常结束行为，作为后续每个候选的基线。

### Phase S1：基础 trace 生命周期

1. 保留现有 rtklib-py trace 重定向，补充统一文件头、`RUN_START/RUN_END/RUN_ABORT` 和模式事件；不改变 trace 过滤条件之外的解算逻辑。
2. 逐级验证 `trace_level=1/2/3` 的内容增量、时间格式、flush、异常关闭和重复运行；级别提升不得改变 `.pos/.rslt`。
3. 先只在 `off` 模式验证，再在 `lc/tc` 验证；任何 trace 写入异常只记录并关闭日志，不得让定位线程崩溃。

### Phase S2：基础 stat 状态行

1. 增加统一 stat writer 生命周期和文件头；先实现纯 GNSS `$POS/$VELACC/$CLK/$SAT`，再实现 LC/TC 的 `$POSI/$VELACCI/$ATT/$GBIAS/$ABIAS/$INSTA`。
2. 验证每条记录换行、字段数、时间排序、单位和未初始化状态处理；与 ignav `.stat` 做逐历元记录数和关键字段对照。
3. 运行 `error_rtdtc.py` 和现有姿态/速度评估，确认增加 stat 不改变主输出精度、有效更新数、降级数和 `qins` 分布。

### Phase S3：TC 事件、残差和协方差摘要

1. 在已有 TC 更新边界接入 `GIPY_EPOCH/GIPY_RES/GIPY_COV/GIPY_INNOV/GIPY_GAIN/GIPY_EVT`；先只输出摘要，不输出全矩阵。
2. 对每个 GNSS 更新点保存 pre/post 状态、`P/K/S` 健康度、反馈、模糊度摘要、观测数和参考星；传播点只写 `qins=2` 基础记录，避免把更新值重复覆盖。
3. 用 `8-24浮点解协方差差距.md` 的 F0-F3 指标验证 stat 能复现现有 diagnostics CSV 的关键数值；发现不一致时先修输出语义，不调整滤波参数。

### Phase S4：逐卫星、逐模糊度和矩阵明细

1. 仅在 `stat_level=3` 和显式实验配置下输出 `$SAT/GIPY_SAT/GIPY_AMB` 明细及选定矩阵块；默认输出必须保持小规模。
2. 固定状态排序、频点、参考星、单位和 DD/SD 定义；每个矩阵快照包含行列索引和健康诊断。
3. 与 ignav/RTKLIB 同历元对比，定位 `first_cov_bad[Tdata]`、参考星切换和周跳前后的变化；不能用不同参数化的逐元素差异直接判定 bug。

### Phase S5：完整兼容验收

1. `off`：纯 GNSS `.pos + .trace + .stat`；
2. `lc`：`.pos + .aligned.csv + .rslt + .trace + .stat`；
3. `tc`：`.rslt + .trace + .stat`，并覆盖更新点/传播点、TC 残差和协方差摘要；
4. 验收文件可被现有脚本逐行读取，时间系统、坐标和状态码可追溯；
5. 主精度指标、输出历元数、GNSS 更新数、降级/重启数与 S0 基线一致；
6. 与 ignav 对照时明确“直接兼容字段”和“GIPY 扩展字段”，不宣称未转换的 `P`/模糊度矩阵完全一致。

## 9. 错误处理、性能与文件安全

- 默认禁止覆盖已有文件；实验必须使用独立输出目录或带实验编号的文件名；
- 写入采用临时文件后原子替换，异常终止保留已写内容并追加 `RUN_ABORT`；
- `flush=update/second/event` 的语义固定，`stat_level=3` 允许按事件采样但不能静默丢失关键异常；
- 所有浮点值使用有限值检查，`nan`/`inf` 必须显式输出并同步 `GIPY_EVT,COV_NONFINITE`；
- 大文件保护只允许限流扩展明细，不得删除基础状态、更新事件或异常记录；
- writer 不能持有滤波器可变状态的引用而延迟读取，事件入队时必须复制需要的状态快照；
- 多线程场景由单一输出线程/会话写文件，禁止多个 logger 直接交错写同一文件；
- 输出关闭必须在正常结束、输入耗尽、用户中止和异常路径均执行。

## 10. 记录模板与回退纪律

| 编号 | 改动范围 | 模式 | trace/stat 级别 | 主结果差异 | 文件解析 | 精度/更新数 | 结论 |
|---|---|---|---|---|---|---|---|
| S0-STAT | 输出关闭回归 | off/lc/tc | 0/0 | 记录实测 | 不适用 | 与基线一致 | 基线 |
| S1-xx | trace 生命周期 | off/lc/tc | 1-3/0 | 记录实测 | 通过/失败 | 记录实测 | 接受/回退 |
| S2-xx | 基础 stat 状态行 | off/lc/tc | 0/1 | 记录实测 | 通过/失败 | 记录实测 | 接受/回退 |
| S3-xx | TC 残差/协方差摘要 | tc | 2-3/2 | 记录实测 | 通过/失败 | 记录实测 | 接受/回退 |
| S4-xx | 逐卫星/矩阵明细 | tc | 3/3 | 记录实测 | 通过/失败 | 记录实测 | 接受/回退/请示 |

每轮候选都必须先评估输出关闭基线，再打开新增输出；如果主结果、精度、更新数、状态切换或运行稳定性恶化，保留输出和日志并回退到上一通过版本。若对齐 ignav 后结果恶化，记录 `ASK-STAT[Texp]`，暂停自行增加字段或修改状态语义。

## 11. 交付边界

本文只作为 gipylib trace/stat 输出实现的指导规划，不代表代码或配置已经修改，也不代表已生成新的 trace/stat 文件。完成 S0-S3 前不得进入算法调参；完成 S4 前不得宣称与 ignav 的逐卫星/协方差输出完全一致。后续任何实现改动必须引用本文件实验编号、保留失败产物，并以现有精度评估和统一字段解析结果作为验收依据。

## 12. 执行记录（2026-08-24，S0-S3 核心完成）

### 已实现

1. **新增 `src/log/stat_writer.py`** (`StatWriter`, GIPY_STAT_V1)：
   - 基础记录（stat_level≥1）：`$POSI/$VELACCI/$ATT/$GBIAS/$ABIAS/$INSTA`（TC/LC），
     `$POS`(+`$VELACC`)（off），字段顺序与 ignav 一致；
   - 不复制 ignav 缺陷：`$ABIAS` 三轴均为真实加计零偏；每条记录独占一行换行结束；
   - 扩展记录（stat_level≥2，TC 更新点）：`GIPY_EPOCH/GIPY_COV/GIPY_INNOV/GIPY_GAIN`
     （P 对角分块、min-eig/cond/对称误差、创新范数、K 范数、参考星）；
   - `GIPY_EVT,INIT_WAIT` 初始化等待事件；`$VELACCI` 加速度为速度有限差分（头部声明）；
   - 速率过滤 update/second/imu；nan/inf 显式输出。
2. **trace 生命周期**：`TraceFileWriter.write_event()`（`<level> <week> <sow> <mode> <event> <msg>`），
   文件头 `schema=GIPY_TRACE_V1`；main.py 写 `RUN_START`（含配置 md5/模式/stat_level）
   与 `RUN_END`（耗时）；写入异常不阻塞解算线程。
3. **output: 开关**（用户要求位置）：`stat_level`(0/1/2/3)、`stat_filename`("")=stem+.stat、
   `stat_rate`(update/second/imu)、`trace_enabled`；main.py 启动时校验非法值即报错。
4. **接线**：tc_stream / lc_stream 接受可选 `stat_writer` 并在三个写出口同步调用；
   tc_integration 每次量测更新存储 `last_update_info` 轻量快照（创新范数/K 范数/参考星等）；
   off 模式经 `_GnssStatTee` 复合器同步写 .pos 与 .stat。

### 验证（S0-S3）

| 项 | 结果 |
|---|---|
| S0 开关全关回归 | RTKINS.rslt md5 与基线一致（66814442...）；无空 stat/trace 生成 ✓ |
| S2 stat_level=1/2 (TC) | 1708 更新点 × 10 记录 + $POS 656 + INIT_WAIT×1；逐行换行、无粘连 ✓ |
| 主输出不变 | stat_level=2 与 trace_level=1 开启时 .rslt md5 仍与基线一致 ✓ |
| S3 数值交叉验证 | GIPY_INNOV 中位 1.040 == 诊断 CSV innovation_norm 中位 1.040（确定性管线）✓ |
| LC 路径 | 1708 $POSI/$GBIAS ✓（修复 write_gnss_only 关键字兼容） |
| off 路径 | 2489 $POS ✓ |
| trace 事件 | RUN_START(config md5/mode/stat_level) + RUN_END(elapsed) ✓ |
| 回归测试 | 147 passed / 1 failed（既有无关失败）✓ |

### 使用示例

```yaml
output:
  stat_level: 2          # 0=off 1=基础状态 2=+TC GIPY_* 摘要
  stat_rate: "update"    # update/second/imu
  stat_filename: ""      # 空=RTKINS.stat
  trace_enabled: true
  trace_level: 1
```

### 遗留（S4，待指示）

`stat_level=3` 的逐卫星 `$SAT/GIPY_SAT/GIPY_AMB` 明细与矩阵块输出未实现——
需要先在 TC 量测路径暴露逐卫星残差/模糊度槽位快照（涉及 tc_measurement 内部状态透传），
建议与 `8-24浮点解协方差差距.md` 的每卫星残差 dump 专项合并实施。
