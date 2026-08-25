# Data19 HG4930 Opensky 解算与 BDS-3 验证设计

## 1. 目标与范围

本任务规划对 `Data19_20201214_HG4930_CAR_Opensky` 数据集完成可复现的 IMU 解码、GNSS/INS 解算和精度分析，覆盖 `gipylib` 与 `ignav-debug` 两套程序。当前阶段只新增数据集专属脚本、配置、运行记录和报告，不修改算法源代码或现有共享配置。

实验矩阵由两部分组成：

1. 主矩阵：两套程序各运行 RTK、RTK-TC、RTK-LC，主星座配置固定为 GPS+BDS。
2. BDS-3 专项：两套程序的纯 GNSS 和 TC 分别运行 `GPS-only`、`BDS-only`、`GPS+BDS`；LC 仅对主矩阵的 GPS+BDS 配置做端到端检查。

精度主参考为 `HG4930_GroundTruth.txt`（IMU 参考点），并使用给定 IMU→GNSS 杆臂和安装旋转转换到天线点，与 `ROVE_GroundTruth.txt` 交叉验证。

## 2. 数据流与时间、坐标基准

原始数据只读自数据集目录：

```text
HG4930.imr
  -> Python IMR 解码器
HG4930_euroc.csv (EuRoC, RFU, 100 Hz)
  -> gipylib / ignav 输入层
ROVE.20O + BASE.20o + brdm3490.20p (+ SP3/CLK)
  -> RTK / TC / LC
  -> 统一评估器 + HG4930/ROVE 真值
```

解码器必须读取 IMR 头中的端序标志、版本、delta 模式、采样率、陀螺/加速度比例因子和时间偏差。本数据预期为 8.703114 版本、100 Hz、角增量和速度增量模式，记录布局为 `double tow + 6×int32`，头长 512 字节。

EuRoC 文件使用 UTC Unix 纳秒时间戳：

```text
timestamp_ns = (315964800 + week*604800 + tow - 18) * 1e9
```

减去 18 秒是为了匹配两个消费者：ignav 的 EuRoC 读取器会执行 `utc2gpst(+18 s)`，gipylib 的 EuRoC formator 会执行等价的 GPST 对齐。评估器始终将时间转换为 `(GPS week, sow)` 后再配对，禁止直接以未经说明的 Unix 日期比较。

IMR 原始数据保持 RFU；配置显式声明 RFU，由两个程序输入层转换到各自内部 FRD。使用 `lever arm = [0.044, -0.005, 0.182] m`（RFU）和安装旋转 `[0, 30, 90] deg`。姿态误差统一输出 roll/pitch/yaw（deg），yaw 残差包络到 ±180°。

## 3. IMR 解码器设计

在 `gipylib/Data19_20201214_HG4930_CAR_Opensky/imu_decode/` 增加数据集专属 Python 脚本，参考 `imr2euroc.m` 但不复制 MATLAB 的时间歧义。脚本职责：

- 按端序标志解析 512 字节头；端序错误或 magic 不匹配立即失败；
- 读取并校验完整 32 字节记录，处理 GPS 周内秒回绕、`dTimeTagBias`、重复和乱序时间戳；
- 在 delta 模式下将角增量、速度增量换算为 rad/s 和 m/s²；
- 保留 RFU 轴顺序，不做第二次轴交换；
- 写出带注释头的 EuRoC CSV，并记录 GPS week、时间范围、采样周期统计和输入哈希；
- 对首、中、尾抽样记录与 MATLAB 解码公式交叉核对；
- 检查加速度幅值、重力方向、有限值、极值和记录连续性。

解码验收条件：记录数等于文件大小推算值；首记录约为 GPST 2136/95007.964691；采样间隔接近 0.01 s；重力方向符合 RFU；不存在 NaN/Inf 或静默丢记录；gipylib 与 ignav 使用的 EuRoC 文件字节内容一致。

## 4. 配置与运行资产

### 4.1 gipylib

在数据集目录增加：

- `gipylib-rtk.yaml`
- `gipylib-rtktc.yaml`
- `gipylib-rtklc.yaml`
- `gipylib-bds-gps.yaml`、`gipylib-bds-bds.yaml`、`gipylib-bds-gpsbds.yaml`
- `gipylib-bds-gps-tc.yaml`、`gipylib-bds-bds-tc.yaml`、`gipylib-bds-gpsbds-tc.yaml`

配置统一指定 RINEX、广播星历、SP3/CLK、基站 ECEF、EuRoC IMU、100 Hz、RFU 和处理时间。HG4930 误差模型从 `IMUErrorModel.txt` 转换到 gipylib 的 SI/PSD 字段，所有转换在注释和报告中保留。主实验关闭未经验证的自适应选项；RTK、TC、LC 的 AR、NHC/ZUPT/ZARU、杆臂估计开关固定并记录。

### 4.2 ignav-debug

在对应数据集目录增加等价配置：

- `ignav-rtk.conf`、`ignav-rtktc.conf`、`ignav-rtklc.conf`
- `ignav-bds-gps.conf`、`ignav-bds-bds.conf`、`ignav-bds-gpsbds.conf`
- `ignav-bds-gps-tc.conf`、`ignav-bds-bds-tc.conf`、`ignav-bds-gpsbds-tc.conf`

使用现有 `navapp/rtkrcv` 入口和 b34 BDS 配置字段，确保 `pos1-navsys`、BDS 频率/信号码、`ins-tc`/`ins-lc`、EuRoC 输入格式、杆臂和 HG4930 噪声参数与 gipylib 的实验语义一致。

每个 case 的输出写入 `results/<program>/<case>/`，保存配置副本、stdout/stderr、trace/stat、原始结果和运行元数据。元数据至少包含输入文件 SHA-256、程序 git commit、命令行、时间范围和解码器版本。

## 5. 统一评估设计

评估器读取 `.pos`/`.rslt` 和两份真值，完成：

- GPS week/sow 对齐；解算历元落在相邻真值历元之间且距两端均不超过 1.01 s 时线性插值，否则不纳入精度统计并计入未匹配覆盖率；
- ECEF 残差转 ENU；
- 位置 E/N/U、水平、3D 的 RMSE、均值、标准差、最大值和 95% 分位；
- 速度误差（ECEF 与 ENU）同样统计；
- roll/pitch/yaw 姿态误差的 RMSE、均值、95% 分位和最大值；
- 全时段、去除初始化窗口、有效 GNSS 更新点三种统计口径；
- 覆盖率、有效历元、Q/状态分布、卫星数、AR 固定率、重启/降级次数和时间残差；
- IMU 点结果到 GNSS 天线点的杆臂转换，并与 ROVE 真值进行交叉一致性检查。

报告至少包含：汇总表、逐历元误差 CSV、位置/姿态误差曲线、卫星数与解状态曲线、BDS 对比表，以及失败 case 的首个异常时间和日志索引。不得只用单一 RMSE 判断成功。

## 6. BDS-3 正确性判据

对每个程序的纯 GNSS 和 TC 分别比较 GPS-only、BDS-only、GPS+BDS：

1. 输入层：RINEX 中 C 卫星及 C1I/L1I、C7I/L7I 观测被识别，卫星/历元计数与参考解析一致。
2. 解算层：BDS-only 有持续有效解；GPS+BDS 的可用卫星数和有效历元不低于 GPS-only；无 NaN、时间跳变或异常 ECEF。
3. 组合层：BDS 加入不造成 TC 持续发散、错误降级或姿态突跳。
4. 精度层：将误差、解状态、AR/固定率与 GPS-only 对比，区分 BDS 解码错误和滤波权重差异。

## 7. 执行顺序与失败回退

1. 校验两个数据目录的原始文件哈希和 RINEX/真值时间范围。
2. 运行 IMR 解码与抽样交叉校验，生成规范 EuRoC。
3. 先运行两套程序的纯 RTK，确认 GNSS 输入、时间和 BDS 观测正常。
4. 在同一 IMU 文件上运行 RTK-TC，再运行 RTK-LC。
5. 运行 BDS 专项 12 组（两程序 × 两模式 × 三星座组合）。
6. 统一评估、生成图表和汇总报告，复核所有 case 的日志与元数据。

若出现时间错位、卫星观测解析失败、首段初始化失败或结果发散，停止扩展矩阵，保留失败配置/日志/统计，先定位到解码、时间、BDS 输入、初始化或滤波配置层。整个过程不修改算法源码；任何必要适配仅通过数据集专属配置或评估脚本完成。

## 8. 预期交付物

- Python IMR→EuRoC 解码脚本及解码摘要；
- 两个数据目录内的 18 个配置文件和分目录结果；
- 每个 case 的运行元数据、日志、原始结果和评估 CSV；
- 统一精度评估脚本及位置/姿态/BDS 图表；
- `HG4930` 主真值、`ROVE` 杆臂交叉验证报告；
- gipylib 与 ignav 的 RTK/TC/LC 及 BDS-3 对比总结。
- `Data19_20201214_HG4930_CAR_Opensky/skill/first_compute.md`：包含环境检查、解码、逐组配置与命令、验收判据、结果索引和故障停止条件的最终运行指导。
