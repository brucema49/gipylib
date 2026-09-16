# 9-15 gnutlib（LibGnut 频点映射/观测读取）移植与 rinex_improve 自动化

> 状态：已完成（含实测数据与遗留风险）。承接 `issue/9-15北斗频点映射.md`：
> 本轮把 LibGnut 的频点映射与观测值读取逻辑移植为 Python（`src/utility/gnutlib/`），
> 将 `rinex_simplifier.py` 升级为 `rinex_improve.py`（调用/参考 gnutlib），
> 信号映射默认自动完成：12 个活动配置只保留 `gnss_t`、`nf` 与
> `remove_sat: ["C01"]`；**按用户反馈第二轮恢复手动 `raw_band_priority`**
> （真实数据与 LibGnut 库级默认冲突时必须以配置固定频点，见第 6、9 节）。
>
> 验收（phone/rtd.yaml，GPS/GAL/BDS 多系统多频，5222 历元）：
> 自动方案 `G[1,5] E[1,5] C[2]`，5222/5222 历元有解（ns≈28），码 DD prefit
> 残差 mean −0.27 m / rms 8.73 m，|v|>50 m 占 **0.295%**，C01 故障历元全部消失。
> phone TC 与旧链路在同等输入下**逐位一致**（Qins=3 水平 4.085 m）；campus01
> 各变体实测见第 6 节（旧链路 1.145 m、自动 B1I 1.123 m、手动 B3I 1.272 m，
> 旧记录 0.548 m 不可复现）。

## 1. 任务与做法

用户指示：

1. 把 `GREAT-MSF/src/LibGnut` 的频点映射与观测值读取移植到
   `gipylib/src/utility/gnutlib`，兼容 pyrinrx 风格，但**处理逻辑第一位是 LibGnut**；
2. `gipylib/src/utility/rinex_simplifier.py` 升级为 `rinex_improve.py`，调用/参考
   gnutlib 处理 GNSS 观测值；
3. 更新 `phone/rtd.yaml` 并用它测试多系统多频观测值是否被正确处理（需残差统计）；
4. 测试成功后修改所有配置文件：删掉星座与信号配置里的信号映射（不该由用户完成），
   只留 `gnss_t` 与卫星剔除 `remove_sat: ["C01"]`。

做法：先只读梳理 LibGnut 源码（`gutils/gnss.{h,cpp}`、`gutils/gsys.{h,cpp}`、
`gdata/gobsgnss.{h,cpp}`、`gcoders/rinexo{2,3}.cpp`），再按「读取前端用 LibGnut、
解算后端保持 pyrinrx」的分工落地。

## 2. gnutlib：LibGnut 的 Python 移植（新增）

| 模块 | 对应 C++ | 内容 |
|---|---|---|
| `gnutlib/gnss.py` | `gutils/gnss.{h,cpp}` | `GSYS/GOBSTYPE/GOBSBAND/GOBSATTR/GOBS` 枚举（数值逐值一致，含 0–99 码 / 100–199 相位 / 200–299 多普勒 / 300–399 SNR / 1000+ RINEX2 遗留码 的区间语义）；`GNSS_BAND_PRIORITY`、`GNSS_FREQ_PRIORITY`、`GNSS_DATA_PRIORITY`、`GNSS_BAND_SORTED`、`GNSS_SATS`；`str2gobs/gobs2str/tba2gobs/gobs2band/pha2snr/char2gobsband…`；`RANGE_ORDER_ATTR_RAW`/`PHASE_ORDER_ATTR_RAW` 与 `select_attr()`（`select_range/select_phase` 的「属性串中位置最大者胜出」语义） |
| `gnutlib/gsys.py` | `gutils/gsys.{h,cpp}` + `gdata/gobsgnss.cpp` | 频率乘数因子（10.23 MHz / GLO 178 MHz）、`frequency(gs,band,channel)`、`wavelength`、`wavelength_L3/WL/NL`、`band2freq_seq/band_priority/freq_priority/band2gfrq/gfrq2band` |
| `gnutlib/rinexo3.py` | `gcoders/rinexo3.cpp` + `rinexo2.cpp` | 头部解析（`SYS / # / OBS TYPES` 13 信号/行含多续行、`SYS / SCALE FACTOR`、`SYS / PHASE SHIFT`、`GLONASS SLOT / FRQ #`）、BDS `fix_band`（≤3.03 `C1x→C2x`/`C3x→C6x`；≥3.04 `C7D/P/Z→C9D/P/Z`）、历元解码（每观测 16 列，14 列数值 + LLI + 强度位→dBHz，0 值丢弃，比例因子）、pyrinrx 风格槽位视图 `band_slot_view()` |

关键实现要点（与 C++ 对照）：

- GOBS 表由「频带基址 + 属性偏移」生成，复刻 `gnss.h` 的枚举数值（例：`C1C=2`、
  `C2I=17`、`C6I=41`、`L1C=102`、`S2I=317`、`P1=1000`），并保留 `C7D/P/Z→C9D/P/Z` 特判；
- `str2gobsband` 取 `s[1]`、`str2gobsattr` 取 `s[2]` 等易错细节按 cpp 原样；
- 观测域起始列偏移做成参数 `field_offset`（LibGnut 规范 3；仓库 pyrinrx 解码器与
  改写器历史上用 4，调用方可对齐，见第 6 节遗留项）。

## 3. rinex_improve：把「用户配置」变成「数据自检」

新模块 `src/utility/rinex_improve.py`（`rinex_simplifier.py` 保留兼容 API 与旧测试）：

```
SYS / # / OBS TYPES
  → parse_obs_header + fix_band                 # 频带归一化(LibGnut)
  → plan_stream_signals(rover, base, gnss_t, nf)
       共同波段 ∩ LibGnut GNSS_BAND_PRIORITY
       + band_consistency 码-相自检(防错波长, fail-open)
  → auto_freq_plan                              # 波段→gnutlib 频率→freq_table/freq_ix0/1/dfreq_glo
  → needs_improvement / improve_rinex           # 跟踪码选择 + C-L-D-S 交织重写
       select_attr(LibGnut 属性表) + code_availability 稀疏码剔除
  → resolve_stream_plan                         # 显式映射 > legacy freq_ix > 自动规划
```

数据自检两条（都是「LibGnut 优先 + 用数据兜底」）：

1. **码-相一致性**（`band_consistency`）：同波段用正确波长时 `Δ(C-λL)`（消历元公共项、
   跨星中位数）为 cm 级，用错波长为 m 级。用于验证 `fix_band` 的频带假设；
   阈值取「绝对 0.6 m 且 8×同系统最优候选」，只有候选 >1 个时生效，剔除后必须仍有余量
   （fail-open，避免手机噪声数据误杀）。
2. **观测码可用性**（`code_availability`）：LibGnut 属性表静态选择可能指向稀疏跟踪码
   （实测手机基站 GPS `L1M` 仅 25% 卫星有值、`C1M` 同理），按同 (波段, 类型) 分组的
   相对可用性剔除，避免丢掉多半个频的数据列。手机基站 GPS 由此从 `L1M` 回到 `L1C`
   （0.25 → 0.997 可用率），RTD 的 GPS slot0 std 9.23 → 10.03 m、粗差剔除 1093 → 189 次。

`resolve_stream_plan()` 是传感器唯一入口：配置缺 `freq_table/freq_ix0/freq_ix1/dfreq_glo`
时自动派生（不改传入配置），显式 `raw_band_priority` 仍作为高级出口优先。

## 4. 配置与代码改动

- `src/stream/{tc_gnss_sensor,internal_gnss_sensor}.py`：改用
  `rinex_improve.resolve_stream_plan / needs_improvement / improve_rinex`，
  自动方案在 `_run_impl` 解析（显式配置仍在 `__init__` 提前校验）。
- `src/core/gnss/rtklib_config_adapter.py`：`freq_table/freq_ix*` 改为可选（缺失即派生）；
  `remove_sat`（新规范键）与 `excsats`（遗留别名）合并，并用 `id2sat` 转成卫星号
  ——此前字符串与卫星号比较，`excsats` 实际是**静默无效**的。
- `src/utility/config_loader.py`：`remove_sat`/`excsats` 类型校验。
- 新增 `phone/plot/rtd_residual_stats.py`：纯 GNSS RTD 的逐系统×频点×卫星码 DD
  prefit 残差统计（输出 `phone/output/rtd-residual-{summary,by-satellite}.csv`）。
- 新增测试：`tests/test_gnutlib_libgnut_port.py`（34 项，枚举/频率/属性表/fix_band/列布局）、
  `tests/test_rinex_improve_autoplan.py`（9 项，自动方案/频率派生/幂等）。
- 更新 4 项旧契约测试到 LibGnut 语义（BDS `C1I` 在 v3.02 即 B1I→band 2；
  属性串末位优先使 GPS band2 选 `C2W` 而非 `C2X`；解码器本身不做频带归一化）。

配置改造（脚本一次性执行 + 人工清理残留注释）：`phone/{rtd,rtdtc,rtd_gps_bds_debug}.yaml`、
`data/{config,rtd,rtdtc,rtk-pure,rtk-ins紧组合,spp-ins-lc,spp-ins-tc}.yaml`、
`data-great/{rtktc-rate,rtktc-increment}.yaml` 共 12 个文件删除
`raw_band_priority/raw_signal_priority/freq_ix0/freq_ix1/freq_table/dfreq_glo`，
`excsats` → `remove_sat: ["C01"]`，只保留 `gnss_t`（+`nf`）。
未改动：`Data19_*/`（数据集归档与 bds_diagnosis 生成结果）、`.worktrees/*`（其他分支
worktree）、`library/pyrinex`（第三方）、`tests/fixtures/config_test.yaml`（保留 legacy 键覆盖）。

## 5. 测试与残差统计

**phone/rtd.yaml（nf=2，GPS+GAL+BDS）**

自动方案：`G[1,5] E[1,5] C[2]`；`freq_ix0={GPS:0,GAL:0,BDS:6}`、
`freq_ix1={GPS:2,GAL:2,BDS:3}`（B1I=1561.098 MHz 命中 freq_table[6]）。

`python3 phone/plot/rtd_residual_stats.py phone/rtd.yaml`：

| 系统×频点 | n | mean (m) | std (m) | rms (m) | max\|v\| (m) |
|---|---|---|---|---|---|
| BDS slot0 (B1I) | 62551 | −0.44 | 7.63 | 7.64 | 297.5 |
| GAL slot0 (E1) | 21245 | +0.60 | 11.21 | 11.23 | 218.8 |
| GPS slot0 (L1) | 28795 | −0.74 | 10.03 | 10.06 | 93.1 |
| GPS slot1 (L5) | 15039 | +0.10 | 5.62 | 5.63 | 95.5 |

整体 mean −0.27 m / rms 8.73 m，|v|>50 m 仅 0.295%（376/127630）；
逐卫星 top 偏差：C41 +8.9 m（已知 B1I 卫星相关码偏差）、C40 仅 10 行、
GAL02/GAL19 等为瞬时多路径；**C01 逐卫星表中已无任何行**（`remove_sat` 生效，
修复前 C01 曾有 3838 行、最大 +599600 m）。

双频有效性（同一配置 nf=2 vs nf=1 的 RTD.pos 对比）：二阶差分高频噪声
lat 1.64 → 2.36 m、lon 0.97 → 1.20 m、h 5.97 → 6.41 m（双频更平滑 7%–30%），
两次解算水平差异 RMS 2.38 m（两次独立码解的离散水平）。

**phone/rtdtc.yaml（TC，GPS+BDS，Qins=3 收敛段，真值 phone/mate40ref.kf）**

| 运行 | 输入 | 水平 RMS | 3D RMS |
|---|---|---|---|
| 旧代码 + 旧显式配置（BDS [2]、`excsats: []`） | 基准 | 4.085 m | 9.089 m |
| 新代码 + 自动方案 + `remove_sat: []` | 与基准**逐位一致**（n=4971） | 4.085 m | 9.089 m |
| 新代码 + 自动方案 + `remove_sat: ["C01"]`（最终配置） | C01 剔除 | 4.180 m | 9.237 m |

结论：**新读取器/自动方案与旧链路等价**（逐位一致），差异只来自按指示剔除 C01
（C01 属 GEO，多数历元健康，剔除后几何略降；issue/9-15 §6.1 原建议剔除 C40/C41）。

## 6. campus01：BDS 选频实测与「0.548 m 记录不可复现」更正

自动方案（LibGnut `GNSS_BAND_PRIORITY`：BDS B1I 优先）在 campus01 选到
`C[2]`（B1I），而历史标定配置用 `C[6]`（B3I）。全部变体都用同一评估口径
（`data-great/plot/evaluate_campus01.py`，truth-left 10 Hz、±0.051 s）：

| 变体 | BDS 频点 | E/N/U RMS (m) | 3D RMS | matched |
|---|---|---|---|---|
| 旧代码 + 旧显式配置（无 C01 剔除） | B3I | 0.8831 / 0.3951 / 0.6124 | **1.145 m** | 5620/6070 |
| 新代码 + 自动方案 + 无 C01 剔除 | B1I | 0.8138 / 0.5472 / 0.5630 | 1.131 m | 5600/6070 |
| 新代码 + 自动方案 + `remove_sat:["C01"]` | B1I | 0.8733 / 0.5364 / 0.4600 | 1.123 m | 5600/6070 |
| 新代码 + 手动 B3I + 无 C01 剔除 | B3I | 0.9382 / 0.4285 / 0.5867 | 1.187 m | 5600/6070 |
| 新代码 + 手动 B3I + `remove_sat:["C01"]`（恢复后的配置） | B3I | 0.9989 / 0.4437 / 0.6505 | 1.272 m | 5600/6070 |

**更正**：上一轮引用的「campus01 基线 3D 0.548 m（E/N/U 0.4500/0.2155/0.2267）」
在**当前代码与数据状态下不可复现**——旧代码配旧配置实测 1.145 m（本条为本轮
`git stash` 实测）。因此「自动 B1I 使 campus01 退化」的结论不成立：三种新方案
（1.123–1.187 m）与旧链路（1.145 m）同量级，自动 B1I 反而最优；手动 B3I 叠加
全局 `remove_sat: ["C01"]` 是最差组合（1.272 m）。

**频点物理判定（仍然成立）**：campus rover `C1I/L1I` 用 B1I 波长时码-相一致性
0.056 m、用 B1C 波长时 1.28 m ⇒ 该文件确实以 `C1I` 记录 **B1I**，LibGnut 的
`fix_band` 正确，历史配置注释「rover 仅 B1C/B2b/B3I」是误判。

**处置**：按用户决定**保留手动频点映射**（`data-great/rtktc-{rate,increment}.yaml`
恢复 `raw_band_priority: {GPS:[1], GAL:[1], BDS:[6], GLO:[1]}` 与
`raw_signal_priority`；`phone/rtdtc.yaml` 恢复 `{GPS:[1], BDS:[2]}`）。
注意当前全局 `remove_sat: ["C01"]` 会抵消 campus01 的手动 B3I 收益，
若要取该数据集最优值需单独评估是否剔除 C01（需用户决策）。

## 7. 测试与回归

```
# 新增/相关测试
pytest -q tests/test_gnutlib_libgnut_port.py tests/test_rinex_improve_autoplan.py \
          tests/test_rinex_simplifier.py tests/test_rinex_multisystem_band_mapping.py \
          tests/test_sensor_decoder_raw_band_mapping.py tests/test_sensor_raw_band_mapping.py
# 全量
pytest -q --ignore=tests/test_matrix_compare.py
```

- 相关 95 项全过；全量 682 passed / 8 failed。
- 8 个失败均为**改动前既有**（`test_formators` ×2、`test_increment_conservation`、
  `test_tc_increment_equivalence`、`test_internal_gnss_{spp,rtk}_e2e`、
  `test_rtklib_config_adapter::test_build_params_basic_attributes` 断言 `efact` 长度、
  `test_matrix_compare` 缺模块）——已用 `git stash` 在基线上逐一复现；
  `test_streamer_base::test_streamer_base_stops_on_shutdown` 为满载时抖动（单跑 8/8 通过）。
- 端到端：`python3 src/main.py phone/rtd.yaml`（103.9 s，5222 历元全部 Q=4）、
  `python3 src/main.py phone/rtdtc.yaml`（254 s）、campus01 increment（84 s）。

## 8. 遗留与后续

1. **列偏移约定不一致**：LibGnut/规范为「卫星号 3 列 + 16 列观测域」，仓库 pyrinrx
   解码器与 `rinex_improve` 改写器历史上用 4 列起点（对短数值靠右对齐可容忍，
   但会截掉满宽度数值的末位、并错位 LLI/SSD）。本轮不改（会动所有基线），
   gnutlib 读取器保持规范起点并提供 `field_offset` 参数；后续可在一次专门轮次里
   统一为规范起点并重跑基线。
2. **campus01 BDS 选频**（见第 6 节）需要产品或算法决策。
3. **BDS B1I 逐卫星码偏差**（C41 +8.9 m、IGSO +1~1.7 m）仍未建模；
   `remove_sat: ["C01"]` 只解决了 GEO 故障历元。
4. C40（10 行、+223 m）与瞬时多路径大残差（0.295%）仍由 `maxcode` 门限兜住。

## 9. 全量配置更新与四条链路实跑（第三轮）

**更新范围**：主工作区全部 21 个配置（`phone/*`、`data/*`、`data-great/*`、
`Data19_20201214_HG4930_CAR_Opensky/*`），统一为新 schema：

1. `remove_sat: ["C01"]`（原 `excsats` 重命名/补齐，现已在适配层转卫星号真正生效）；
2. 显式 `raw_band_priority` 固化——**由 `freq_table[freq_ix]` 的频率值反解 raw band**
   （gnutlib 频率表），而不是沿用 `LEGACY_FREQ_BAND_METADATA`。这一步修掉了 Data19
   的历史不一致：其 `freq_table[6]=1.561098 GHz`（B1I），而 legacy 表把 ix 6 解析成
   B3I；反解结果 GPS `[1,2]` / BDS `[2,7]` / GAL `[1,5]` / GLO `[1,2]`，与该数据集
   频率表语义一致。`phone/rtd.yaml` 例外：BDS 与基站仅 1 个共同波段（< `nf=2` 的
   长度校验），保持自动方案并在配置里注明。
3. 清理被 loader 明确禁止的键：Data19 的 `ins` 段废弃键
   （`pos_diff_vel_max_std`/`rtk_float_pos_std`）与 `*-tc` 配置中的松组合专属键
   （`pos_psd`/`vel_psd`/`pos_diff_vel_std`/`innov_reject_*`）——两者均无消费点，
   删除后 **21/21 配置可加载**（此前 4 个 TC 配置无法加载）。

未改动：`Data19_*/bds_diagnosis/results/*`（诊断脚本生成的产物配置）、
`.worktrees/*`（其他分支）、`library/pyrinex`、`tools/KF-GINS/config`、`tests/fixtures`。

**四条链路实跑**（同一份代码与配置）：

| 配置 | 运行时长 | 产物 | 指标 |
|---|---|---|---|
| `phone/rtdtc.yaml` | 250.2 s | `phone/output/RTDTC.rslt`（Qins: 2→498733 / 3→4986） | Qins=3 水平 **4.180 m**、3D 9.237 m |
| `data-great/rtktc-increment.yaml` | 79.1 s | `data-great/output/rtktc-increment/nav-100hz.csv` | truth-left 3D **1.272 m**（B3I + 剔除 C01） |
| `data-great/rtktc-rate.yaml` | 78.6 s | `data-great/output/rtktc-rate/nav-100hz.csv` | truth-left 3D **1.272 m** |
| `data/rtk-ins紧组合.yaml` | 109.5 s | `data/output/RTKINS.rslt`（173492 行，Qins=3 → 1708） | 正常收敛，无解算异常 |

四条链路均未出现 `ObservationMappingError`/KeyError，说明自动方案、显式手动映射、
`remove_sat` 三类输入在 rinex_improve 下都自洽。

## 10. 工作记录

- 2026-09-15 — [gipylib `dev-ubuntu`] 完成 gnutlib 移植（`gnss.py`/`gsys.py`/`rinexo3.py`）、
  `rinex_improve.py`（自动方案 + 频率派生 + 两条数据自检）、两个传感器与配置适配器改造、
  `remove_sat` 键与卫星号转换（同时修掉 `excsats` 静默无效）、`phone/plot/rtd_residual_stats.py`、
  34+9 项新测试；phone 多系统多频 RTD 验收通过（0.295% 大残差、C01 剔除生效），
  phone TC 与旧链路逐位一致；12 个活动配置改为只留 `gnss_t` + `remove_sat: ["C01"]`；
  `skills/conf.md` §3.8 重写为自动方案说明。campus01 自动改选 B1I 的精度影响已实测记录待决策。
- 2026-09-15（第二轮，按用户反馈）：
  1. **类型现代化**：`src/utility/gnutlib/*.py`、`src/utility/rinex_improve.py`、
     `phone/plot/rtd_residual_stats.py` 及两个新测试文件把 `typing.Dict/List/Tuple/Set`
     注解全部改为内建 `dict/list/tuple/set` 泛型（PEP 585），并清理对应 typing 导入。
  2. **保留手动频点映射**：`data-great/rtktc-rate.yaml`、`data-great/rtktc-increment.yaml`
     恢复 `raw_band_priority: {GPS:[1], GAL:[1], BDS:[6], GLO:[1]}` 与 `raw_signal_priority`；
     `phone/rtdtc.yaml` 恢复 `{GPS:[1], BDS:[2]}`；`phone/rtd.yaml` 与 data/*.yaml 原本
     无该键（自动方案与其 legacy 语义一致），保持自动。`freq_table/freq_ix*` 仍自动派生
     （按 Hz 查表，与手写等价，已核验 B3I=1268.52 MHz 命中同一频率）。
  3. **顺带修复潜伏缺陷**：某系统在流动站有观测但与基站无共同波段时（如 `data/config.yaml`
     的 GLO），自动方案不给它槽位，`freq_ix` 也随之缺失，而 `zdres_sat` 对每颗被解码卫星
     无条件访问 `nav.obs_idx[f][sys]` ⇒ KeyError。新增 `_frequency_plan_scope()` 让频率派生
     覆盖「流动站声明 ∩ gnss_t」的全部系统（解码槽位不变），并加回归测试
     `test_frequency_plan_covers_decodable_systems_without_common_band`。
  4. campus01 全变体重测 + 更正旧记录（见第 6 节）：0.548 m 不可复现，旧链路 1.145 m；
     自动 B1I 1.123 m 为当前最优，手动 B3I + `remove_sat:["C01"]` 1.272 m。全量测试
     684 passed / 7 failed（7 项均为改动前既有）。
- 2026-09-16（第三轮，按用户反馈）：
  1. 全量 21 个配置统一 schema：按**频率值反解**固化显式 `raw_band_priority`
     （修掉 Data19 `freq_table[6]`=B1I 却被 legacy 表解析成 B3I 的历史不一致），
     `remove_sat: ["C01"]` 补齐/去重；`phone/rtd.yaml` 因 BDS 共同波段数 < nf 保持自动。
  2. 清理禁止键（Data19 `ins` 废弃键、TC 配置里的松组合专属键）→ 21/21 可加载。
  3. 实跑 4 条链路（phone TC、campus01 rate/increment、data RTK-INS TC）均正常收敛，
     指标见第 9 节。
  4. 类型现代化补充：`Optional[X]` → `X | None`、补齐缺失参数注解，新文件 lint 清零。
