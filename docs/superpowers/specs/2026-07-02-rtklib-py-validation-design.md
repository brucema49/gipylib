# rtklib-py 直跑结果与 gipylib 输出对比验证设计

> 日期: 2026-07-02
> 主题: 用 rtklib-py 直接跑 SPP/RTK，与 `output/test_spp.pos`、`output/test_rtk.pos` 比对，定位并修复差异
> 范围: 仅 SPP 与 RTK 两种模式的算法结果验证；不涉及 INS、PPP、紧组合

---

## 1. 背景与目标

### 1.1 当前状态

gipylib 已在 `src/core/gnss/` 实现 SPP/RTK 处理器，薄封装 `library/rtklib-py/src/` 的 `pntpos`/`relpos`。已有产出：

- `output/test_spp.pos`：7月2 10:03 生成，**首历元 GPS 周 2569（异常，应为 2046）**——疑似 `solution_converter.py` 修复前生成的 stale 文件
- `output/test_rtk.pos`：7月2 13:10 生成，首历元 GPS 周 2046（正确），但相比 rover RINEX **缺失 18 个历元**

已有调试脚本（项目根目录，未提交）：

- `_debug_missing.py`：找出 test_rtk.pos 缺失的 18 个历元
- `_debug_rtk_fail.py`：分析这 18 个失败历元的 trace
- `_debug_time.py`：对比 SPP 文件的时间转换

rtklib-py 自带 `run_ppk.py` 仅支持 PPK（RTK），无 SPP 入口；其默认 `config_f9p.py` 参数（`cnr_min=[35,35]`、`maxout=20`、`eratio=[300,300]`、`freq_ix1={GPS:1, GLO:5, GAL:3}` 等）与项目 YAML **不一致**。

### 1.2 目标

1. 用 rtklib-py 的 `pntpos`/`rtkpos` 直接跑同一份数据，生成 reference `.pos`
2. 与 `output/test_spp.pos` / `output/test_rtk.pos` 逐历元比对
3. 找出差异根因，修复 gipylib 直到两边一致
4. **一致标准**：平面位置误差 < 0.5m（lat/lon < 5e-6 度），高程误差 < 1.0m；Q 与 ns 整数精确一致；历元数一致

### 1.3 非目标

- 不验证 `library/rtklib-py/` 本身的算法正确性（只验证 gipylib 封装是否忠实复现）
- 不修改 `library/rtklib-py/` 任何文件
- 不引入新定位模式（PPP/紧组合）
- 不重写 RINEX 流式解码

---

## 2. 架构与组件

### 2.1 新增文件

```
tools/verify_rtklib_py.py    # 直接跑 rtklib-py，产出 reference .pos
tools/compare_pos.py          # 逐历元 diff 两个 .pos
output/verify_spp.pos         # rtklib-py 直跑 SPP 结果（运行时生成）
output/verify_rtk.pos         # rtklib-py 直跑 RTK 结果（运行时生成）
output/diff_spp.txt           # SPP 比对报告（运行时生成）
output/diff_rtk.txt           # RTK 比对报告（运行时生成）
tests/test_compare_pos.py     # compare_pos 单元测试
```

### 2.2 组件职责

**`tools/verify_rtklib_py.py`** — rtklib-py 直跑器

职责：完全绕过 gipylib 的 Sensor/Queue/Writer 路径，直接调用 rtklib-py 的算法函数。复用 gipylib 的 `RtklibEnv` 与 `rinex_simplifier` 以消除配置与 RINEX 预处理这两个噪声变量。

```python
def run_spp(cfg_path: str, out_path: str) -> None:
    """SPP 路径: 直接循环 pntpos, 用 rtklib-py savesol 写出."""
    # 1. RtklibEnv(gnss_cfg).setup()
    # 2. rinex_simplifier 预处理 rover
    # 3. rnx_decode(rover).decode_obsfile + decode_nav
    # 4. for obsr in rov.obslist:
    #        sol = pntpos(obsr, nav)
    #        if sol.stat != SOLQ_NONE: sols.append(sol)
    # 5. savesol(sols, out_path)  # 复用 rtklib-py 自己的 savesol

def run_rtk(cfg_path: str, out_path: str) -> None:
    """RTK 路径: 直接调 procpos, 与 run_ppk.py 同一路径."""
    # 1-3. 同上, 加 base
    # 4. sol = procpos(nav, rov, base, fp_stat)
    # 5. savesol(sol, out_path)
```

命令行接口：`python tools/verify_rtklib_py.py {spp|rtk} [cfg_path]`

**`tools/compare_pos.py`** — .pos 比对器

职责：解析两个 `.pos` 文件，按时间戳对齐，逐历元逐列 diff，输出报告。

```python
def compare(test_path: str, ref_path: str, out_path: str) -> int:
    """返回 0=一致, 1=存在不一致."""
    # 1. 解析两文件, 跳过表头
    # 2. 按 (week, sow) 对齐历元
    # 3. 逐历元逐列 diff, 应用容差
    # 4. 写报告: 总历元/匹配/不一致/仅一边有的历元/首个不一致详情/列级统计
```

容差表：

| 列 | 容差 | 说明 |
|----|------|------|
| week, sow | 0 (整数精确) | 时间戳必须完全一致 |
| Q, ns | 0 (整数精确) | 解算质量与卫星数必须一致 |
| lat, lon | 5e-6 度 | ≈ 0.5m 平面 |
| height | 1.0 m | 高程 |
| sdn/sde/sdu/sdne/sdeu/sdun | 1e-4 m | 诊断信息，不卡一致但记录 |
| age, ratio | 1e-2 | 诊断信息，不卡一致但记录 |

命令行接口：`python tools/compare_pos.py <test.pos> <ref.pos> [--out report.txt]`

### 2.3 不改动的现有文件

- `library/rtklib-py/` 全部（只读）
- `src/` 下所有现有模块（验证阶段不预设改动；调试阶段发现 bug 时再针对性修复，每次修复都更新此 spec 的"差异根因记录"章节）

### 2.4 与 gipylib 的边界

`verify_rtklib_py.py` 仅复用两个 gipylib 模块：
- `src/core/gnss/rtklib_config_adapter.py` 的 `RtklibEnv`（cfg 注入 + sys.path 管理）
- `src/utility/rinex_simplifier.py` 的 `needs_simplification` + `simplify_rinex`

不导入 `src/core/gnss/spp_processor.py`、`rtk_processor.py`、`solution_converter.py`、`src/log/solution_writer.py`、`src/stream/internal_gnss_sensor.py`——这些正是被验证的对象。

---

## 3. 数据流

### 3.1 验证主流程

```
data/cfg_test_spp.yaml ─┐
                        ▼
    tools/verify_rtklib_py.py spp
        │
        ├─ RtklibEnv(gnss_cfg).setup()
        │   └─ 注入 __ppk_config、加 sys.path、init tracelevel
        ├─ rinex_simplifier.rover → 临时简化文件 (如需)
        ├─ rnx_decode(rover).decode_obsfile + decode_nav
        ├─ for obsr in rov.obslist:
        │     sol = pntpos(obsr, nav)
        │     if sol.stat != SOLQ_NONE: sols.append(sol)
        ├─ rtklib-py postpos.savesol(sols, "output/verify_spp.pos")
        └─ RtklibEnv.cleanup() + 删临时文件

data/cfg_test_rtk.yaml ─┐
                        ▼
    tools/verify_rtklib_py.py rtk
        │
        ├─ RtklibEnv(gnss_cfg).setup()
        ├─ rinex_simplifier.rover + base → 临时简化文件 (如需)
        ├─ rnx_decode(rover).decode_obsfile + decode_nav
        ├─ rnx_decode(base).decode_obsfile
        ├─ nav.rb = base.pos  (若 nav.rb[0]==0)
        ├─ rtklib-py postpos.procpos(nav, rov, base, fp_stat)
        │   └─ 内部: firstpos → rtkpos 循环 → relpos 逐历元
        ├─ rtklib-py postpos.savesol(sol, "output/verify_rtk.pos")
        └─ cleanup
```

### 3.2 比对流程

```
output/test_spp.pos  ─┐
output/verify_spp.pos ─┤
                       ▼
    tools/compare_pos.py
        │
        ├─ 1. 解析两文件, 跳过表头, 每行 split() → 15 列
        ├─ 2. 按 (week, sow) 建索引, 找出仅一边有的历元
        ├─ 3. 共有历元逐列 diff:
        │     - week/sow/Q/ns: 整数精确
        │     - lat/lon: |Δ| < 5e-6 度
        │     - h: |Δ| < 1.0 m
        │     - sd*: |Δ| < 1e-4 m (诊断)
        ├─ 4. 写报告 output/diff_spp.txt:
        │     - 总历元数 / 匹配数 / 不一致数 / 仅一边有的历元数
        │     - 首个不一致历元: week sow test_vals ref_vals Δ
        │     - 不一致按列分布: {lat: 5, lon: 3, h: 12, Q: 2, ...}
        │     - 平面/高程最大误差统计
        └─ 5. exit 0 (全一致) / 1 (存在不一致)
```

---

## 4. 错误处理

| 错误场景 | 处理 | 退出码 |
|---------|------|--------|
| `RtklibEnv.setup()` 失败（cfg 字段缺失） | 打印缺失字段名 | 2 |
| `decode_obsfile` 报 "Obs file too complex" | 说明简化器未生效，检查 `needs_simplification` 逻辑 | 3 |
| SPP 某历元 `pntpos` 抛异常 | 打印历元时间戳，跳过该历元，继续 | 0（仍写出已成功历元） |
| RTK `procpos` 抛异常 | 整体失败，打印异常 | 4 |
| `compare_pos.py` 一边文件不存在 | 提示先跑 `verify_rtklib_py.py` | 5 |
| 两边历元数差异 > 10% | 报告加粗警告"历元数差异过大，可能是配置或 RINEX 不一致" | 1 |
| 单历元 Q 不同但位置在容差内 | 仍标记为"不一致"（Q 是关键诊断信息），独立统计 | 1 |
| 临时简化文件清理失败 | 警告但不影响退出码 | 同主流程 |

---

## 5. 测试与验证策略

### 5.1 单元测试

| 模块 | 测试文件 | 测试内容 |
|------|---------|---------|
| `compare_pos` | `tests/test_compare_pos.py` | mock 两个 `.pos` 字符串：完全一致 / 平面超容差 / 高程超容差 / Q 不同 / 历元数不同 / 一边缺历元 |

不为 `verify_rtklib_py.py` 写单测——它是工具脚本，端到端验证即可。

### 5.2 端到端验证流程

```bash
# 1. 跑 rtklib-py 直接出 reference
python tools/verify_rtklib_py.py spp
python tools/verify_rtklib_py.py rtk

# 2. 跑 gipylib 出被测
python src/main.py data/cfg_test_spp.yaml
python src/main.py data/cfg_test_rtk.yaml

# 3. 比对
python tools/compare_pos.py output/test_spp.pos output/verify_spp.pos --out output/diff_spp.txt
python tools/compare_pos.py output/test_rtk.pos output/verify_rtk.pos --out output/diff_rtk.txt
```

### 5.3 已知差异的预期处理

**差异 1：SPP week=2569（应为 2046）**
- `verify_spp.pos` 跑出来 week=2046，与 `test_spp.pos` 的 week=2569 直接不匹配 → 报告"仅一边有的历元"会列出全部历元
- 根因：`test_spp.pos` 是 `solution_converter.py` 修复前生成的 stale 文件
- 修复：重跑 `python src/main.py data/cfg_test_spp.yaml` 覆盖 `test_spp.pos`，应自动得到 week=2046
- 验证：再次跑 `compare_pos.py` 应一致

**差异 2：RTK 缺 18 个历元**
- `verify_rtk.pos`（用原版 `procpos`/`rtkpos` 循环）应输出全历元
- `test_rtk.pos` 缺 18 个 → 报告"仅 verify 有的历元"列出这 18 个
- 根因：`RtkProcessor.process_epoch` 的循环结构偏离 `rtkpos()` 原版——`_prev_t` 在 `relpos` 之后才更新，而原版在 `relpos` 之前用上历元的 `t` 计算 `nav.tt`
- 修复：调整 `RtkProcessor` 的 `nav.tt` 计算时机对齐 `rtkpos()`（具体修复方案在调试阶段确定）
- 验证：再次跑 `compare_pos.py` 应一致

### 5.4 调试循环策略

**SPP 不一致时**：
1. 看 week/sow 是否对得上 → 不对则修 `solution_converter.py` 的 time 转换
2. 对得上但 lat/lon 偏 → 检查 `SppProcessor.process_epoch` 是否有跨历元状态污染（SPP 应每历元独立）
3. Q 不同 → 检查 `sol.ns` 的近似逻辑（spp_processor.py:27-28）

**RTK 不一致时**：
1. 看历元数 → 不对则修 `RtkProcessor` 的循环结构
2. 历元数对但某些历元 Q/位置不同 → 检查 `nav.tt` 计算、`use_sing_pos` 触发条件、`nav.x[6:9]=1e-6` 是否在每次 pntpos 后重置
3. 仅特定历元不同 → 用 `_debug_rtk_fail.py` 同款 trace 方法定位

---

## 6. 实施顺序

1. **写 `tools/verify_rtklib_py.py`**：SPP + RTK 两路径，复用 `RtklibEnv` + `savesol`
2. **跑 reference**：`verify_spp.pos` + `verify_rtk.pos`，人工检查首历元 week/sow 合理
3. **写 `tools/compare_pos.py`**：解析 + diff + 报告
4. **写 `tests/test_compare_pos.py`**：mock 数据覆盖各 diff 场景
5. **跑全量比对**：得到 `diff_spp.txt` + `diff_rtk.txt`
6. **重跑 gipylib SPP**（针对差异 1）：覆盖 `test_spp.pos`，预期 week 修复
7. **调试 RTK 18 个缺失历元**（针对差异 2）：定位 + 修复 `RtkProcessor`
8. **回归**：重跑全量比对，两边应一致；跑 `pytest` 确保 42+ 现有测试不破坏

---

## 7. 设计决策记录

### 7.1 为何选方法 A（独立脚本法）而非用 run_ppk.py

- 用户明确要"用 rtklib-py 跑"——方法 A 直接调用 rtklib-py 的 `pntpos`/`rtkpos`，最贴合需求
- `run_ppk.py` 的 `config_f9p.py` 参数与项目 YAML 不一致，手工翻译易出错
- `run_ppk.py` 不简化 RINEX，rtklib-py 会因多频点+乱序报"Obs file too complex"
- `run_ppk.py` 无 SPP 入口
- 方法 A 复用 `RtklibEnv` + `rinex_simplifier` 消除配置与 RINEX 两个噪声变量，把对比聚焦在"封装层差异"

### 7.2 为何 RTK 用 `procpos` 而 SPP 用手写循环

- RTK：`procpos` → `rtkpos` → `relpos` 是 rtklib-py 的官方 PPK 路径，直接复用最忠实
- SPP：rtklib-py 无 SPP-only 入口，`pntpos` 是单历元函数，必须手写循环。循环结构参考 `rtkpos()` 但去掉 base/relpos 部分

### 7.3 为何用 rtklib-py 的 `savesol` 而非 gipylib 的 `SolutionWriter`

- 验证目标是"算法层一致"，格式差异是噪声
- `savesol` 是 rtklib-py 自带，零格式差异风险
- gipylib 的 `SolutionWriter` 复用 `ecef2llh`/`ecef2enu_matrix`（自实现），可能与 rtklib-py 的 `ecef2pos`/`covenu` 有微小数值差异——这些差异属于"封装层"，应在算法层一致后再单独验证

### 7.4 为何容差定为 0.5m/1.0m 而非精确一致

- 用户明确要求（"平面位置误差在0.5m以内，高程在1m以内就算一致"）
- 即便同代码路径，浮点累加顺序差异也可能导致 1e-9 量级偏差
- 0.5m/1.0m 远大于浮点噪声，远小于真实算法差异——既能容忍噪声，又能捕获 bug

### 7.5 为何把 Q 与 ns 列为整数精确

- Q（FIX/FLOAT/DGPS/SINGLE）是解算质量的定性判断，任何差异都意味着算法分支不同
- ns（卫星数）反映剔除逻辑，差异说明卫星选择或 outlier 检测不同
- 这两个是比位置更敏感的 bug 指标

---

## 8. 差异根因记录（已全部修复）

### 8.1 SPP week=2569（已修复）

- **现象**：`test_spp.pos` 首历元 week=2569，`verify_spp.pos` 为 week=2046
- **根因**：`test_spp.pos` 是 `solution_converter.py` 修复前生成的 stale 文件
- **修复**：重跑 `python src/main.py data/cfg_test_spp.yaml` 覆盖
- **结果**：SPP 2489/2489 历元完全一致（位置精确匹配）

### 8.2 RTK 协方差 ENU 索引错位（已修复）

- **现象**：RTK 位置精确匹配（max planar 1e-9 deg, max height 1e-4 m），但 sdn/sde 交换、sdeu/sdun 交换
- **根因**：`SolutionWriter` 的 ENU 协方差索引与 rtklib-py `savesol` 不一致
  - gipylib 原代码: `sdn=cov[0,0]`(E), `sde=cov[1,1]`(N) — 错误
  - rtklib-py savesol: `sdn=std[1,1]`(N), `sde=std[0,0]`(E) — 正确
- **修复**：`src/log/solution_writer.py` 对齐 rtklib-py 的索引映射
- **结果**：sd 6 列全部匹配

### 8.3 RTK 协方差丢失非对角项（已修复）

- **现象**：gipylib 的 `GnssSolution.sd` 仅 3 元素对角线，`SolutionWriter` 用 `diag(sd**2)` 重构丢失非对角项
- **根因**：`solution_converter` 只取 `sol.qr` 对角线，`SolutionWriter` 只用对角协方差旋转
- **修复**：
  - `data_types.py`: `GnssSolution` 添加可选 `cov: [3,3]` 字段
  - `solution_converter.py`: 设置 `cov = sol.qr[0:3, 0:3]`
  - `solution_writer.py`: 优先用 `sol.cov`，回退 `diag(sd**2)`
- **结果**：ENU 协方差完整旋转，sd 值与 rtklib-py 一致

### 8.4 RTK ns 差异（诊断，不卡一致）

- **现象**：test_rtk.pos 的 ns=4/12 等，verify_rtk.pos 的 ns=0
- **根因**：rtklib-py 的 `relpos` 不写 `sol.ns`（只写 `nav.ns`），`savesol` 直接输出 ns=0
- **gipylib 行为**：`RtkProcessor` 有 workaround `sol.ns = nav.ns`，提供有用诊断信息
- **处理**：ns 降级为诊断信息（与 sd/age/ratio 同级），不卡一致性判定
- **理由**：ns 是卫星数诊断，不影响定位结果；gipylib 的 workaround 保留诊断价值

### 8.5 最终比对结果

| 模式 | 历元数 | 位置匹配 | Q 匹配 | sd 匹配 | 结论 |
|------|--------|---------|--------|---------|------|
| SPP  | 2489/2489 | 精确 (0m) | ✓ | ✓ (全 0) | **CONSISTENT** |
| RTK  | 2489/2489 | max planar 1e-9° (~0mm), max h 1e-4m | ✓ | ✓ | **CONSISTENT** |

---

## 9. 未来工作

1. 若发现 `SolutionWriter` 与 rtklib-py `savesol` 的数值差异，单独验证 `ecef2llh` vs `ecef2pos`、`covenu` 实现一致性
2. 若 RTK 18 个缺失历元的根因是 `RtkProcessor` 架构问题，考虑用 `procpos` 直接替换 `RtkProcessor` 的循环
3. 真正流式 RINEX 解码（rtklib-py `rinex.py` 重写）后，重跑本验证确保流式与批量一致
