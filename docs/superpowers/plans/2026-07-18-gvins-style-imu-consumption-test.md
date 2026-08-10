# GVINS 风格 IMU 数据消费 测试文档

> 配套设计文档：`docs/superpowers/specs/2026-07-18-gvins-style-imu-consumption-design.md`
>
> 验证策略：运行 `data/config.yaml` 端到端测试，检查输出文件完整性 + 精度对比 + 回归对比。

---

## 1. 测试目标

| 编号 | 目标 | 验收标准 |
|------|------|----------|
| T1 | 端到端运行无异常 | 退出码 0，无 traceback |
| T2 | 三输出文件生成完整 | RTK.pos / RTKLC.pos / aligned_internal_rtk.csv 行数 > 0 |
| T3 | RTKLC.pos 精度达标 | 平面 ≤0.5m，高程 ≤1m（与 RTK.pos 整数秒对比） |
| T4 | 运行时长合理 | < 600s（参考最近邻策略 ~348s） |
| T5 | 回归对比 | 与上一次最近邻策略输出差异 < 0.5m（同一整数秒） |

## 2. 测试环境

- 工作目录：`/home/mxl/workplace/gipylib`
- 配置文件：`data/config.yaml`（internal+on 模式）
- Python：`python3`（系统默认为 Python 2.7，必须用 python3）
- 输入数据：
  - `data/cpt0870.19o`（流动站观测）
  - `data/cpt0870_base.19o`（基站观测）
  - `data/brdm0870.19p`（星历）
  - `data/cpt_euroc.csv`（IMU 数据，EuRoC 格式）

## 3. 测试步骤

### 3.1 预检查

```bash
# 确认输入文件存在
ls -la data/cpt0870.19o data/cpt0870_base.19o data/brdm0870.19p data/cpt_euroc.csv

# 清空输出目录（避免与上次结果混淆）
rm -f data/output/RTK.pos data/output/RTKLC.pos data/output/aligned_internal_rtk.csv
```

### 3.2 执行

```bash
cd /home/mxl/workplace/gipylib
python3 src/main.py data/config.yaml 2>&1 | tee /tmp/gvins_run.log
```

**预期终端输出包含**：
- `LcStream 初始化成功` 或 `LcStream 初始化成功 (静态回退)`
- `LcStream 输出: N 历元`（N > 0）
- `运行时长: Xs (Ym Zs)`
- 无 `Traceback`、无 `ERROR`

### 3.3 输出文件检查

```bash
# 文件存在 + 行数
wc -l data/output/RTK.pos data/output/RTKLC.pos data/output/aligned_internal_rtk.csv

# 首尾行检查（时间戳范围）
head -3 data/output/RTKLC.pos
tail -3 data/output/RTKLC.pos
```

### 3.4 精度对比（RTKLC.pos vs RTK.pos）

两个文件均为 `.pos` 格式，列：`week sec x y z ...`（ECEF）。

对比脚本逻辑（Python）：
```python
# 读取两文件整数秒解
# 对齐 week+sec
# 计算 |x_diff|, |y_diff|, |z_diff|
# 平面误差 = sqrt(dx² + dy²)
# 高程误差 = |dz|
# 统计: max, mean, >0.5m 平面数, >1m 高程数
```

**验收标准**（项目硬约束）：
- 平面位置误差 ≤ 0.5m（lat/lon 差 < 5e-6 度，等价 ECEF dx²+dy² ≤ 0.25）
- 高程误差 ≤ 1m（|dz| ≤ 1.0）
- 允许少数历元超限（< 5%），但 max 不超过 2x 阈值

### 3.5 回归对比（与最近邻策略输出）

如果有上一次最近邻策略的 `RTKLC.pos` 备份（如 `RTKLC.pos.nearest`）：
```bash
# 对比同一整数秒的 ECEF 解
diff <(awk '{print $1, $2, $3, $4, $5}' data/output/RTKLC.pos) \
     <(awk '{print $1, $2, $3, $4, $5}' data/output/RTKLC.pos.nearest) | head -20
```

**验收标准**：
- 同一整数秒 ECEF 解差异 < 0.5m（GVINS 插值 vs 最近邻，预期 < 0.1m）
- 若无备份，跳过此项（T5 标记为 N/A）

## 4. 自动化验证脚本

由于本项目无单元测试框架，采用内联 Python 脚本验证：

```python
#!/usr/bin/env python3
"""GVINS 风格 IMU 消费验证脚本。"""
import sys
from pathlib import Path

def parse_pos(path):
    """解析 .pos 文件，返回 { (week, int_sec): (x, y, z) }。"""
    sols = {}
    with open(path) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            week = int(parts[0])
            sec = float(parts[1])
            x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
            int_sec = int(round(sec))
            if abs(sec - int_sec) < 0.01:  # 整数秒
                sols[(week, int_sec)] = (x, y, z)
    return sols

def main():
    rtk = parse_pos('data/output/RTK.pos')
    rtklc = parse_pos('data/output/RTKLC.pos')

    common = sorted(set(rtk.keys()) & set(rtklc.keys()))
    if not common:
        print("FAIL: 无共同整数秒历元")
        sys.exit(1)

    planar_errors = []
    vertical_errors = []
    for k in common:
        x1, y1, z1 = rtk[k]
        x2, y2, z2 = rtklc[k]
        planar = ((x1-x2)**2 + (y1-y2)**2) ** 0.5
        vertical = abs(z1 - z2)
        planar_errors.append(planar)
        vertical_errors.append(vertical)

    n = len(common)
    planar_max = max(planar_errors)
    planar_mean = sum(planar_errors) / n
    vert_max = max(vertical_errors)
    vert_mean = sum(vertical_errors) / n
    planar_fail = sum(1 for e in planar_errors if e > 0.5)
    vert_fail = sum(1 for e in vertical_errors if e > 1.0)

    print(f"共同历元数: {n}")
    print(f"平面误差 [m]: max={planar_max:.3f}, mean={planar_mean:.3f}, "
          f">0.5m={planar_fail}/{n} ({100*planar_fail/n:.1f}%)")
    print(f"高程误差 [m]: max={vert_max:.3f}, mean={vert_mean:.3f}, "
          f">1.0m={vert_fail}/{n} ({100*vert_fail/n:.1f}%)")

    # 验收
    ok = True
    if planar_max > 1.0:  # 2x 阈值
        print(f"FAIL: 平面 max {planar_max:.3f} > 1.0m")
        ok = False
    if vert_max > 2.0:  # 2x 阈值
        print(f"FAIL: 高程 max {vert_max:.3f} > 2.0m")
        ok = False
    if planar_fail / n > 0.05:
        print(f"FAIL: 平面超限率 {100*planar_fail/n:.1f}% > 5%")
        ok = False
    if vert_fail / n > 0.05:
        print(f"FAIL: 高程超限率 {100*vert_fail/n:.1f}% > 5%")
        ok = False

    if ok:
        print("PASS: 精度达标")
        sys.exit(0)
    else:
        print("FAIL: 精度未达标")
        sys.exit(1)

if __name__ == '__main__':
    main()
```

## 5. 测试用例矩阵

| 用例 | 输入 | 预期 | 验证方法 |
|------|------|------|----------|
| TC1 正常运行 | data/config.yaml | 三文件生成，无异常 | 3.2 + 3.3 |
| TC2 精度达标 | RTKLC.pos + RTK.pos | 平面≤0.5m, 高程≤1m | 3.4 / 自动化脚本 |
| TC3 IMU 队列满 | imu_queue=200 | 传感器线程阻塞但不死锁 | 运行完成即验证 |
| TC4 GNSS 队列满 | gnss_queue=3 | 传感器线程阻塞但不死锁 | 运行完成即验证 |
| TC5 IMU 跨 GNSS 时刻 | 第一条 t>=gnss.t 的 IMU | 触发插值+融合 | LcIntegration 内部验证（日志） |
| TC6 多 GNSS 同区间 | 两个 GNSS 落在同一 IMU 区间 | while 循环逐个处理 | 日志 + 输出连续性 |
| TC7 时间戳对齐 | imu.t == gnss.t | 直接触发（不插值） | 边界条件，运行时自然覆盖 |
| TC8 回归对比 | 本次 vs 上次 RTKLC.pos | 差异 < 0.5m | 3.5（若有备份） |

## 6. 失败处理

| 症状 | 排查方向 |
|------|----------|
| LcStream 未初始化 | 检查 IMU/GNSS 数据时间范围是否重叠 |
| RTKLC.pos 行数为 0 | 初始化失败或输出窗口未刷出，查日志 |
| 平面误差 > 0.5m | 检查插值方向（w1/w2 是否反了）、time_update dt 计算 |
| 高程误差 > 1m | 检查 NHC decimation、ZUPT 静态检测 |
| 运行时长 > 600s | 队列阻塞或死锁，检查 Logger 喂入逻辑 |
| Traceback | 按堆栈定位，常见：插值返回 None 未处理 |

## 7. 测试通过判据

全部满足则标记为 PASS：
- T1: 退出码 0，无 traceback
- T2: 三文件行数 > 0
- T3: 平面 max ≤ 1.0m 且超限率 ≤ 5%，高程 max ≤ 2.0m 且超限率 ≤ 5%
- T4: 运行时长 < 600s
- T5: 若有备份，差异 < 0.5m；否则 N/A
