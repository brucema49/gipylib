# 内部 GNSS 解算（纯 GNSS 模式）设计

> 日期: 2026-07-01
> 主题: 在 `gnss_source=internal` + `ins.enabled=off` 模式下，借助 rtklib-py 实现 SPP/RTK 纯 GNSS 解算分支
> 范围: 仅纯 GNSS 解算路径；INS 估计器、紧组合、PPP 不在本任务范围

---

## 1. 背景与目标

### 1.1 当前状态

GInsStream 现已实现 external 模式（IMU 100Hz + 外部 .pos 1Hz → 块状 CSV 对齐输出，42 测试通过）。`data/config.yaml` 的 `gnss:` 段已包含 rtklib-py 风格参数（rover_path / base_path / eph_path / nf / pmode / armode 等），但 `config_loader.py` 强制 `gnss_source == "external"`，内部解算路径未实现。

rtklib-py 库位于 `library/rtklib-py/src/`，提供 `pntpos(obs, nav)` (SPP)、`relpos(nav, obsr, obsb, sol)` (RTK)、`rnx_decode` / `decode_obsfile` / `decode_nav` / `first_obs` / `next_obs`。**全量读取** RINEX 文件到 `obslist`，非流式。

可用数据：`data/cpt0870.19o`（流动站）、`data/cpt0870_base.19o`（基站）、`data/brdm0870.19p`（星历）。

### 1.2 目标

1. 在 `ins:` 段新增 `enabled: on/off` 主开关与 `reboot` 重启阈值
2. 在 `gnss:` 段新增 `positioning_mode: "spp"/"rtk"` 字段
3. 实现 `gnss_source=internal` + `ins.enabled=off` 的纯 GNSS 解算路径
4. 仅测 SPP 和 RTK，不测 PPP
5. 借助 rtklib-py 但保持架构清晰，不污染 `library/`
6. 修改线程控制支持该模式，先测试代码再合并到 src

### 1.3 非目标

- INS 估计器实现（`internal + on` 抛 `NotImplementedError`）
- 紧组合
- PPP
- `ins.reboot` 的实施与验证（仅设计加配置项）
- rtklib-py RINEX 解码器的真正流式重写（阶段一妥协，见 4.3）

---

## 2. 架构总览

### 2.1 三种运行路径

由 `(gnss.gnss_source, ins.enabled)` 决定：

| 路径 | gnss_source | ins.enabled | sensors | logger | writer | 输出 |
|------|-------------|-------------|---------|--------|--------|------|
| A（现有） | external | on | ImuSensor + GnssSolSensor | Logger（现有） | AlignedWriter | aligned.csv（块状） |
| B（本任务） | internal | off | InternalGnssSensor（新） | SolutionLogger（新） | SolutionWriter（新） | solution.pos |
| C（未来） | internal | on | ImuSensor + InternalGnssSensor | InsLogger（未来） | — | INS 解算结果 |

### 2.2 main.py 装配

新增 `_assemble_pipeline(config, control, imu_queue, gnss_queue) -> (sensors, logger)` 装配函数，根据 `(gnss_source, ins.enabled, positioning_mode)` 选择 sensors 列表与 logger 实例。`main()` 本身不膨胀。

### 2.3 线程模型（路径 B）

```
InternalGnssSensor 线程 ──put──→ gnss_queue ──→ SolutionLogger 线程 ──→ solution.pos
   （逐历元调用 rtklib-py）
```

- 不创建 imu_queue，不启动 ImuSensor
- SolutionLogger 仅消费 gnss_queue，无 IMU 等待逻辑
- EOF sentinel 仍用 `None`，与现有约定一致
- ThreadControl 不变（running 标志 + shutdown）

### 2.4 关键约束

- 现有 external 路径（路径 A）零改动，保持 42 测试通过
- 路径 B 的 rtklib-py `nav` 状态由 `InternalGnssSensor` 持有（实例属性），跨历元维护模糊度/P 矩阵
- 不修改 `library/rtklib-py/` 任何文件
- `__ppk_config` 模块用 `types.ModuleType` 动态注入 `sys.modules`，避免写文件污染项目根目录

---

## 3. 配置 Schema

### 3.1 新增配置项

**`gnss:` 段**（紧邻 `gnss_source` 之后）：

```yaml
gnss:
  gnss_source: "external"      # 现有
  positioning_mode: "spp"      # 新增: "spp" / "rtk"，仅 internal 模式生效
```

**`ins:` 段顶部新增**（在"处理时间"之前）：

```yaml
ins:
  # 主开关（必填）: on = 组合导航路径 / off = 纯 GNSS 解算
  # - external 模式必须为 on（外部 GNSS 已有，必走组合导航）
  # - internal + off = 纯 GNSS 解算（本任务）
  # - internal + on  = 内部 GNSS + 组合导航（未来）
  enabled: "on"

  # GNSS 中断重启阈值 [s] (默认 50s)
  # 仅 ins.enabled=on 时生效: INS 初始化完成后，若 GNSS 中断超过此阈值，
  # 重新进行组合导航初始化的数据对齐操作（IMU 连续但 GNSS 不一定连续）
  # 注: 本任务仅设计，不实现验证
  reboot: 50

  # 现有字段保持不变 ...
```

### 3.2 配置组合合法性表

| gnss_source | ins.enabled | 合法性 | 路径 | 说明 |
|-------------|-------------|--------|------|------|
| external | on | ✓ | A 现有 | 块状对齐输出（未来加 INS 估计） |
| external | off | ✗ ValueError | — | 外部 GNSS 已有，无意义 |
| external | (absent) | ✗ ValueError | — | 必填 |
| internal | off | ✓ | B 本任务 | 纯 GNSS .pos 输出 |
| internal | on | ✗ NotImplementedError | C 未来 | 内部 GNSS + INS 未实现 |
| internal | (absent) | ✗ ValueError | — | 必填 |
| internal | positioning_mode 缺失 | ✗ ValueError | — | internal 必填 |
| internal | positioning_mode=ppp | ✗ ValueError | — | 不支持 PPP |

### 3.3 `config_loader.py` 校验规则

```
1. ins.enabled 必填，必须 ∈ {"on","off"}
2. gnss_source=external:
   - ins.enabled 必须为 "on"，否则 ValueError
   - 现有 external_sol_format 校验保持
   - data_rate 必须 100 (现有 IMU 流要求)
3. gnss_source=internal:
   - positioning_mode 必填，必须 ∈ {"spp","rtk"}
   - rover_path / eph_path 非空
   - positioning_mode=rtk 时 base_path 非空
   - ins.enabled=off → 纯 GNSS 路径，不校验 data_rate（IMU 不读）
   - ins.enabled=on  → NotImplementedError ("INS estimator not implemented")
```

### 3.4 `data/config.yaml` 改动

- `gnss:` 段 `gnss_source` 后加 `positioning_mode: "spp"`
- `ins:` 段顶部加 `enabled: "on"` 和 `reboot: 50`
- 现有 `gnss_source: "external"` + `ins.enabled: "on"` 组合保持当前行为不变

---

## 4. 组件清单与职责

### 4.1 新增文件

**`src/core/gnss/__init__.py`** — 包标识

**`src/core/gnss/gnss_processor.py`** — ABC + 异常

```python
class GnssProcessor(ABC):
    """GNSS 处理器抽象基类（内部模式）。"""
    @abstractmethod
    def process_epoch(self, obsr, obsb=None) -> Optional[GnssSolution]: ...
    @abstractmethod
    def reset(self) -> None: ...

class GnssConfigError(ValueError): ...
class GnssSolutionError(RuntimeError): ...
```

**`src/core/gnss/rtklib_config_adapter.py`** — YAML → rtklib-py cfg 模块对象

- 把 `config["gnss"]` 翻译成 rtklib-py 期望的 `__ppk_config` 模块对象
- 用 `types.ModuleType` 动态构建并注入 `sys.modules['__ppk_config']`
- 处理 `freq_ix0/1` 的 key 从字符串 "GPS" → rtklib-py 的 `uGNSS.GPS` 枚举
- 不修改 `library/rtklib-py/` 任何文件，不写 `__ppk_config.py` 文件

**`src/core/gnss/spp_processor.py`** — SPP 处理器

```python
class SppProcessor(GnssProcessor):
    """SPP 单点定位处理器，薄封装 rtklib-py 的 pntpos。"""
    def __init__(self, nav, rover_decoder):
        # nav: rtklib-py Nav 对象（持有跨历元状态）
        # rover_decoder: rtklib-py rnx_decode 对象
    def process_epoch(self, obsr, obsb=None) -> Optional[GnssSolution]:
        # 调用 pntpos(obsr, nav)，把 Sol 转成 GnssSolution
```

**`src/core/gnss/rtk_processor.py`** — RTK 处理器

```python
class RtkProcessor(GnssProcessor):
    """RTK 处理器，薄封装 rtklib-py 的 relpos。"""
    def __init__(self, nav, rover_decoder, base_decoder):
        # base_decoder: 基站 RINEX 解码器
    def process_epoch(self, obsr, obsb=None) -> Optional[GnssSolution]:
        # 调用 relpos(nav, obsr, obsb, sol)，把 Sol 转成 GnssSolution
```

**`src/core/gnss/solution_converter.py`** — rtklib-py `Sol` → 本项目 `GnssSolution`

- 位置 ECEF 直接复制 `Sol.rr[0:3]`
- quality 映射: rtklib `Sol.stat` (`SOLQ_SINGLE=1`, `SOLQ_DGPS=2`, `SOLQ_FIX=5`, ...) → 本项目 quality
- sd 从 `Sol.qr[0:3, 0:3]` 对角线平方根
- 时间戳: rtklib `Sol.t` (gpst) → 周内秒 + 周号

**`src/stream/internal_gnss_sensor.py`** — 内部 GNSS 传感器线程

```python
class InternalGnssSensor(StreamerBase):
    """内部 GNSS 解算传感器线程。"""
    def __init__(self, config, output_queue, control):
        # 装配 rtklib-py: rnx_decode(rover), rnx_decode(base), rtkinit(nav)
        # 根据 positioning_mode 创建 SppProcessor 或 RtkProcessor
    def run(self):
        # 1. decode_obsfile(rover, maxepoch=None)
        # 2. first_obs(nav, rov, base, dir=1)
        # 3. 循环 next_obs(nav, rov, base, dir=1):
        #      processor.process_epoch(obsr, obsb) → GnssSolution
        #      output_queue.put(SensorData(tag="gnss_solution", gnss_solution=sol))
        # 4. output_queue.put(None)  # EOF sentinel
```

**`src/log/solution_writer.py`** — .pos 输出器

```python
class SolutionWriter(WriterBase):
    """rtklib 风格 .pos 输出器。"""
    def open(self): # 写表头
    def write(self, sol: GnssSolution): # 一行: GPST lat lon h Q ns sdx sdy sdz ...
    def close(self): ...
```

**`src/log/solution_logger.py`** — 纯 GNSS 日志线程

```python
class SolutionLogger(Thread):
    """纯 GNSS 模式日志线程，仅消费 gnss_queue。"""
    def __init__(self, gnss_queue, writer, control): ...
    def run(self):
        # 循环 gnss_queue.get(timeout=0.1):
        #   None → break
        #   SensorData → writer.write(sol)
```

### 4.2 修改的现有文件

- **`src/main.py`**：新增 `_assemble_pipeline(config, control, imu_queue, gnss_queue) -> (sensors, logger)` 装配函数
- **`src/utility/config_loader.py`**：按 3.3 改造校验规则
- **`data/config.yaml`**：按 3.1 新增字段
- **`src/stream/factory.py`**：`SensorFactory` 加 `create_internal_gnss_sensor(config, gnss_queue, control)` 静态方法
- **`tests/fixtures/config_test.yaml`**：加 `ins.enabled: "on"` 使现有 fixture 仍合法

### 4.3 不改动的现有文件

- `src/log/logger.py`（现有 Logger，路径 A 专用）
- `src/log/aligner.py` / `aligned_writer.py`（路径 A 专用）
- `src/stream/imu_sensor.py` / `gnss_sol_sensor.py`（路径 A 专用）
- `src/core/data_types.py`（GnssSolution 复用，无需改）

### 4.4 与 `library/rtklib-py/` 的边界

- **只读不改**：不修改 library/ 下任何文件
- **导入方式**：通过 `sys.path.insert` 把 `library/rtklib-py/src` 加入路径
- **`__ppk_config` 模块机制**：适配器用 `types.ModuleType` 动态构建并注入 `sys.modules['__ppk_config']`，避免写文件污染项目根目录

### 4.5 已知妥协：rtklib-py 全量加载 RINEX

rtklib-py 的 `rnx_decode.decode_obsfile` 全量加载 obslist 到内存。真正流式 RINEX 解码需重写 rtklib-py 的 rinex.py，超出本任务范围。本任务在 Sensor 线程内"逐历元消费 obslist"，对外暴露流式接口，内部仍批量加载。这是阶段一妥协，未来需重写 RINEX 解码器以实现真正的 O(1) 内存流式。

---

## 5. 数据流与处理流程

### 5.1 路径 B 完整数据流

```
文件系统                Stream 层                    Log 层
─────────              ─────────                    ──────

cpt0870.19o ──┐
              │   InternalGnssSensor 线程
brdm0870.19p ─┼──→  1. rtklib_config_adapter 注入 __ppk_config
              │     2. rnx_decode(rover) + decode_obsfile
cpt0870_base  │     3. rnx_decode(base) + decode_obsfile
   .19o ──────┘     4. rtkinit(nav)
                    5. SppProcessor 或 RtkProcessor 装配
                    6. first_obs(nav, rov, base, dir=1)
                    7. 循环:
                       obsr, obsb = next_obs(nav, rov, base, 1)
                       if obsr == []: break
                       sol = processor.process_epoch(obsr, obsb)
                       if sol: gnss_queue.put(SensorData(sol))
                    8. gnss_queue.put(None)  # EOF
                                       │
                                       ▼
                              SolutionLogger 线程
                                1. writer.open()
                                2. 循环 gnss_queue.get(timeout=0.1):
                                     None → break
                                     SensorData → writer.write(sol)
                                3. writer.close()
                                       │
                                       ▼
                              output/solution.pos
```

### 5.2 SPP 处理流程（SppProcessor.process_epoch）

```
obsr (单历元流动站观测)
  │
  ├── 1. pntpos(obsr, nav)  # rtklib-py 函数
  │     内部: satposs → estpos (最小二乘迭代)
  │     返回: rtklib Sol 对象
  │
  ├── 2. solution_converter.sol_to_gnss_solution(sol)
  │     ECEF 位置、quality 映射、sd 提取、时间戳转换
  │
  └── 3. 返回 GnssSolution 或 None（解算失败时）
```

### 5.3 RTK 处理流程（RtkProcessor.process_epoch）

```
obsr, obsb (单历元流动站+基站观测)
  │
  ├── 1. sol = Sol()  # 新建 rtklib Sol
  ├── 2. 首历元或 sol.rr[0]==0 时: 先调用 pntpos(obsr, nav) 取初值填入 sol.rr
  ├── 3. relpos(nav, obsr, obsb, sol)  # rtklib-py 函数
  │     内部: satposs → zdres(base/rover) → selsat → udstate → ddres
  │            → Kalman filter → LAMBDA 模糊度固定
  │     修改: nav.x, nav.P, nav.azel, nav.fix, sol.rr, sol.qr, sol.stat
  │
  ├── 4. solution_converter.sol_to_gnss_solution(sol)
  │     quality 映射: SOLQ_FIX=5 (RTK固定), SOLQ_FLOAT=4 (浮点),
  │                   SOLQ_DGPS=2 (RTD退化), SOLQ_SINGLE=1 (SPP)
  │
  └── 5. 返回 GnssSolution 或 None
```

### 5.4 跨历元状态管理

- `nav` 对象由 `InternalGnssSensor` 持有为实例属性，跨历元维护：
  - 位置/速度/加速度状态 `nav.x[0:9]`
  - 协方差矩阵 `nav.P`
  - 模糊度状态 `nav.x[9:]`
  - 仰角/方位角 `nav.azel`
  - 周跳/锁定计数 `nav.slip`, `nav.lock`
  - 解算历史 `nav.sol`（仅追加，不修改）
- `SppProcessor` / `RtkProcessor` 接收 `nav` 引用，每历元调用 rtklib-py 函数更新 `nav`
- 线程安全：`InternalGnssSensor` 单线程运行，无并发访问 `nav`

---

## 6. 错误处理

### 6.1 配置错误（启动期）

| 错误 | 异常 | 处理 |
|------|------|------|
| `ins.enabled` 缺失或非法值 | `ValueError` | 启动失败，提示用户 |
| `external + ins.enabled=off` | `ValueError` | 启动失败 |
| `internal + positioning_mode` 缺失/非法 | `ValueError` | 启动失败 |
| `internal + positioning_mode=rtk` 但 `base_path` 空 | `ValueError` | 启动失败 |
| `internal + ins.enabled=on` | `NotImplementedError` | 启动失败，提示未实现 |
| RINEX 文件不存在 | `FileNotFoundError` | 启动失败 |

### 6.2 解算错误（运行期）

| 错误 | 处理 |
|------|------|
| 单历元 SPP 不收敛（`sol.stat == SOLQ_NONE`） | 跳过该历元，不写入 .pos，记录 warning 日志 |
| RTK `relpos` 内部 `maxage` 超限 | relpos 内部 return，`sol.stat` 保持 NONE，跳过该历元 |
| 双差共视卫星 < 4 | relpos 内部 return，跳过该历元 |
| 模糊度 ratio 检验失败 | 自动退化为浮点解或 RTD（rtklib-py 内部处理） |
| 星历缺失某卫星 | rtklib-py `seleph` 返回 None，该卫星跳过 |

### 6.3 线程控制错误

| 错误 | 处理 |
|------|------|
| `control.is_running() == False` | Sensor 线程退出循环，推入 EOF sentinel |
| SolutionLogger 收到 EOF | break 循环，关闭 writer |
| Sensor 线程异常 | `finally` 块推入 EOF sentinel，Logger 仍能正常关闭 writer |

### 6.4 资源清理

- `InternalGnssSensor.run()` 用 `try/finally` 确保 EOF sentinel 一定推入
- `SolutionLogger.run()` 用 `try/finally` 确保 `writer.close()` 一定调用
- `main()` 在 `logger.join()` 后调用 `control.shutdown()`，再 `s.join(timeout=2)` 等所有 sensor 退出

---

## 7. 测试策略

### 7.1 单元测试

| 模块 | 测试文件 | 测试内容 |
|------|---------|---------|
| `config_loader` | `tests/test_config_loader.py` | 新增校验规则：external+on 合法、external+off 非法、internal+off 合法、internal+on 抛 NotImplementedError、positioning_mode 校验 |
| `rtklib_config_adapter` | `tests/test_rtklib_config_adapter.py` | YAML → cfg 模块对象转换正确性（nf/pmode/armode/freq_ix 等） |
| `solution_converter` | `tests/test_solution_converter.py` | Sol → GnssSolution 映射（位置/quality/sd/时间戳） |
| `solution_writer` | `tests/test_solution_writer.py` | .pos 格式输出正确性（表头 + 数据行） |
| `solution_logger` | `tests/test_solution_logger.py` | EOF 处理、SensorData 解包、writer 调用 |
| `spp_processor` | `tests/test_spp_processor.py` | mock rtklib-py pntpos，验证调用与转换 |
| `rtk_processor` | `tests/test_rtk_processor.py` | mock rtklib-py relpos，验证调用与转换 |

### 7.2 集成测试

| 测试 | 文件 | 验证 |
|------|------|------|
| SPP 端到端 | `tests/test_internal_gnss_spp_e2e.py` | 用 `data/cpt0870.19o` + `data/brdm0870.19p` 跑 SPP，验证 `output/solution.pos` 行数 > 0、首行格式正确、时间戳单调递增 |
| RTK 端到端 | `tests/test_internal_gnss_rtk_e2e.py` | 用 `data/cpt0870.19o` + `data/cpt0870_base.19o` + `data/brdm0870.19p` 跑 RTK，验证 `output/solution.pos` 行数 > 0、含固定解/浮点解/RTD 之一 |

### 7.3 回归测试

- 现有 42 个测试必须全部通过（路径 A 零改动）
- `tests/fixtures/config_test.yaml` 加 `ins.enabled: "on"` 使其仍合法
- `tests/test_config_loader.py` 中现有 `test_invalid_gnss_source`（断言 internal 非法）需改为断言 `internal + ins.enabled=off` 合法（internal 现已合法）

### 7.4 手动验证

- `python src/main.py data/config.yaml`（external+on 模式）应输出 `output/aligned.csv`（行为不变）
- 临时改 `data/config.yaml` 为 `gnss_source: "internal"` + `ins.enabled: "off"` + `positioning_mode: "spp"`，运行应输出 `output/solution.pos`
- 同上改 `positioning_mode: "rtk"`，运行应输出 `output/solution.pos`（RTK 解算）

### 7.5 测试数据

- SPP: `data/cpt0870.19o`（流动站）+ `data/brdm0870.19p`（星历）
- RTK: 上述 + `data/cpt0870_base.19o`（基站）
- 已有 `data/spp.pos` 可作为 SPP 结果对比参考（可选）

---

## 8. 实施顺序

1. **配置层**：改 `data/config.yaml` + `tests/fixtures/config_test.yaml` + `config_loader.py` 校验 → 跑 `test_config_loader.py`
2. **rtklib 适配层**：`rtklib_config_adapter.py` + `solution_converter.py` + 单测
3. **处理器层**：`gnss_processor.py` (ABC) + `spp_processor.py` + `rtk_processor.py` + 单测（mock rtklib-py）
4. **传感器层**：`internal_gnss_sensor.py` + 集成到 `SensorFactory`
5. **输出层**：`solution_writer.py` + `solution_logger.py` + 单测
6. **装配层**：`main.py` 的 `_assemble_pipeline` + `SensorFactory` 改造
7. **端到端测试**：SPP E2E + RTK E2E
8. **回归测试**：跑全量 `pytest`，确保 42 + 新增测试全过
9. **手动验证**：运行 `python src/main.py` 验证三种配置组合

---

## 9. 设计决策记录

### 9.1 为何选择流式逐历元封装而非批量预解算

- 与项目"纯 threading + queue 流式"架构一致
- 未来可替换 rtklib-py 的 RINEX 解码器为真正流式实现，Sensor 接口不变
- 批量预解算会引入临时文件，与 external 模式语义混淆

### 9.2 为何不引入 LoggerBase ABC

- 当前仅 2 种模式（aligned / pure_gnss），ABC 略显过度设计
- 等 INS 估计器实际接入时（路径 C）再演化到 ABC 体系
- 现有 Logger 与新 SolutionLogger 职责差异大，强行抽象基类会污染现有稳定代码

### 9.3 为何 `__ppk_config` 用动态模块注入

- rtklib-py 通过 `import __ppk_config as cfg` 读取配置
- 写文件方式（`shutil.copyfile`）会污染项目根目录，且多实例并发不安全
- `types.ModuleType` + `sys.modules` 注入是 Python 标准做法，无副作用

### 9.4 为何 `ins.enabled` 用字符串 "on"/"off" 而非 0/1

- 用户明确要求 "off/on" 语义
- 与现有 `gnss_enable: 1` 等子开关（数字）区分，强调这是主开关
- 字符串更易读，避免 0/1 与布尔混淆

### 9.5 为何 `internal + ins.enabled=on` 抛 NotImplementedError 而非 silently 走纯 GNSS

- 用户配置明确表达 "要做组合导航"，silently 降级会隐藏配置错误
- 显式异常引导用户改为 `off` 或等待 INS 估计器实现

---

## 10. 未来工作

1. **路径 C（INS 估计器）**：`internal + ins.enabled=on` 实现，接入 `src/core/imu/` + `src/core/estimator/`
2. **真正流式 RINEX 解码**：重写 rtklib-py `rinex.py` 的 `decode_obsfile` 为逐历元生成器，避免全量加载
3. **`ins.reboot` 实施**：GNSS 中断检测 + INS 重新对齐初始化
4. **LoggerBase ABC 体系**：当路径 C 实现时，提取 `LoggerBase` → `AlignedLogger` / `SolutionLogger` / `InsLogger`
5. **紧组合**：`TcEstimator` + `TcIntegration`（见 GInsStream.md 第 11 节）
6. **GnssProcessor 完整 ABC 体系**：按 skills/gnss.md 长期规划，`BaseModel` → `CombDD` 双差组合模型独立模块化
