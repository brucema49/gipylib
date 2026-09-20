# README 结果图表与 GREAT 配置路径设计

## 目标

完善项目展示文档，使 README 能直接展示现有结果图，并提供可复现的 GREAT-MSF 对照数据来源说明；同时将 `config/rtktc-rate.yaml` 中的绝对路径改为以 `gipylib` 仓库根目录为基准的相对路径。本次不运行配置、不生成新结果，也不修改路径解析代码。

## 修改范围

### README 图表展示

在 README 中新增“结果对比”小节，使用紧凑三图画廊排列以下图片：

- `plot/tra-diff-rtkdc.png`：GInsStream 与 ignav2s 的平面轨迹对比；
- `plot/rtk-tc-error.png`：GInsStream 与 GREAT RTK-TC 的 ENU 定位误差对比；
- `plot/tra-enu.png`：GInsStream 与 GREAT 及真值的 ENU 轨迹对比。

图片采用仓库相对路径，并为每张图添加清晰的中文说明。

### GREAT 数据说明

README 说明结果来自 `GREAT-MSF/sample_data/MSF_20201027` 的 `campus01` 数据集，并列出本配置使用的流动站观测、两段基站观测、BRDM 星历和 100 Hz `campus-01-MEMS.txt` IMU 数据。

### 配置路径

仅修改 `config/rtktc-rate.yaml` 中的路径字符串：

- GREAT-MSF 外部数据由绝对路径改为 `../GREAT-MSF/sample_data/...`；
- 仓库内输出和诊断路径保留为相对仓库根目录的 `data-great/output/rtktc-rate/...`；
- 不修改参数值、数据集选择、算法开关或输出文件名。

README 使用以下命令示例表达配置的相对路径基准，但不执行该命令：

```bash
python3 src/main.py config/rtktc-rate.yaml
```

## 验收标准

1. README 中三张图片均使用 `plot/` 相对路径并有对应说明。
2. README 明确写出 GREAT-MSF `MSF_20201027/campus01` 数据及关键输入文件。
3. `config/rtktc-rate.yaml` 不再包含 `/home/mxl/workplace/GREAT-MSF` 绝对路径。
4. 配置中的外部输入路径均以 `../GREAT-MSF/` 开头，仓库内输出路径仍以 `data-great/` 开头。
5. 本次不执行配置，不生成或修改结果文件。
