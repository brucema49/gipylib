# Skills 文档状态刷新 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将全部 `skills/` 文档与 issue 记录反映的最新项目状态对齐。

**Architecture:** 以新增的 `skills/项目当前状态.md` 作为权威快照；现有文档保留历史设计，仅在顶部增加索引和必要的过期表述修正。

**Tech Stack:** Markdown、PowerShell `rg`/`Get-Content`、Git diff 检查。

---

### Task 1: 建立权威状态页

**Files:**
- Create: `skills/项目当前状态.md`

- [ ] 汇总 13 个 issue 的最终结论、当前基线、边界和遗留项。
- [ ] 标注实验值与通用默认值的区别。

### Task 2: 更新全部现有 skill 文件索引

**Files:**
- Modify: `skills/初始化.md`
- Modify: `skills/初始化调整.md`
- Modify: `skills/机械编排.md`
- Modify: `skills/紧组合.md`
- Modify: `skills/配置文件调整.md`
- Modify: `skills/conf.md`
- Modify: `skills/config.md`
- Modify: `skills/estimator.md`
- Modify: `skills/GInsStream.md`
- Modify: `skills/gnss.md`
- Modify: `skills/imu.md`
- Modify: `skills/logANDoutput.md`
- Modify: `skills/motion.md`
- Modify: `skills/NHC_ZUPT.md`
- Modify: `skills/robust.md`
- Modify: `skills/StreamDesign.md`

- [ ] 在每个文件顶部加入当前状态索引和权威状态页链接。
- [ ] 对架构、插值、TC AR、BDS 和输出等明显过期内容做最小必要修正。

### Task 3: 文档一致性校验

- [ ] 用 `rg` 检查所有文件均引用当前状态页。
- [ ] 检查 Git diff 和工作区，确认没有覆盖用户已有修改。
- [ ] 检查 Markdown 文件可读性和重复插入。
