#!/usr/bin/env python3
"""生成 skills/模糊度固定.md 所需的原理插图。

用法:
    /home/mxl/anaconda3/bin/python skills/图片/generate_ambiguity_figures.py

图片输出到本脚本所在目录 (skills/图片/)。其中:
  - 原理类图 (01-07) 由几何/代数构造生成;
  - 实测类图 (08) 由 data/output/RTKTCAR.rslt (AR 固定) 与 RTKINS.rslt (纯 GPS 单频
    浮点基线) 对照 data/truth.csv 生成, 数据缺失时自动跳过。
"""

from __future__ import annotations

import csv
import math
import os
from bisect import bisect_left

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, FancyArrowPatch, FancyBboxPatch, Rectangle

# 中文字体 (Noto Sans CJK 覆盖简体汉字)
plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 160
plt.rcParams["savefig.bbox"] = "tight"

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(OUT_DIR, "..", ".."))

C_FLOAT = "#1f77b4"
C_FIX = "#2ca02c"
C_BAD = "#d62728"
C_GREY = "#7f7f7f"


def save(fig, name: str) -> None:
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, facecolor="white")
    plt.close(fig)
    print("saved", path)


# ---------------------------------------------------------------------------
# LAMBDA 核心代数 (与 rnx2rtkp/src/lambda.c 一一对应, 便于文档对照引用)
# ---------------------------------------------------------------------------
def ld_factor(Q: np.ndarray):
    """LD 分解: Q = L' * diag(D) * L (L 单位下三角, 对应 lambda.c: LD())。"""
    n = Q.shape[0]
    A = Q.astype(float).copy()
    L = np.zeros((n, n))
    D = np.zeros(n)
    for i in range(n - 1, -1, -1):
        D[i] = A[i, i]
        a = math.sqrt(D[i])
        for j in range(i + 1):
            L[i, j] = A[i, j] / a
        for j in range(i):
            for k in range(j + 1):
                A[j, k] -= L[i, k] * L[i, j]
        for j in range(i + 1):
            L[i, j] /= L[i, i]
    return L, D


def lambda_reduction(Q: np.ndarray):
    """LAMBDA 去相关, 返回整数变换矩阵 Z (对应 lambda.c: reduction())。"""
    n = Q.shape[0]
    L, D = ld_factor(Q)
    L = L.copy()
    Z = np.eye(n)
    j, k = n - 2, n - 2
    while j >= 0:
        if j <= k:
            for i in range(j + 1, n):
                mu = int(round(L[i, j]))
                if mu != 0:
                    L[i:, j] -= mu * L[i:, i]
                    Z[:, j] -= mu * Z[:, i]
        delta = D[j] + L[j + 1, j] ** 2 * D[j + 1]
        if delta + 1e-6 < D[j + 1]:
            eta = D[j] / delta
            lam = D[j + 1] * L[j + 1, j] / delta
            D[j], D[j + 1] = eta * D[j + 1], delta
            for kk in range(j):
                a0, a1 = L[j, kk], L[j + 1, kk]
                L[j, kk] = -L[j + 1, j] * a0 + a1
                L[j + 1, kk] = eta * a0 + lam * a1
            L[j + 1, j] = lam
            for kk in range(j + 2, n):
                L[kk, j], L[kk, j + 1] = L[kk, j + 1], L[kk, j]
            Z[:, j], Z[:, j + 1] = Z[:, j + 1].copy(), Z[:, j].copy()
            k = j
            j = n - 2
        else:
            j -= 1
    return Z


def ils_bruteforce(a, Q, span=4):
    """二维格点穷举 ILS, 返回按二次型距离排序的 (整数解, 距离) 列表。"""
    base = np.round(a).astype(int)
    cands = []
    for dx in range(-span, span + 1):
        for dy in range(-span, span + 1):
            z = base + np.array([dx, dy])
            d = (a - z) @ np.linalg.solve(Q, (a - z))
            cands.append((z, float(d)))
    cands.sort(key=lambda t: t[1])
    return cands


# ---------------------------------------------------------------------------
# 图 1: 双差观测量的几何与差分链路
# ---------------------------------------------------------------------------
def fig_dd_geometry():
    fig, ax = plt.subplots(figsize=(9.6, 5.8))
    ax.set_axis_off()

    sats = {"k": (1.6, 8.2), "m": (8.4, 8.2)}
    recs = {"r": (1.5, 4.6), "b": (8.5, 4.6)}

    for name, (x, y) in sats.items():
        ax.plot(x, y, marker="^", ms=16, color="#444444")
        ax.text(x, y + 0.42, f"卫星 {name}", ha="center", fontsize=12)
    for name, (x, y) in recs.items():
        ax.plot(x, y, marker="s", ms=15, color="#8c564b")
        ax.text(x, y - 0.75,
                f"测站 {name}" + ("(流动站)" if name == "r" else "(基准站)"),
                ha="center", fontsize=11.5)

    for r, rp in recs.items():
        for s, sp in sats.items():
            ax.plot([rp[0], sp[0]], [rp[1], sp[1]], ls=":", lw=1.4, color="#999999")
            # 标签放在离测站 1/3 处, 避免交叉点重叠
            mx, my = rp[0] + (sp[0] - rp[0]) * 0.34, rp[1] + (sp[1] - rp[1]) * 0.34
            ax.text(mx - 0.42, my, rf"$\Phi_{{{r}}}^{{{s}}}$", fontsize=12,
                    bbox=dict(fc="white", ec="none", alpha=0.95))

    # 星间单差 (同一测站, 两颗卫星作差)
    ax.add_patch(FancyArrowPatch((1.6, 8.2), (8.4, 8.2), arrowstyle="<->",
                                 mutation_scale=14, lw=2.0, color=C_FLOAT))
    ax.text(5.0, 8.45, r"星间单差 $\Delta \Phi_r = \Phi_r^k-\Phi_r^m$",
            ha="center", fontsize=11, color=C_FLOAT)
    # 站间单差 (同一卫星, 两个测站作差)
    ax.add_patch(FancyArrowPatch((1.6, 8.2), (1.5, 4.6), arrowstyle="<->",
                                 mutation_scale=14, lw=1.8, color=C_FIX))
    ax.text(0.75, 6.4, r"站间单差 $\nabla\Phi^k$", ha="left", fontsize=11,
            color=C_FIX, rotation=90)

    ax.add_patch(Rectangle((-0.35, 0.15), 10.6, 9.3, fill=False, ec="#dddddd"))
    ax.text(5.0, 2.25,
            "双差: "
            r"$\nabla\Delta\Phi_{rb}^{km}=\left(\Phi_r^k-\Phi_r^m\right)-"
            r"\left(\Phi_b^k-\Phi_b^m\right)$"
            "\n"
            r"$\Rightarrow$ 消掉卫星钟、接收机钟、电离层/对流层一阶项,"
            "\n"
            r"剩余量含 $-\lambda\cdot\nabla\Delta N$ (双差整周模糊度)",
            ha="center", va="center", fontsize=11.5,
            bbox=dict(fc="#f6f8fa", ec="#cccccc", boxstyle="round,pad=0.5"))
    ax.set_xlim(-0.5, 10.3)
    ax.set_ylim(0.0, 9.9)
    ax.set_title("图1  双差观测量的构造: 星间单差 × 站间单差", fontsize=13)
    save(fig, "01_dd_geometry.png")


# ---------------------------------------------------------------------------
# 图 2: SD 浮点解 → DD 浮点解 (D' 变换与协方差传播)
# ---------------------------------------------------------------------------
def fig_dd_transform():
    rng = np.random.default_rng(3)
    n = 6
    # 构造一个"看起来像" SD 模糊度协方差的正定矩阵
    A = rng.normal(size=(n, n))
    Q_sd = A @ A.T * 0.02 + np.eye(n) * 0.03

    D = np.zeros((n - 1, n))
    for i in range(n - 1):
        D[i, 0] = -1.0          # 基准星索引 ix[2i]
        D[i, i + 1] = 1.0       # 目标星索引 ix[2i+1]
    Q_dd = D @ Q_sd @ D.T

    fig, axes = plt.subplots(1, 3, figsize=(12.6, 3.9),
                             gridspec_kw={"width_ratios": [1.15, 1, 1]})

    ax = axes[0]
    ax.imshow(D, cmap="coolwarm", vmin=-1, vmax=1)
    for (i, j), v in np.ndenumerate(D):
        if v != 0:
            ax.text(j, i, f"{v:+.0f}", ha="center", va="center", fontsize=11)
    ax.set_xticks(range(n))
    ax.set_xticklabels([f"$N_{i+1}$" for i in range(n)])
    ax.set_yticks(range(n - 1))
    ax.set_yticklabels([f"$\\nabla\\Delta_{i+1}$" for i in range(n - 1)])
    ax.set_title("(a) 单差→双差变换矩阵 $D'$\n(rtkpos.c ddidx: ix[2i], ix[2i+1])",
                 fontsize=11)
    ax.set_xlabel("单差相位偏差状态", fontsize=10)
    ax.set_ylabel("双差分量", fontsize=10)

    for ax, Q, ttl in ((axes[1], Q_sd, "(b) 单差协方差 $Q_{SD}$"),
                       (axes[2], Q_dd, "(c) 双差协方差 $Q_{DD}=D'Q_{SD}D'^{T}$")):
        im = ax.imshow(Q, cmap="viridis")
        ax.set_title(ttl, fontsize=11)
        ax.set_xticks([]); ax.set_yticks([])
        plt.colorbar(im, ax=ax, fraction=0.046)
    fig.suptitle("图2  双差量是单差相位偏差状态的线性组合, 协方差随之传播 "
                 r"($y=D'x,\ Q_b=D'Q D'^{T}$)", fontsize=12.5)
    save(fig, "02_dd_transform.png")


# ---------------------------------------------------------------------------
# 图 3+: 浮点解与整数格点 / 去相关 / 搜索
# ---------------------------------------------------------------------------
def _toy_problem():
    """构造一个强相关的二维浮点模糊度问题 (模拟短基线双差)。"""
    a = np.array([2.28, 3.62])
    # 相关系数 0.93, 长短轴比 ~8
    s1, s2, rho = 0.16, 0.42, 0.93
    C = np.array([[s1 ** 2, rho * s1 * s2], [rho * s1 * s2, s2 ** 2]])
    return a, C


def fig_float_lattice():
    a, C = _toy_problem()
    fig, axes = plt.subplots(1, 2, figsize=(11.6, 4.8),
                             gridspec_kw={"width_ratios": [1.25, 1]})

    ax = axes[0]
    for i in range(0, 5):
        for j in range(2, 6):
            ax.plot(i, j, marker=".", color=C_GREY, ms=7, alpha=0.55)
    # 1σ / 2σ / 3σ 置信椭圆
    w, V = np.linalg.eigh(C)
    for k, ls in ((1, "-"), (2, "--"), (3, ":")):
        ang = math.degrees(math.atan2(V[1, -1], V[0, -1]))
        ax.add_patch(Ellipse(a, width=2 * k * math.sqrt(w[-1]),
                             height=2 * k * math.sqrt(w[0]), angle=ang,
                             fill=False, ec=C_FLOAT, lw=1.6, ls=ls))
    ax.plot(*a, marker="*", ms=16, color=C_FLOAT, label=r"浮点解 $\hat{a}$")
    ax.annotate("", xy=(2.0, 4.0), xytext=(a[0], a[1]),
                arrowprops=dict(arrowstyle="->", lw=1.6, color=C_BAD))
    ax.text(1.28, 3.62, r"逐分量取整方向", fontsize=10, color=C_BAD)
    ax.plot(2.0, 4.0, marker="s", ms=9, mfc="none", mec=C_BAD, mew=2,
            label=r"独立取整 $\mathrm{round}(\hat{a})$")
    ils = ils_bruteforce(a, C)[0][0]
    ax.plot(*ils, marker="o", ms=10, mfc="none", mec=C_FIX, mew=2.2,
            label=rf"ILS 整数解 (LAMBDA) = {ils}")
    ax.set_xlim(1.1, 4.0); ax.set_ylim(2.4, 5.0)
    ax.set_xlabel(r"$N_1$ [周]", fontsize=11)
    ax.set_ylabel(r"$N_2$ [周]", fontsize=11)
    ax.set_title("(a) 强相关浮点解与整数格点: 逐分量取整 ≠ 最优整数解", fontsize=11)
    ax.legend(fontsize=9, loc="upper left"); ax.grid(alpha=0.3)

    ax = axes[1]
    xs = np.linspace(-0.6, 1.6, 400)
    sig = math.sqrt(C[0, 0])
    ax.plot(xs, np.exp(-0.5 * (xs / sig) ** 2), color=C_FLOAT, lw=2,
            label=rf"浮点模糊度分布 ($\sigma={sig:.2f}$ 周)")
    ax.axvspan(-0.15, 0.15, color=C_FIX, alpha=0.18)
    ax.annotate("GREAT maxdev\n±0.15 周\n(可接受带)", xy=(0.15, 0.15),
                xytext=(0.95, 0.62), fontsize=9, color=C_FIX,
                arrowprops=dict(arrowstyle="->", lw=1.2, color=C_FIX))
    for k in range(-2, 4):
        ax.axvline(k, color=C_GREY, lw=1.0, ls=":")
    ax.annotate("", xy=(1.0, 0.35), xytext=(0.0, 0.35),
                arrowprops=dict(arrowstyle="->", lw=1.5, color=C_BAD))
    ax.text(0.5, 0.38, r"偏差 0.28 周 $>$ maxdev", ha="center", fontsize=9.5, color=C_BAD)
    ax.set_xlabel(r"$\hat{N}-\mathrm{round}(\hat{N})$ [周]", fontsize=11)
    ax.set_ylabel("概率密度", fontsize=11)
    ax.set_title("(b) 整数性判决: 浮点值离整数太远的候选直接否决", fontsize=11)
    ax.legend(fontsize=9); ax.grid(alpha=0.3)

    fig.suptitle("图3  从浮点解到整数解: 解空间是格点, 判决靠统计量", fontsize=12.5)
    save(fig, "03_float_lattice.png")


def fig_decorrelation():
    a, C = _toy_problem()
    Z = lambda_reduction(C)
    Zinv = np.linalg.inv(Z)
    at = Z.T @ a
    Ct = Z.T @ C @ Z

    fig, axes = plt.subplots(1, 2, figsize=(11.4, 4.9))

    def draw(ax, aa, CC, grid_fn, title, color):
        w, V = np.linalg.eigh(CC)
        ang = math.degrees(math.atan2(V[1, -1], V[0, -1]))
        for k, ls, lw in ((1, "-", 2.0), (2, "--", 1.4), (3, ":", 1.2)):
            ax.add_patch(Ellipse(aa, width=2 * k * math.sqrt(w[-1]),
                                 height=2 * k * math.sqrt(w[0]), angle=ang,
                                 fill=False, ec=color, lw=lw, ls=ls))
        pts = grid_fn()
        ax.plot(pts[:, 0], pts[:, 1], ls="none", marker=".", ms=6,
                color=C_GREY, alpha=0.6)
        ax.plot(*aa, marker="*", ms=15, color=color)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=11)
        ax.grid(alpha=0.25)

    g = np.array([[i, j] for i in range(-3, 6) for j in range(-3, 6)], float)
    draw(axes[0], a, C, lambda: g, "(a) 原始问题: 置信椭球极度拉长,\n搜索空间大, 候选多",
         C_BAD)
    axes[0].set_xlabel(r"$a_1$ [周]"); axes[0].set_ylabel(r"$a_2$ [周]")
    # 变换后的整数格点: a = Z^{-T} z
    gz = np.array([[i, j] for i in range(-4, 5) for j in range(-4, 5)], float)
    axes[0].plot([], [], ls="none")

    def grid_t():
        return (Zinv.T @ gz.T).T

    draw(axes[1], at, Ct, grid_t, "(b) $Z$ 变换后: 近球形椭球,\n逐层搜索即可快速收敛", C_FIX)
    axes[1].set_xlabel(r"$z_1$ [周]"); axes[1].set_ylabel(r"$z_2$ [周]")
    ell = plt.matplotlib.patches.Ellipse((0, 0), 0, 0)
    fig.text(0.5, 0.005,
             r"$z=Z^{T}a,\ Q_z=Z^{T}QZ$; 变换 $Z$ 为整数矩阵 "
             r"($\det Z=\pm1$), 保持格点不变只改变基底 —— LAMBDA 去相关的本质",
             ha="center", fontsize=10.5)
    fig.suptitle("图4  LAMBDA 去相关的几何意义", fontsize=12.5)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    save(fig, "04_decorrelation.png")


def fig_mlambda_search():
    a, C = _toy_problem()
    Z = lambda_reduction(C)
    a_t = Z.T @ a
    C_t = Z.T @ C @ Z
    cands = ils_bruteforce(a_t, C_t, span=3)

    fig, axes = plt.subplots(1, 2, figsize=(11.4, 4.8))
    ax = axes[0]
    grid = np.array([[i, j] for i in range(-3, 6) for j in range(-3, 6)], float)
    ax.plot(grid[:, 0], grid[:, 1], ls="none", marker=".", color=C_GREY, ms=6,
            alpha=0.5, label="整数格点")
    w, V = np.linalg.eigh(C_t)
    ang = math.degrees(math.atan2(V[1, -1], V[0, -1]))
    best = cands[0][1]
    for k, (mul, ls) in enumerate(((1.0, "-"), (1.5, "--"), (2.2, ":"))):
        ax.add_patch(Ellipse(a_t, 2 * math.sqrt(w[-1] * best * mul),
                             2 * math.sqrt(w[0] * best * mul), angle=ang,
                             fill=False, ec=C_FLOAT, ls=ls, lw=1.4))
    for idx, (z, d) in enumerate(cands[:6]):
        col = C_FIX if idx == 0 else C_BAD
        ax.plot(*z, marker="o", ms=13 if idx == 0 else 9, mfc="none",
                mec=col, mew=2)
        dx, dy = (0.06, -0.28) if idx == 0 else (0.10, 0.10)
        ax.text(z[0] + dx, z[1] + dy, f"{idx}: {d:.2f}", fontsize=9, color=col)
    ax.plot(*a_t, marker="*", ms=15, color=C_FLOAT, label=r"变换后浮点解 $z$")
    ax.set_xlim(a_t[0] - 2.2, a_t[0] + 2.2); ax.set_ylim(a_t[1] - 2.2, a_t[1] + 2.2)
    ax.set_xlabel(r"$z_1$"); ax.set_ylabel(r"$z_2$")
    ax.set_title("(a) 椭球约束内枚举候选, 每找到更优解即收缩 maxdist", fontsize=11)
    ax.legend(fontsize=9); ax.grid(alpha=0.25)

    ax = axes[1]
    ds = [d for _, d in cands[:8]]
    ax.bar(range(len(ds)), ds, color=[C_FIX] + [C_BAD] * (len(ds) - 1))
    ax.set_xlabel("候选解序号 (按二次型残差升序)", fontsize=10.5)
    ax.set_ylabel(r"$(\hat{z}-z)^{T}Q_z^{-1}(\hat{z}-z)$", fontsize=11)
    ax.set_title(r"ratio $=s_2/s_1$ 由前两个候选的残差决定", fontsize=11)
    ax.text(0.03, 0.9, f"s1={ds[0]:.2f}  s2={ds[1]:.2f}  ratio={ds[1]/ds[0]:.2f}",
            transform=ax.transAxes, fontsize=10.5,
            bbox=dict(fc="#f6f8fa", ec="#cccccc", boxstyle="round,pad=0.35"))
    ax.grid(alpha=0.25, axis="y")
    fig.suptitle("图5  MLAMBDA 搜索: 与整数最小二乘等价的有限枚举", fontsize=12.5)
    save(fig, "05_mlambda_search.png")


def fig_ratio_test():
    fig, axes = plt.subplots(1, 2, figsize=(11.6, 4.4))
    x = np.linspace(1.0, 8.0, 600)
    ax = axes[0]
    # 概率密度示意: 正确固定倾向大 ratio, 错误固定倾向小 ratio
    p_ok = (x - 1) ** 4 * np.exp(-(x - 1) / 0.75)
    p_ok /= np.trapezoid(p_ok, x)
    p_bad = np.exp(-(x - 1) / 0.55)
    p_bad /= np.trapezoid(p_bad, x)
    ax.plot(x, p_ok, color=C_FIX, lw=2.2, label="正确固定的 ratio 分布 (示意)")
    ax.plot(x, p_bad, color=C_BAD, lw=2.2, label="错误固定的 ratio 分布 (示意)")
    ax.fill_between(x, 0, p_bad, where=(x < 3.0), color=C_BAD, alpha=0.18)
    ax.fill_between(x, 0, p_ok, where=(x >= 3.0), color=C_FIX, alpha=0.15)
    ax.axvline(3.0, color=C_GREY, ls="--", lw=1.6)
    ax.text(3.12, max(p_ok) * 0.92, "ratio 门限 = 3.0\n(右侧接受)", fontsize=10)
    ax.annotate("阈值以下的失败大多\n是错误固定 ⇒ 被拒绝", xy=(1.75, 0.05),
                xytext=(1.15, max(p_ok) * 0.55), fontsize=9, color=C_BAD,
                arrowprops=dict(arrowstyle="->", lw=1.1, color=C_BAD))
    ax.set_xlabel(r"ratio $=s_2/s_1$")
    ax.set_ylabel("概率密度 (归一化)", fontsize=10)
    ax.set_title("(a) ratio 检验: 次优解残差必须显著大于最优解", fontsize=11)
    ax.legend(fontsize=9); ax.grid(alpha=0.3)

    ax = axes[1]
    ax.set_axis_off()
    boxes = [
        (r"$s_1$ 与 $s_2$", "LAMBDA 返回前两个解及其二次型残差", "#eef5ff"),
        (r"ratio $=s_2/s_1 \geq$ thresar", "GREAT <ratio> 3.0; rnx2rtkp 可随卫星数自适应", "#eaf7ea"),
        (r"maxsig $\leq$ 0.10 周", "GREAT narrowlane/widelane_decision: 浮点解标准差", "#fff7e6"),
        (r"maxdev $\leq$ 0.15 周", "GREAT: 浮点解与整数解的最大偏差", "#fff7e6"),
        ("全部通过 ⇒ 固定", "否则保持浮点; 部分固定时剔除候选后重试", "#fdeaea"),
    ]
    y = 0.86
    for title, desc, col in boxes:
        ax.add_patch(FancyBboxPatch((0.03, y - 0.13), 0.94, 0.15,
                                    boxstyle="round,pad=0.012",
                                    fc=col, ec="#bbbbbb"))
        ax.text(0.06, y - 0.03, title, fontsize=11.5, va="top")
        ax.text(0.06, y - 0.085, desc, fontsize=9.5, va="top", color="#444444")
        y -= 0.185
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.set_title("(b) 实际工程中的多重判决 (GREAT / gipylib 采用)", fontsize=11)
    fig.suptitle("图6  比值检验与整数性判决: 固定不是“四舍五入”, 而是统计判定",
                 fontsize=12.5)
    save(fig, "06_ratio_test.png")


def fig_workflow():
    fig, ax = plt.subplots(figsize=(11.0, 5.6))
    ax.set_axis_off()

    steps = [
        ("① 浮点解", "卡尔曼滤波输出\n$\\hat{a},\\ Q_{\\hat a}$ (单差相位偏差)", "#eef5ff"),
        ("② 构成双差", r"$y=D'\hat a$" "\n" r"$Q_b=D'Q D'^{T}$", "#eef5ff"),
        ("③ LD 分解", r"$Q_b=L'DL$" "\n" "lambda.c: LD()", "#f3e8ff"),
        ("④ 整数去相关", "整数高斯变换 + 置换\nlambda.c: gauss() / perm()\n"
                        "reduction()", "#f3e8ff"),
        ("⑤ 搜索", r"$\min_z (z-\hat z)^{T}Q_z^{-1}(z-\hat z)$"
                   "\nlambda.c: search()", "#eaf7ea"),
        ("⑥ 逆变换", r"$b=Z'^{-1}z$" "\n返回 $b_1,b_2,s_1,s_2$", "#eaf7ea"),
        ("⑦ 有效性检验", r"ratio $=s_2/s_1$; maxsig/maxdev" "\n通过才继续", "#fff7e6"),
        ("⑧ 回代与保持", "非模糊度状态回代\n+ restamb / holdamb", "#fdeaea"),
    ]
    for i, (title, desc, col) in enumerate(steps):
        col_i, row_i = i % 4, i // 4
        x, y = 0.02 + col_i * 0.245, 0.60 - row_i * 0.42
        ax.add_patch(FancyBboxPatch((x, y), 0.215, 0.30,
                                    boxstyle="round,pad=0.018",
                                    fc=col, ec="#999999"))
        ax.text(x + 0.108, y + 0.245, title, ha="center", fontsize=11.5)
        ax.text(x + 0.108, y + 0.115, desc, ha="center", fontsize=9.2,
                color="#333333")
        if col_i < 3:
            ax.add_patch(FancyArrowPatch((x + 0.215, y + 0.15),
                                         (x + 0.245, y + 0.15),
                                         arrowstyle="->", mutation_scale=13,
                                         lw=1.4, color="#666666"))
    ax.add_patch(FancyArrowPatch((0.775, 0.60), (0.775, 0.48), arrowstyle="-",
                                 lw=1.4, color="#666666"))
    ax.add_patch(FancyArrowPatch((0.775, 0.48), (0.095, 0.48), arrowstyle="-",
                                 lw=1.4, color="#666666"))
    ax.add_patch(FancyArrowPatch((0.095, 0.48), (0.095, 0.45), arrowstyle="->",
                                 mutation_scale=13, lw=1.4, color="#666666"))
    ax.text(0.5, 0.985, "图7  LAMBDA 模糊度固定全流程及其与 rtklib 代码的对应关系",
            ha="center", fontsize=12.5)
    ax.text(0.5, 0.03,
            "关键: ③④ 只做“换基底”(整数可逆变换, 格点不变), ⑤ 才是搜索; "
            "因此固定结果与直接整数最小二乘等价, 但计算量从指数级降为可接受。",
            ha="center", fontsize=10, color="#444444")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    save(fig, "07_workflow.png")


# ---------------------------------------------------------------------------
# 图 8: 真实数据案例 (gipylib data/cpt0870)
# ---------------------------------------------------------------------------
def _load_rslt(path):
    """返回 (sow, lat_rad, lon_rad, height, Q)。.rslt 中经纬度为度, 统一转弧度。"""
    rows = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if not line.strip() or line.startswith("%"):
                continue
            p = line.split()
            if len(p) < 7:
                continue
            try:
                rows.append((float(p[1]), math.radians(float(p[2])),
                             math.radians(float(p[3])), float(p[4]), p[5]))
            except ValueError:
                continue
    rows.sort(key=lambda r: r[0])
    return rows


def _load_truth(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append((float(r["sow"]), math.radians(float(r["lat_deg"])),
                         math.radians(float(r["lon_deg"])), float(r["height_m"])))
    rows.sort(key=lambda r: r[0])
    return rows


def _enu(lat0, lon0, h0, lat, lon, h):
    sa, ca = math.sin(lat0), math.cos(lat0)
    dn = 6378137.0 * (1 - 6.69437999e-3) / (1 - 6.69437999e-3 * sa ** 2) ** 1.5 \
        * (lat - lat0)
    de = 6378137.0 * ca * (lon - lon0)
    return de, dn, h - h0


def fig_real_case():
    ar = os.path.join(REPO, "data/output/RTKTCAR.rslt")
    base = os.path.join(REPO, "data/output/RTKINS.rslt")
    truth_path = os.path.join(REPO, "data/truth.csv")
    if not all(os.path.exists(p) for p in (ar, base, truth_path)):
        print("skip real-case figure (缺少 data/output 产物)")
        return

    truth = [t for t in _load_truth(truth_path) if t[0] >= 359000]

    def errors(path):
        sol = _load_rslt(path)
        sows = [s[0] for s in sol]
        out = []
        for sow, lat, lon, h in truth:
            i = bisect_left(sows, sow)
            best, bd = None, None
            for c in (i - 1, i, i + 1):
                if 0 <= c < len(sol):
                    d = abs(sol[c][0] - sow)
                    if bd is None or d < bd:
                        best, bd = sol[c], d
            if best is None or bd is None or bd > 0.05:
                continue
            de, dn, du = _enu(lat, lon, h, best[1], best[2], best[3])
            out.append((sow, de, dn, du, best[4]))
        return out

    sol_ar = _load_rslt(ar)
    e_ar, e_base = errors(ar), errors(base)
    fig, axes = plt.subplots(1, 3, figsize=(14.2, 4.3),
                             gridspec_kw={"width_ratios": [1.35, 1, 0.85]})

    ax = axes[0]
    tail = [r for r in sol_ar if r[0] >= 358000]
    sows = [r[0] for r in tail]
    qs = [1 if r[4].split(".")[0] == "1" else 0 for r in tail]
    ax.fill_between(sows, 0, qs, step="mid", color=C_FIX, alpha=0.35,
                    label="Q=1 固定")
    ax.fill_between(sows, 0, [1 - q for q in qs], step="mid", color=C_FLOAT,
                    alpha=0.30, label="Q=2 浮点")
    ax.axvline(358754, color=C_BAD, ls="--", lw=1.4)
    ax.text(358762, 0.62, "首次固定\nSOW 358754", fontsize=9.5, color=C_BAD)
    ax.set_ylim(0, 1.05); ax.set_xlim(358000, max(sows))
    ax.set_xlabel("GPST 周内秒 [s]"); ax.set_ylabel("状态占比")
    ax.set_title("(a) 模糊度状态时间线 (data/rtktcar.yaml)", fontsize=11)
    ax.legend(fontsize=9, loc="center left"); ax.grid(alpha=0.3)

    ax = axes[1]
    for tag, errs, col in (("AR 固定 (rtktcar)", e_ar, C_FIX),
                           ("纯 GPS 单频浮点 (基线)", e_base, C_GREY)):
        y = np.array([r[2] for r in errs])
        ax.plot([r[0] for r in errs], y - y.mean(), lw=1.1, color=col, label=tag)
    ax.set_xlabel("GPST 周内秒 [s]")
    ax.set_ylabel("高程误差 (去常偏) [m]")
    ax.set_title("(b) 高程误差序列", fontsize=11)
    ax.legend(fontsize=9); ax.grid(alpha=0.3)

    ax = axes[2]
    labels = ["rmsE", "rmsN", "rmsU", "3D"]
    def stats(errs):
        e = np.array([[r[1], r[2], r[3]] for r in errs])
        re, rn, ru = [math.sqrt((e[:, i] ** 2).mean()) for i in range(3)]
        return [re, rn, ru, math.sqrt(re ** 2 + rn ** 2 + ru ** 2)]
    sa, sb = stats(e_ar), stats(e_base)
    x = np.arange(len(labels))
    ax.bar(x - 0.19, sa, 0.38, color=C_FIX, label="AR 固定")
    ax.bar(x + 0.19, sb, 0.38, color=C_GREY, label="浮点基线")
    ymax = max(max(sa), max(sb))
    for xi, (va, vb) in enumerate(zip(sa, sb)):
        ax.text(xi - 0.19, va + ymax * 0.03, f"{va:.3f}", ha="center",
                fontsize=8, rotation=90, va="bottom")
        ax.text(xi + 0.19, vb + ymax * 0.03, f"{vb:.3f}", ha="center",
                fontsize=8, rotation=90, va="bottom")
    ax.set_ylim(0, ymax * 1.35)
    ax.set_xticks(x); ax.set_xticklabels(labels)
    ax.set_ylabel("RMSE [m]")
    ax.set_title("(c) 稳态精度对比 (SOW≥359000)", fontsize=11)
    ax.legend(fontsize=9); ax.grid(alpha=0.3, axis="y")

    fig.suptitle("图8  实测案例 (data 数据集, cpt0870): 固定成功后精度提升",
                 fontsize=12.5)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    save(fig, "08_real_case.png")


def main():
    fig_dd_geometry()
    fig_dd_transform()
    fig_float_lattice()
    fig_decorrelation()
    fig_mlambda_search()
    fig_ratio_test()
    fig_workflow()
    fig_real_case()


if __name__ == "__main__":
    main()
