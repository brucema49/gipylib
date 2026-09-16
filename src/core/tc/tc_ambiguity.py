"""紧组合模糊度固定 (调 mlambda, 不操作全局 nav.x/P)。

策略参考 GREAT-MSF ``LibGREAT/gambfix/gambiguity.cpp`` + rtklib
``manage_amb_LAMBDA``:

  - 候选数门限: DD 模糊度个数 < ``min_amb``(GREAT ``full_fix_num``=3) 不做搜索
  - 最小共视历元: 卫星连续参与历元数 < ``min_common_epochs``
    (GREAT ``min_common_time``=30 s, 按 1 Hz 折算为历元) 的 DD 先剔除/不参与
  - ratio 检验: ratio = s[1]/s[0] > ``thresar``(GREAT ``ratio``=3.0) 才接受
  - 部分固定 (GREAT ``part_fix``): 全量未通过时, 按「锁定不足优先、其次方差
    最大」逐个剔除候选并重试, 剩余个数不低于 ``part_min_amb``(GREAT
    ``part_fix_num``=2)
  - 位置方差门限 (rtklib thresar1): posvar > ``thresar_var`` 跳过
  - fix-and-hold: 连续 ``hold_count`` 历元固定解一致 → hold; 通过约束量测收缩 P
"""
import numpy as np

from src.core.gnss.rtklib.mlambda import mlambda


# GREAT gsetamb 默认值 (见 LibGREAT/gset/gsetamb.cpp)
GREAT_RATIO_DEFAULT = 3.0      # <ratio> 3.0
GREAT_FULL_FIX_NUM = 3         # <full_fix_num> 3
GREAT_PART_FIX_NUM = 2         # <part_fix_num> 2
GREAT_MIN_COMMON_TIME_S = 30   # <min_common_time> 30
GREAT_MIN_LOCK_EPO = 5         # _lock_epo_num < 5 的卫星在部分固定时优先剔除
# GREAT widelane/narrowlane_decision: maxdev=0.15 周, maxsig=0.10 周
GREAT_MAXDEV_CYCLES = 0.15
GREAT_MAXSIG_CYCLES = 0.10


class TcAmbiguity:
    """RTK-INS 模糊度管理器。

    直接调 mlambda(a, Q, m=2) 做整数搜索, 不依赖 rtklib 全局 nav.x/P 状态。
    """

    def __init__(self, thresar: float = GREAT_RATIO_DEFAULT,
                 thresar_var: float = 0.5, hold_count: int = 10,
                 min_amb: int = GREAT_FULL_FIX_NUM,
                 min_common_epochs: int = 0,
                 part_fix: bool = False,
                 part_min_amb: int = GREAT_PART_FIX_NUM,
                 min_lock_epo: int = GREAT_MIN_LOCK_EPO,
                 maxdev: float = GREAT_MAXDEV_CYCLES,
                 maxsig: float = GREAT_MAXSIG_CYCLES):
        self.thresar = float(thresar)
        self.thresar_var = float(thresar_var)
        self.hold_threshold = int(hold_count)
        self.min_amb = int(min_amb)
        self.min_common_epochs = int(min_common_epochs)
        self.part_fix = bool(part_fix)
        self.part_min_amb = int(part_min_amb)
        self.min_lock_epo = int(min_lock_epo)
        self.maxdev = float(maxdev)
        self.maxsig = float(maxsig)
        self._last_fixed = None
        self._hold_count = 0
        self._is_holding = False
        # 卫星连续参与历元数 (GREAT _lock_epo_num 语义)
        self._lock_epo_num: dict = {}
        # 诊断: 上一次搜索的结果
        self.last_fixed_mask = None
        self.last_n_candidate = 0
        self.last_partial = False
        self.last_maxdev = None   # max|浮点值 − 整数| [周]
        self.last_maxsig = None   # max sqrt(diag(Q)) [周]

    # ===== 锁定/共视历元统计 =====
    def update_lock(self, sats) -> None:
        """更新卫星连续参与历元数（本历元未出现的卫星清零）。"""
        present = set(int(s) for s in sats)
        for sat in list(self._lock_epo_num):
            if sat not in present:
                self._lock_epo_num[sat] = 0
        for sat in present:
            self._lock_epo_num[sat] = self._lock_epo_num.get(sat, 0) + 1

    def lock_epochs(self, sat) -> int:
        return int(self._lock_epo_num.get(int(sat), 0))

    def reset_lock(self) -> None:
        self._lock_epo_num.clear()

    # ===== 固定 =====
    def try_fix(self, x_amb: np.ndarray, P_amb: np.ndarray,
                posvar: float = 0.0, sat_pairs=None):
        """尝试 LAMBDA 固定。

        Args:
            x_amb: 浮点模糊度 (n,)
            P_amb: 模糊度协方差 (n, n)
            posvar: 位置方差 (P[pos,pos] 对角均值), 超阈值时跳过 AR
            sat_pairs: 可选 [(sat_i, sat_j), ...]（与 x_amb 对齐）; 提供时按
                ``min_common_epochs`` 剔除共视不足的候选（GREAT min_common_time）

        Returns:
            (fixed[n], ratio, ok)
            fixed: 整数解; 部分固定时未固定的分量为 NaN, ok=False 时为空数组
        """
        x = np.asarray(x_amb, dtype=np.float64)
        n = len(x)
        self.last_fixed_mask = np.zeros(n, dtype=bool)
        self.last_partial = False
        if n == 0:
            return np.array([]), 0.0, False
        if posvar > self.thresar_var:
            return np.array([]), 0.0, False

        P = np.asarray(P_amb, dtype=np.float64)
        P = 0.5 * (P + P.T)
        eig_min = float(np.min(np.linalg.eigvalsh(P)))
        if eig_min <= 1e-9:
            jitter = max(1e-6, 1e-9 * float(np.trace(P)) / n)
            P = P + np.eye(n) * (jitter - min(eig_min, 0.0))

        # 共视时间筛选 (GREAT: (dd.end_epo - dd.beg_epo) < min_common_time 剔除)
        active = np.ones(n, dtype=bool)
        if (sat_pairs is not None and self.min_common_epochs > 0
                and len(sat_pairs) == n):
            for k, pair in enumerate(sat_pairs):
                if any(self.lock_epochs(s) < self.min_common_epochs
                       for s in pair):
                    active[k] = False
            if int(active.sum()) < self.min_amb:
                return np.array([]), 0.0, False

        idx = np.nonzero(active)[0]
        self.last_n_candidate = int(len(idx))
        # 候选数门限 (GREAT full_fix_num)
        if len(idx) < self.min_amb:
            return np.array([]), 0.0, False

        y = x[idx]
        Q = P[np.ix_(idx, idx)]
        ratio, afix = self._search(y, Q)
        ok = ratio >= self.thresar
        # GREAT widelane/narrowlane_decision: 整数性判决
        # (max|浮点−整数| <= maxdev 周, sigma <= maxsig 周), 防止 false fix
        if ok and afix is not None:
            dev = float(np.max(np.abs(y - afix[:, 0])))
            sig = float(np.max(np.sqrt(np.diag(Q))))
            self.last_maxdev, self.last_maxsig = dev, sig
            if dev > self.maxdev or sig > self.maxsig:
                ok = False

        # 部分固定 (GREAT part_fix): 逐个剔除「锁定不足 / 方差最大」的候选后重试
        if not ok and self.part_fix:
            keep = np.ones(len(idx), dtype=bool)
            while int(keep.sum()) > max(self.part_min_amb, 1):
                drop = self._worst_index(y, Q, keep, sat_pairs, idx)
                if drop is None:
                    break
                keep[drop] = False
                if int(keep.sum()) < self.part_min_amb:
                    break
                sub = np.nonzero(keep)[0]
                y_sub = y[sub]
                Q_sub = Q[np.ix_(sub, sub)]
                ratio_sub, afix_sub = self._search(y_sub, Q_sub)
                ok_sub = ratio_sub >= self.thresar
                if ok_sub and afix_sub is not None:
                    dev_sub = float(np.max(np.abs(y_sub - afix_sub[:, 0])))
                    sig_sub = float(np.max(np.sqrt(np.diag(Q_sub))))
                    self.last_maxdev, self.last_maxsig = dev_sub, sig_sub
                    ok_sub = dev_sub <= self.maxdev and sig_sub <= self.maxsig
                if ok_sub:
                    ratio, afix, idx, ok = ratio_sub, afix_sub, idx[sub], True
                    self.last_partial = True
                    break

        if not ok:
            return np.array([]), float(ratio), False

        fixed_subset = np.asarray(afix[:, 0], dtype=np.float64)
        fixed = np.full(n, np.nan)
        fixed[idx] = fixed_subset
        mask = np.zeros(n, dtype=bool)
        mask[idx] = True
        self.last_fixed_mask = mask

        # fix-and-hold: 仅在同一固定集合连续出现时累计
        signature = (tuple(idx.tolist()),
                     tuple(np.round(fixed_subset, 6).tolist()))
        if self._last_fixed == signature:
            self._hold_count += 1
            if self._hold_count >= self.hold_threshold:
                self._is_holding = True
        else:
            self._last_fixed = signature
            self._hold_count = 1
        return fixed, float(ratio), True

    def _search(self, y: np.ndarray, Q: np.ndarray):
        """LAMBDA 搜索, 返回 (ratio, afix)。失败返回 (0.0, None)。"""
        n = len(y)
        Q = 0.5 * (Q + Q.T)
        eig_min = float(np.min(np.linalg.eigvalsh(Q))) if n else 0.0
        if eig_min <= 1e-9:
            jitter = max(1e-6, 1e-9 * float(np.trace(Q)) / n)
            Q = Q + np.eye(n) * (jitter - min(eig_min, 0.0))
        try:
            afix, s = mlambda(y, Q, m=2)
        except np.linalg.LinAlgError:
            return 0.0, None
        ratio = float(s[1] / s[0]) if s[0] > 1e-12 else 0.0
        return ratio, afix

    def _worst_index(self, y, Q, keep, sat_pairs, idx):
        """返回待剔除候选在子集内的下标 (GREAT: 锁定不足优先, 其次方差最大)。"""
        sub = np.nonzero(keep)[0]
        if len(sub) == 0:
            return None
        # 1) 锁定历元不足 (GREAT _lock_epo_num < 5)
        if sat_pairs is not None and len(sat_pairs) == len(y):
            weak = [k for k in sub
                    if any(self.lock_epochs(s) < self.min_lock_epo
                           for s in sat_pairs[int(idx[k])])]
            if weak:
                diag = np.diag(Q)
                return int(max(weak, key=lambda k: diag[k]))
        # 2) 方差最大
        diag = np.diag(Q)
        return int(max(sub, key=lambda k: diag[k]))

    @property
    def is_holding(self) -> bool:
        return self._is_holding

    def reset(self):
        """降级/重启时清空。"""
        self._last_fixed = None
        self._hold_count = 0
        self._is_holding = False
        self.last_fixed_mask = None
        self.last_partial = False
        self.reset_lock()

    def apply_correction(self, x: np.ndarray, fixed: np.ndarray,
                         amb_slice: slice):
        """固定后将浮点解替换为整数解（NaN 分量保持浮点）。"""
        values = np.asarray(fixed, dtype=np.float64)
        for k, value in enumerate(values):
            if np.isfinite(value):
                x[amb_slice][k] = value
