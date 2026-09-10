"""紧组合导航流式运行器。

类似 LcStream，但使用 TcIntegration 替代 LcIntegration。
消费 IMU + 原始 GNSS 观测 (obsr, obsb, nav)，流式输出 .rslt。

流程:
1. 初始化前：小缓冲累积 IMU+GNSS 原始观测，每个新 GNSS 到来时尝试初始化
2. 初始化成功：回放缓冲数据，然后切换增量模式
3. 增量模式：feed_imu 委托 TcIntegration 做 time_update
   feed_gnss_raw 入 pending 队列（由后续 IMU 跨越 gnss.t 时 GVINS 风格触发）
4. 输出：每次真实机械编排提交写一行 (100Hz)，由 TcIntegration 的
   output_callback 在提交点触发；GNSS 边界拆分时 head/tail 各写一个真实端点
"""
import logging
from typing import List, Optional

import numpy as np

from src.core.data_types import ImuMeasurement
from src.core.gnss.rtklib.rtkcmn import sat2prn, uGNSS
from src.core.tc.tc_integration import TcIntegration

logger = logging.getLogger(__name__)


def _filter_gps_svh(obsr, svh):
    """将非 GPS 卫星的 svh 标记为非零, 使 estpos/satposs 跳过它们。

    用于 SPP 单点定位: BDS/GAL 时间系统偏差 (BDT-GPST ~14s, GST-GPST)
    在 rtklib-py 中未完全处理, 多星座 SPP 会发散。
    TC 量测更新有自己的 clk_bias 状态处理多星座, 不受此限制。
    """
    filtered_svh = svh.copy()
    for i in range(len(obsr.sat)):
        sys, _ = sat2prn(obsr.sat[i])
        if sys != uGNSS.GPS:
            filtered_svh[i] = 99  # 非零 = 不健康, estpos 会跳过
    return filtered_svh


class TcStream:
    """流式紧组合导航：增量喂入 IMU+原始GNSS观测，per-IMU 流式输出 .rslt。

    与 LcStream 的区别：
    - 使用 TcIntegration (原始观测融合) 替代 LcIntegration (GnssSolution 融合)
    - feed_gnss 接收原始观测 (obsr, obsb, nav) 而非 GnssSolution
    - 初始化由 TcIntegration._try_init 内部完成 (SPP 粗定位)
    """

    def __init__(self, config: dict, writer, stat_writer=None):
        self.config = config
        self.writer = writer
        self.stat_writer = stat_writer
        self._tc_mode = config.get("gnss", {}).get("positioning_mode", "spp")
        # IMU 时标固定偏移 [s]: 部分数据集 IMR 时标相对 GNSS 存在系统滞后
        # (Data19 HG4930 实测 ≈ -0.75s: 真值起步/时移扫描/逐历元匹配三重验证),
        # 在喂入边界统一修正, 使 INS 状态内容与时间戳对齐。
        import dataclasses
        self._imu_dataclasses = dataclasses
        self._imu_time_offset = float(
            config.get("ins", {}).get("imu_time_offset_s", 0.0))
        self._integ = TcIntegration(
            config, mode=self._tc_mode, output_callback=self._write_integration_output
        )

        # 初始化前缓冲
        self._init_imu: List[ImuMeasurement] = []
        self._init_obs: list = []   # [(obsr, obsb, nav, t_gnss), ...]
        self._initialized = False
        self._output_count = 0
        self._last_init_gnss_t: float = -1.0  # 避免重复触发 _try_init

    @property
    def initialized(self) -> bool:
        return self._initialized

    def open(self) -> None:
        self.writer.open()
        if self.stat_writer is not None:
            self.stat_writer.open()

    def close(self) -> None:
        self._integ.close()
        if self.stat_writer is not None:
            self.stat_writer.close()
        self.writer.close()

    def finalize(self) -> int:
        """流式结束，返回总输出数。"""
        if not self._initialized:
            logger.warning("TcStream: 未初始化，无 TC 输出")
            return 0
        logger.info(f"TcStream 输出: {self._output_count} 历元")
        return self._output_count

    # ===== 增量喂入 =====

    def feed_imu(self, imu: ImuMeasurement) -> None:
        """喂入 IMU。初始化前缓冲，初始化后委托 TcIntegration。

        TC 的写出由 ``TcIntegration`` 在每个真实机械编排提交后通过
        ``output_callback`` 触发: 每个提交一行, GNSS 边界拆分时 head/tail
        各写一个真实端点, 不做 per-IMU 的无条件补写。
        """
        if self._imu_time_offset != 0.0:
            imu = self._imu_dataclasses.replace(
                imu, timestamp=imu.timestamp + self._imu_time_offset)
        if not self._initialized:
            self._init_imu.append(imu)
            # 修剪 _init_imu: 只保留最新 GNSS 历元前 10s 的数据
            # _try_init 只需 t_gnss ± 2s 窗口, 10s 足够覆盖
            # 防止 _init_imu 无限增长导致 O(n²) 性能问题
            if self._init_obs and len(self._init_imu) > 2000:
                cutoff = self._init_obs[-1][3] - 10.0
                self._init_imu = [m for m in self._init_imu
                                  if m.timestamp >= cutoff]
            # 当有 GNSS obs 且当前 IMU 时间戳 >= 最新 GNSS 时间戳时, 尝试初始化
            # 此时 IMU 数据已包夹 GNSS 时间戳 (before + after), 满足插值条件
            # 每个 GNSS 历元只尝试一次 (避免 100Hz IMU 重复触发昂贵的 relpos)
            if self._init_obs and imu.timestamp >= self._init_obs[-1][3]:
                if self._last_init_gnss_t != self._init_obs[-1][3]:
                    self._last_init_gnss_t = self._init_obs[-1][3]
                    self._try_init()
            return
        self._integ.add_imu(imu)

    def feed_gnss_raw(self, obsr, obsb, nav) -> None:
        """喂入原始 GNSS 观测。初始化前缓冲并输出纯GNSS解, 初始化后委托 TcIntegration。"""
        if not self._initialized:
            t_gnss = float(obsr.t.time + obsr.t.sec)
            self._init_obs.append((obsr, obsb, nav, t_gnss))
            # 未初始化时输出纯 GNSS 解 (Qins=0, 1Hz GNSS频率, 姿态=0)
            self._write_gnss_only(obsr, obsb, nav, t_gnss)
            # 不在此处调用 _try_init: feed_gnss_raw 在 after-GNSS IMU 之前被调用,
            # 此时 IMU 缓冲只有 before-GNSS 数据, 不满足包夹条件。
            # _try_init 由 feed_imu 在 after-GNSS IMU 到达时触发。
            return
        self._integ.add_gnss(obsr, obsb, nav)

    # ===== 初始化 =====

    def _try_init(self) -> None:
        """委托 TcIntegration._try_init 完成初始化。"""
        # 只传递最新 GNSS 历元附近的 IMU 数据 (避免 O(n) 拷贝全部缓冲)
        # _try_init 内部用 t_gnss ± 2s 窗口过滤, 这里预剪枝到 ±5s 减少拷贝量
        if self._init_obs:
            t_gnss = self._init_obs[-1][3]
            imu_window = [imu for imu in self._init_imu
                          if t_gnss - 5.0 <= imu.timestamp <= t_gnss + 5.0]
        else:
            imu_window = list(self._init_imu)
        self._integ._init_imu = imu_window
        self._integ._init_obs = list(self._init_obs)
        self._integ._try_init()

        if self._integ.initialized:
            self._initialized = True
            logger.info(
                f"TcStream 初始化成功: mode={self._tc_mode}, "
                f"pos={self._integ.state.pos_e}")
            # 清空初始化缓冲 (TcIntegration._replay_buffer 已回放)
            self._init_imu.clear()
            self._init_obs.clear()

    # ===== 输出 =====

    def _write_integration_output(self, state, P, qins: int) -> None:
        """Write exactly one row per successful TC mechanization commit."""
        si = self._integ.si
        if state is None or P is None or si is None:
            return
        self._write_state_values(state, P, si, qins)

    def _write_state_values(self, state, P, si, qins: int) -> None:
        """Write one mechanization state row to the main writer and stat writer."""
        q = self._integ._last_q if hasattr(self._integ, '_last_q') else 5
        num_sv = self._integ._last_ns if hasattr(self._integ, '_last_ns') else 0
        self.writer.write(state, P, si, q, qins, num_sv)
        if self.stat_writer is not None:
            self.stat_writer.write(
                state, P, si, q, qins, num_sv,
                update_info=getattr(self._integ, "last_update_info", None),
            )
        self._output_count += 1

    def _write_gnss_only(self, obsr, obsb, nav, t_gnss: float) -> None:
        """未初始化时输出纯 GNSS 解 (Qins=0, 速度=0, 姿态=0)。

        ``positioning_mode`` 为 rtk/rtd 时使用 rover/base 相对解；SPP
        仅作为 relpos 的数值初值，不作为相对模式的输出回退。
        输出频率为 GNSS 频率 (1Hz)。
        """
        import numpy as np

        # 保存/恢复所有 relpos/SPP 可能修改的导航状态。
        saved_x = nav.x.copy()
        saved_P = nav.P.copy()
        saved_fix = nav.fix.copy() if hasattr(nav, "fix") else None
        saved_lock = nav.lock.copy() if hasattr(nav, "lock") else None
        try:
            from src.core.gnss.rtklib.ephemeris import satposs
            from src.core.gnss.rtklib.pntpos import estpos
            rs, var, dts, svh = satposs(obsr, nav)
            svh_gps = _filter_gps_svh(obsr, svh)  # GPS-only SPP
            if np.any(nav.rb):
                nav.x[0:3] = nav.rb
            sol, x_spp = estpos(obsr, nav, rs[:, :3], dts, svh_gps)
            if not sol.stat:
                return
        except Exception as e:
            logger.debug(f"GNSS-only SPP 异常: {e}")
            return
        finally:
            nav.x[:] = saved_x
            nav.P[:] = saved_P
            if saved_fix is not None:
                nav.fix[:] = saved_fix
            if saved_lock is not None:
                nav.lock[:] = saved_lock

        quality = 5       # SPP
        rr = x_spp[:3].copy()
        ns = int(sol.ns)
        pos_sd = np.array([10.0, 10.0, 10.0])

        if self._tc_mode in ("rtk", "rtd"):
            if obsb is None:
                return
            try:
                from src.core.gnss.rtklib.rtkpos import relpos
                from src.core.gnss.rtklib.rtkcmn import Sol, SOLQ_NONE

                nav.x[0:6] = sol.rr[0:6]
                nav.x[6:9] = 1e-6
                relative_sol = Sol()
                relative_sol.t = obsr.t
                relpos(nav, obsr, obsb, relative_sol)
                if relative_sol.stat == SOLQ_NONE:
                    return
                rr = relative_sol.rr[:3].copy()
                ns = relative_sol.ns if relative_sol.ns > 0 else ns
                if self._tc_mode == "rtd":
                    quality = 4
                    pos_sd = np.array([1.0, 1.0, 1.0])
                else:
                    quality = relative_sol.stat
                    pos_sd = {
                        1: np.array([0.1, 0.1, 0.1]),
                        2: np.array([0.3, 0.3, 0.3]),
                        4: np.array([1.0, 1.0, 1.0]),
                    }.get(quality, pos_sd)
            except Exception as e:
                logger.debug(f"GNSS-only relative positioning 异常: {e}")
                return
            finally:
                nav.x[:] = saved_x
                nav.P[:] = saved_P
                if saved_fix is not None:
                    nav.fix[:] = saved_fix
                if saved_lock is not None:
                    nav.lock[:] = saved_lock

        self.writer.write_gnss_only(t_gnss, rr, quality, ns, pos_sd)
        if self.stat_writer is not None:
            self.stat_writer.write_gnss_only(t_gnss, rr, quality, ns, pos_sd)
        self._output_count += 1
