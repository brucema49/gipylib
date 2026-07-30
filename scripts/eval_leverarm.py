"""从 .rslt 文件读取杆臂参数, 计算平均值并打印。

只读取 Qins=3 的历元 (GNSS 量测更新后, 杆臂已更新)。
计算两个统计量:
  1. 全部 Qins=3 历元的杆臂平均值
  2. 最新 100 个 Qins=3 历元的杆臂平均值

.rslt 字段索引 (列号, 0-based):
  0:week 1:sow 2:lat 3:lon 4:h 5:Q 6:Qins 7:ns
  8-13:sdn sde sdu sdne sdeu sdun  14:age 15:ratio
  16-18:vx vy vz  19-24:sdvx sdvy sdvz sdvxy sdvyz sdvzx
  25-27:roll pitch yaw  28-30:sdroll sdpitch sdyaw
  31-33:lever_x lever_y lever_z  34-36:sdlx sdly sdlz

用法:
    python3 scripts/eval_leverarm.py [rslt_path]

默认 rslt_path = data/output/RTKLC.rslt
"""
import sys
import numpy as np

# 字段索引
QINS = 6
LX, LY, LZ = 31, 32, 33
SDLX, SDLY, SDLZ = 34, 35, 36


def load_leverarm(path):
    """从 .rslt 读取 Qins=3 历元的杆臂参数。

    Returns:
        levers: (N, 3) 杆臂 [lx, ly, lz]
        sdls: (N, 3) 杆臂 sd
        sows: (N,) 时间戳
    """
    levers = []
    sdls = []
    sows = []
    with open(path) as f:
        for line in f:
            if line.startswith("%") or not line.strip():
                continue
            p = line.split()
            if len(p) <= SDLZ:
                continue
            qins = int(p[QINS])
            if qins != 3:
                continue
            levers.append([float(p[LX]), float(p[LY]), float(p[LZ])])
            sdls.append([float(p[SDLX]), float(p[SDLY]), float(p[SDLZ])])
            sows.append(float(p[1]))
    return (np.array(levers), np.array(sdls), np.array(sows))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "data/output/RTKLC.rslt"
    levers, sdls, sows = load_leverarm(path)

    print("=" * 60)
    print("杆臂参数评估 (仅 Qins=3 历元)")
    print("=" * 60)
    print("文件: %s" % path)
    print("Qins=3 历元数: %d" % len(levers))

    if len(levers) == 0:
        print("ERROR: 无 Qins=3 历元, 无法计算杆臂参数")
        sys.exit(1)

    # 1. 全部 Qins=3 历元的平均值
    mean_all = levers.mean(axis=0)
    std_all = levers.std(axis=0)
    print("\n--- 1. 全部 %d 个 Qins=3 历元的杆臂平均 ---" % len(levers))
    print("  lever_x = %+.5f m  (std=%.5f)" % (mean_all[0], std_all[0]))
    print("  lever_y = %+.5f m  (std=%.5f)" % (mean_all[1], std_all[1]))
    print("  lever_z = %+.5f m  (std=%.5f)" % (mean_all[2], std_all[2]))
    print("  |lever| = %.5f m" % float(np.linalg.norm(mean_all)))

    # 2. 最新 100 个 Qins=3 历元的平均值
    n_last = min(100, len(levers))
    last_levers = levers[-n_last:]
    mean_last = last_levers.mean(axis=0)
    std_last = last_levers.std(axis=0)
    print("\n--- 2. 最新 %d 个 Qins=3 历元的杆臂平均 ---" % n_last)
    print("  lever_x = %+.5f m  (std=%.5f)" % (mean_last[0], std_last[0]))
    print("  lever_y = %+.5f m  (std=%.5f)" % (mean_last[1], std_last[1]))
    print("  lever_z = %+.5f m  (std=%.5f)" % (mean_last[2], std_last[2]))
    print("  |lever| = %.5f m" % float(np.linalg.norm(mean_last)))

    # 首末对比 (收敛趋势)
    print("\n--- 收敛趋势 ---")
    print("  首个 Qins=3: lever = [%.5f, %.5f, %.5f] at sow=%.3f" %
          (levers[0, 0], levers[0, 1], levers[0, 2], sows[0]))
    print("  末个 Qins=3: lever = [%.5f, %.5f, %.5f] at sow=%.3f" %
          (levers[-1, 0], levers[-1, 1], levers[-1, 2], sows[-1]))

    # 杆臂 sd (不确定性)
    mean_sdl = sdls[-n_last:].mean(axis=0)
    print("\n--- 最新 %d 历元杆臂 sd (不确定性) ---" % n_last)
    print("  sdlx = %.5f m" % mean_sdl[0])
    print("  sdly = %.5f m" % mean_sdl[1])
    print("  sdlz = %.5f m" % mean_sdl[2])

    print("=" * 60)


if __name__ == "__main__":
    main()
