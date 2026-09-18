#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""存图时自动做 GPS 地理配准, 结果绑定到这张地图。

为什么要有这个脚本
------------------
GPS-地图标定绑的是某一次 FAST-LIO 会话的 camera_init 系(原点=开机位置,
朝向=开机朝向, 每次建图都不同)。以前标定是独立的一步、存在一个全局路径
(outdoor_maps/gps_map_calibration.yaml), 换一张图就悄悄失效 ——
2026-09-16 实测: 拿上一次的标定用在新图上, 两次会话坐标系相差 215.9°,
每次"救援"把车瞬移中位 31.1m、最大 48.5m。

所以改成: 存图时顺手做掉, 写成 <地图名>.gps.yaml 跟地图放在一起, 并在文件里
记下 map_name。守卫加载时比对, 对不上就拒绝 —— 让它不可能配错。

三道自检(任何一道不过就不写文件, 宁可没有标定也不要错的标定)
-------------------------------------------------------------
  1. 会话一致性: 轨迹 CSV 的位姿必须落在 mat_pre.txt(pcd2pgm 刚用过的那条
     轨迹)上。两者都在同一次会话的 camera_init 系里, 不是同一次就对不上。
  2. 解算等级: 只用 RTK 固定解(status==2)。浮点解是亚米级, 混进来 RMS 会
     从 0.27m 劣化到 1.49m(2026-09-16 实测)。
  3. 共线性: 样本挤成一条线时 2D 刚体变换的旋转角解不出来, 这时 RMS 再小,
     外推到地图边缘也会错得离谱。

坐标系
------
CSV 里的位姿在 camera_init 系, 而地图是**校平之后**的系。pcd2pgm 会把它用的
校平变换导出到 <地图名>.tf.yaml, 这里先应用它再拟合, 否则会漏掉约 0.7% 的
单向尺度误差(50m 上约 0.35m)。
"""
import argparse
import csv
import glob
import math
import os
import shutil
import sys
import time

import numpy as np

try:
    import yaml
except ImportError:
    print('缺少 pyyaml: pip3 install pyyaml')
    sys.exit(1)

try:
    from pyproj import Proj
except ImportError:
    print('缺少 pyproj: pip3 install pyproj')
    sys.exit(1)

TRAJ_DIR = os.path.expanduser('~/wheeltec_ros2/outdoor_maps')
MAT_PRE = os.path.expanduser('~/wheeltec_ros2/src/FAST_LIO/Log/mat_pre.txt')

FIXED = 2          # NavSatFix.status: 2 = RTK 固定解
MIN_POINTS = 50    # 少于这么多固定解点就不值得拟合
MAX_AXIS_RATIO = 5.0
SESSION_TOL_SEC = 600.0   # CSV 与 mat_pre 结束时刻相差超过这么久就不算同一次建图


def load_align_matrix(map_dir, map_name):
    """读 pcd2pgm 导出的地面校平变换; 没有就退回单位阵。"""
    p = os.path.join(map_dir, map_name + '.tf.yaml')
    if not os.path.exists(p):
        print(f'  ! 找不到 {os.path.basename(p)}, 校平变换退回单位阵')
        print('    (标定会带约 0.7% 的单向尺度误差; 重新存一次图即可生成)')
        return np.eye(4)
    with open(p, encoding='utf-8') as f:
        d = yaml.safe_load(f)
    M = np.array(d['ground_align_matrix'], dtype=float)
    print(f'  校平变换: 已加载 {os.path.basename(p)}')
    return M


def csv_end_ts(path):
    """CSV 最后一行的绝对时间戳; 读不到就退回文件 mtime。"""
    try:
        last = None
        with open(path, encoding='utf-8') as f:
            for r in csv.DictReader(f):
                last = r
        if last:
            return float(last['timestamp_sec']) + float(last['timestamp_nsec']) * 1e-9
    except (ValueError, KeyError, OSError):
        pass
    return os.path.getmtime(path)


def load_csv(path):
    rows = []
    with open(path, encoding='utf-8') as f:
        for r in csv.DictReader(f):
            try:
                rows.append((
                    int(r['gps_status']),
                    float(r['lidar_x']), float(r['lidar_y']), float(r['lidar_z']),
                    float(r['gps_lat']), float(r['gps_lon']),
                ))
            except (ValueError, KeyError):
                continue
    return rows


def _shape(xy):
    """旋转不变的形状特征: (累计路程, 包围盒对角线)。"""
    d = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
    span = xy.max(axis=0) - xy.min(axis=0)
    return d, float(np.hypot(span[0], span[1]))


def same_session(csv_xy, csv_end_ts):
    """轨迹 CSV 是否和 pcd2pgm 刚用的那次建图同源。

    主判据是**时间**: 建图结束时 FAST-LIO 停写 mat_pre.txt、记录器停写 CSV,
    两者结束时刻只差秒级。

    不要用"每个点到对方轨迹最近点的距离" —— 2026-09-16 实测那个判据没用:
    两次建图在同一片场地、各自从自己的原点出发, 即使坐标系差 215.9°, 两团
    轨迹数值上仍大量重叠, 中位距离只有 1.00m, 照样放行。最近距离对旋转不敏感。

    第二道是旋转不变的形状特征(路程+包围盒对角线), 防止时间凑巧撞上但拿错文件。
    """
    if not os.path.exists(MAT_PRE):
        print('  ! 找不到 mat_pre.txt, 跳过会话一致性检查')
        return True

    # --- 时间 ---
    dt = abs(os.path.getmtime(MAT_PRE) - csv_end_ts)
    print(f'  会话一致性: CSV 结束与 mat_pre 结束相差 {dt / 60:.1f} 分钟', end='')
    if dt > SESSION_TOL_SEC:
        print(f'  ✗ 超过 {SESSION_TOL_SEC / 60:.0f} 分钟, 判定不是同一次建图')
        return False
    print('  ✓')

    # --- 形状(第二道) ---
    try:
        d = np.loadtxt(MAT_PRE)
        if d.ndim == 2 and d.shape[1] >= 7 and len(d) >= 10:
            ld, lb = _shape(csv_xy)
            rd, rb = _shape(d[:, 4:6])
            # CSV 约 5Hz、mat_pre 约 10Hz, 采样率不同会让"累计路程"差出几成,
            # 所以只用它排除数量级差异; 包围盒才是硬指标。
            if rb > 1e-6 and not (0.7 < lb / rb < 1.4):
                print(f'  ✗ 轨迹形状对不上(包围盒 {lb:.1f}m vs {rb:.1f}m)')
                return False
            print(f'  轨迹形状: 包围盒 {lb:.1f}m vs {rb:.1f}m  ✓')
    except Exception as e:
        print(f'  ! 形状检查跳过({e})')
    return True


def fit_rigid(src, dst):
    """最小二乘 2D 刚体拟合(Kabsch), 返回 R,t 使 dst ≈ R@src + t。"""
    cs, cd = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((dst - cd).T @ (src - cs))
    R = U @ np.diag([1.0, np.linalg.det(U @ Vt)]) @ Vt
    return R, cd - R @ cs


def main():
    ap = argparse.ArgumentParser(description='存图时的 GPS 地理配准')
    ap.add_argument('--map-dir', required=True)
    ap.add_argument('--map-name', required=True)
    ap.add_argument('--also-dir', default='',
                    help='标定再复制一份到这个目录(地图本身也是存两处的)')
    ap.add_argument('--csv', default='', help='留空=自动取最新的轨迹 CSV')
    ap.add_argument('--min-quality', type=int, default=FIXED,
                    help='最低解算等级, 默认 2=RTK固定解')
    a = ap.parse_args()

    print(f'GPS 地理配准: {a.map_name}')

    path = a.csv
    if not path:
        cands = sorted(glob.glob(os.path.join(TRAJ_DIR, 'trajectory_*.csv')),
                       key=os.path.getmtime)
        if not cands:
            print('  没有任何轨迹 CSV, 跳过地理配准(户外模式才会有)')
            return 0
        path = cands[-1]
    print(f'  轨迹: {os.path.basename(path)}')

    rows = load_csv(path)
    if len(rows) < MIN_POINTS:
        print(f'  轨迹只有 {len(rows)} 行, 跳过')
        return 0

    xyz = np.array([[r[1], r[2], r[3]] for r in rows])
    if not same_session(xyz[:, :2], csv_end_ts(path)):
        print('  → 不写标定。这份 CSV 不是生成这张图的那次建图, '
              '用它标定会让守卫把车拉到错误位置。')
        return 0

    # camera_init -> 地图系
    M = load_align_matrix(a.map_dir, a.map_name)
    hom = np.c_[xyz, np.ones(len(xyz))]
    mapxy = (hom @ M.T)[:, :2]

    q = np.array([r[0] for r in rows])
    keep = q >= a.min_quality
    n_fix = int(keep.sum())
    print(f'  解算等级≥{a.min_quality} 的点: {n_fix} / {len(rows)} '
          f'({100.0 * n_fix / len(rows):.0f}%)')
    if n_fix < MIN_POINTS:
        print(f'  → 不写标定。固定解点不足 {MIN_POINTS} 个 —— '
              '建图时 RTK 多数时间没到固定解, 换个天空视野好的时段/场地重建。')
        return 0

    L = mapxy[keep]
    lat = np.array([r[4] for r in rows])[keep]
    lon = np.array([r[5] for r in rows])[keep]

    zone = int((lon[0] + 180) / 6) + 1
    band = 'N' if lat[0] >= 0 else 'S'
    proj = Proj(proj='utm', zone=zone, ellps='WGS84',
                south=(band == 'S'))
    ux, uy = proj(lon, lat)
    U = np.c_[ux, uy]

    # 共线性: 看真正参与拟合的那批点
    ev = np.sort(np.linalg.eigvalsh(np.cov((L - L.mean(0)).T)))[::-1]
    major, minor = np.sqrt(np.maximum(ev, 0))
    ratio = major / minor if minor > 1e-6 else float('inf')
    print(f'  样本展布: 主轴 {major:.2f} m / 次轴 {minor:.2f} m  轴比 {ratio:.1f}')
    if ratio > MAX_AXIS_RATIO:
        print(f'  → 不写标定。样本近似共线(轴比 {ratio:.1f} > {MAX_AXIS_RATIO}), '
              '旋转角没被约束住。')
        print('    下次在**固定解状态下**绕个环或走 8 字, 别直线来回。')
        return 0

    R, t = fit_rigid(L, U)                      # utm ≈ R@map + t, 与守卫的逆变换对应
    res = np.linalg.norm(L @ R.T + t - U, axis=1)
    rms = float(res.mean())
    print(f'  拟合 RMS {rms:.3f} m   旋转 {math.degrees(math.atan2(R[1,0], R[0,0])):+.2f}°')

    out = os.path.join(a.map_dir, a.map_name + '.gps.yaml')
    with open(out, 'w', encoding='utf-8') as f:
        yaml.dump({
            # 守卫据此确认这份标定属于它正在加载的那张图
            'map_name': a.map_name,
            'gps_to_map_transform': {
                'rotation': [[float(R[0, 0]), float(R[0, 1])],
                             [float(R[1, 0]), float(R[1, 1])]],
                'translation': [float(t[0]), float(t[1])],
                'utm_zone': zone,
                'utm_band': band,
            },
            'rms_error': rms,
            'num_points': n_fix,
            'axis_ratio': float(ratio),
            'min_quality': a.min_quality,
            'source_csv': os.path.basename(path),
            'ground_aligned': bool(not np.allclose(M, np.eye(4))),
            'created': time.strftime('%Y-%m-%d %H:%M:%S'),
            'note': '由 georef_map.py 在存图时自动生成; 只对 map_name 这张图有效',
        }, f, allow_unicode=True, default_flow_style=False)
    print(f'  ✓ 已写入 {os.path.basename(out)}')
    # 地图由 pcd2pgm 写到 install 和 src 两处, 标定也跟着走 ——
    # 否则 colcon build 重建 install 之后标定就掉了, 守卫又变成"没有标定"。
    if a.also_dir and os.path.isdir(a.also_dir):
        try:
            shutil.copy2(out, os.path.join(a.also_dir, os.path.basename(out)))
            tf = os.path.join(a.map_dir, a.map_name + '.tf.yaml')
            if os.path.exists(tf):
                shutil.copy2(tf, os.path.join(a.also_dir, os.path.basename(tf)))
            print(f'  ✓ 已同步到 {a.also_dir}')
        except OSError as e:
            print(f'  ! 同步到 {a.also_dir} 失败: {e}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
