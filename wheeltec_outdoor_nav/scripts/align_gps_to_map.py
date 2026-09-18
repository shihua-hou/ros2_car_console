#!/usr/bin/env python3
# coding=utf-8
"""
GPS-地图对齐脚本：离线计算GPS轨迹与雷达地图的刚体变换

输入：
  trajectory_XXXXXX.csv - 建图时记录的双轨迹

输出：
  gps_map_calibration.yaml - 对齐参数（旋转+平移）

算法：
  1. 提取所有RTK固定解的点对
  2. 使用 Horn's method 或 ICP 计算刚体变换
  3. 计算RMS误差评估对齐质量

使用方法：
  cd ~/wheeltec_ros2/outdoor_maps/
  ros2 run wheeltec_outdoor_nav align_gps_to_map.py
  # 或指定轨迹文件
  ros2 run wheeltec_outdoor_nav align_gps_to_map.py --input trajectory_20260907_120000.csv
"""
import os
import sys
import csv
import argparse
import numpy as np
import yaml

try:
    from pyproj import Proj
    PYPROJ_AVAILABLE = True
except ImportError:
    PYPROJ_AVAILABLE = False
    print('错误: pyproj库未安装')
    print('请运行: pip3 install pyproj')
    sys.exit(1)


def load_trajectory(csv_file, min_quality=0):
    """加载轨迹CSV文件。

    min_quality: 加载阶段的下限, 默认 0 = 全量加载, 这样下面的"质量分布统计"
    统计的才是原始数据。真正决定用哪些点拟合的过滤在 align_trajectories 里,
    那里默认只用固定解。
    """
    print(f'读取轨迹文件: {csv_file} (最低解算等级 {min_quality})')

    lidar_points = []
    gps_points = []
    gps_quality = []

    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # 雷达里程计位置
            lidar_x = float(row['lidar_x'])
            lidar_y = float(row['lidar_y'])

            # GPS数据
            gps_lat = float(row['gps_lat'])
            gps_lon = float(row['gps_lon'])
            gps_status = int(row['gps_status'])

            # 只使用达到要求解算等级的点(默认仅 RTK 固定解)
            if (gps_status >= min_quality
                    and abs(gps_lat) > 0.001 and abs(gps_lon) > 0.001):
                lidar_points.append([lidar_x, lidar_y])
                gps_points.append([gps_lat, gps_lon])
                gps_quality.append(gps_status)

    lidar_points = np.array(lidar_points)
    gps_points = np.array(gps_points)
    gps_quality = np.array(gps_quality)

    print(f'  总点数: {len(lidar_points)}')
    print(f'  GPS有效点: {len(gps_points)}')

    if len(gps_points) == 0:
        print('错误: 没有有效的GPS数据！')
        print('  可能原因：')
        print('    1. 建图时GPS模块未连接')
        print('    2. GPS从未获得固定解')
        print('    3. 轨迹记录时GPS话题名称不匹配')
        sys.exit(1)

    # 统计GPS质量分布
    for quality in [0, 1, 2]:
        count = np.sum(gps_quality == quality)
        quality_name = ['单点定位', '差分定位', 'RTK固定解'][quality]
        print(f'    {quality_name}: {count} 点')

    return lidar_points, gps_points, gps_quality


def gps_to_utm(gps_points, utm_zone=None):
    """GPS经纬度 → UTM坐标"""
    # 自动检测UTM区域（使用第一个点）
    if utm_zone is None:
        lon = gps_points[0, 1]
        utm_zone = int((lon + 180) / 6) + 1

    # 检测南北半球
    lat = gps_points[0, 0]
    hemisphere = 'north' if lat >= 0 else 'south'
    utm_band = 'N' if lat >= 0 else 'S'

    proj = Proj(proj='utm', zone=utm_zone, ellps='WGS84',
                datum='WGS84', units='m', south=(hemisphere == 'south'))

    utm_points = []
    for lat, lon in gps_points:
        x, y = proj(lon, lat)
        utm_points.append([x, y])

    utm_points = np.array(utm_points)

    print(f'\nUTM坐标转换完成:')
    print(f'  UTM Zone: {utm_zone}{utm_band}')
    print(f'  范围: X=[{utm_points[:, 0].min():.1f}, {utm_points[:, 0].max():.1f}]')
    print(f'       Y=[{utm_points[:, 1].min():.1f}, {utm_points[:, 1].max():.1f}]')

    return utm_points, utm_zone, utm_band


def compute_rigid_transform(source, target):
    """
    计算刚体变换（Horn's method）
    使得 target ≈ R @ source + t

    返回: R (2x2旋转矩阵), t (2x1平移向量)
    """
    # 中心化
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)

    source_centered = source - source_center
    target_centered = target - target_center

    # 计算协方差矩阵
    H = source_centered.T @ target_centered

    # SVD分解
    U, S, Vt = np.linalg.svd(H)

    # 计算旋转矩阵
    R = Vt.T @ U.T

    # 确保旋转矩阵的行列式为正（右手系）
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # 计算平移
    t = target_center - R @ source_center

    return R, t


def compute_rms_error(source, target, R, t):
    """计算RMS误差"""
    transformed = (R @ source.T).T + t
    errors = np.linalg.norm(transformed - target, axis=1)
    rms = np.sqrt(np.mean(errors ** 2))
    return rms, errors


def align_trajectories(lidar_points, gps_points, gps_quality,
                      min_quality=2, max_outlier_ratio=0.1):
    """
    对齐雷达和GPS轨迹

    参数:
      min_quality: 最低GPS质量要求（0=单点, 1=差分/浮点, 2=RTK固定解）
                   默认 2。1 会把亚米级的浮点解收进来 —— 2026-09-16 实测同一份
                   轨迹: 全部样本 RMS 1.487m, 只用固定解 0.266m。
      max_outlier_ratio: 最大离群点比例（用于剔除异常GPS跳变）
    """
    # 只使用高质量GPS点
    valid_idx = np.where(gps_quality >= min_quality)[0]

    if len(valid_idx) < 10:
        # 这里原本会静默降到 min_quality=0 继续跑。那等于拿米级误差的单点解去
        # 标定厘米级的 RTK —— 结果看着有值, 实则是废的, 而且只有一行 warn,
        # 很容易被当成正常输出忽略过去。宁可明确失败。
        print(f'\n错误: 质量≥{min_quality} 的GPS点只有 {len(valid_idx)} 个(需要至少10个)。')
        print('  建图期间 RTK 多数时间没到这个等级。可选:')
        print('    - 重新建图, 保证 RTK 保持固定解(看界面 GPS 状态);')
        print('    - 或显式放宽: --min-quality 1 (浮点解, 亚米级, 精度会明显变差)。')
        sys.exit(1)

    lidar_valid = lidar_points[valid_idx]
    gps_valid = gps_points[valid_idx]

    print(f'\n使用 {len(valid_idx)} 个GPS质量≥{min_quality}的点进行对齐')

    # 共线性检查: 样本挤成一条线时, 2D 刚体变换的旋转角几乎解不出来。
    # 这种情况下 RMS 可能很漂亮, 但外推到地图边缘会错得离谱 ——
    # RMS 小不等于标定可用, 必须同时看展布。
    _c = lidar_valid - lidar_valid.mean(axis=0)
    _ev = np.sort(np.linalg.eigvalsh(np.cov(_c.T)))[::-1]
    _major, _minor = np.sqrt(np.maximum(_ev, 0))
    _ratio = _major / _minor if _minor > 1e-6 else float('inf')
    print(f'参与拟合的点展布: 主轴 {_major:.2f} m / 次轴 {_minor:.2f} m  '
          f'轴比 {_ratio:.1f}')
    if _ratio > 5:
        print('⚠ 样本近似共线, 旋转角没有被约束住 —— 这份标定不可靠, 别拿去用。')
        print('  重新建图, 并保证固定解期间车走出二维展布(绕环, 别直线来回)。')

    # 第一次对齐
    R, t = compute_rigid_transform(lidar_valid, gps_valid)
    rms, errors = compute_rms_error(lidar_valid, gps_valid, R, t)

    print(f'  初始RMS误差: {rms:.3f} m')

    # 剔除离群点（GPS跳变）
    error_threshold = np.percentile(errors, 95)  # 保留95%的点
    inlier_idx = np.where(errors < error_threshold)[0]

    if len(inlier_idx) < len(valid_idx) * (1 - max_outlier_ratio):
        print(f'  检测到过多离群点，使用全部点')
    else:
        lidar_valid = lidar_valid[inlier_idx]
        gps_valid = gps_valid[inlier_idx]
        print(f'  剔除 {len(valid_idx) - len(inlier_idx)} 个离群点')

        # 第二次对齐（使用内点）
        R, t = compute_rigid_transform(lidar_valid, gps_valid)
        rms, errors = compute_rms_error(lidar_valid, gps_valid, R, t)

        print(f'  优化后RMS误差: {rms:.3f} m')

    # 计算旋转角度（用于检查）
    angle_rad = np.arctan2(R[1, 0], R[0, 0])
    angle_deg = np.degrees(angle_rad)

    print(f'\n对齐结果:')
    print(f'  旋转角度: {angle_deg:.2f}°')
    print(f'  平移: ({t[0]:.2f}, {t[1]:.2f}) m')
    print(f'  最终RMS: {rms:.3f} m')
    print(f'  使用点数: {len(lidar_valid)}')

    # 警告检查
    if rms > 2.0:
        print(f'\n警告: RMS误差较大（{rms:.2f}m > 2.0m）')
        print('  可能原因：')
        print('    1. GPS信号质量差（遮挡/多路径）')
        print('    2. 建图时FAST-LIO漂移过大')
        print('    3. 建图轨迹太短，对齐不准确')
        print('  建议：重新建图，走更长的闭环轨迹')

    if abs(angle_deg) > 10:
        print(f'\n警告: 旋转角度较大（{angle_deg:.1f}° > 10°）')
        print('  这是正常的，可能是地图朝向与正北不一致')

    return R, t, rms, len(lidar_valid)


def save_calibration(R, t, utm_zone, utm_band, rms, num_points, output_file):
    """保存对齐参数"""
    calibration = {
        'gps_to_map_transform': {
            'rotation': R.tolist(),
            'translation': t.tolist(),
            'utm_zone': int(utm_zone),
            'utm_band': utm_band
        },
        'rms_error': float(rms),
        'num_points': int(num_points),
        'note': 'GPS UTM → 地图坐标的刚体变换。在guard节点中，用逆变换将GPS转到地图坐标'
    }

    with open(output_file, 'w') as f:
        yaml.dump(calibration, f, default_flow_style=False)

    print(f'\n对齐参数已保存: {output_file}')
    print(f'  现在可以启动 outdoor_navigation.launch.py 使用GPS守卫功能')


def main():
    parser = argparse.ArgumentParser(description='GPS-地图对齐工具')
    parser.add_argument('--input', '-i',
                       help='输入轨迹CSV文件（默认自动查找最新的）')
    parser.add_argument('--output', '-o',
                       default='gps_map_calibration.yaml',
                       help='输出对齐参数文件')
    parser.add_argument('--min-quality', type=int, default=2,
                       help='最低GPS质量 (0=单点, 1=差分/浮点, 2=RTK固定解). '
                            '默认2; 放宽到1精度会明显变差')
    parser.add_argument('--utm-zone', type=int,
                       help='指定UTM区域（默认自动检测）')

    args = parser.parse_args()

    # 确定工作目录
    work_dir = os.path.expanduser('~/wheeltec_ros2/outdoor_maps/')
    os.chdir(work_dir)

    # 查找输入文件
    if args.input:
        csv_file = args.input
    else:
        # 自动查找最新的trajectory文件
        csv_files = [f for f in os.listdir('.') if f.startswith('trajectory_') and f.endswith('.csv')]
        if not csv_files:
            print('错误: 未找到轨迹文件')
            print(f'  请先运行 outdoor_mapping.launch.py 建图')
            print(f'  或使用 --input 参数指定文件')
            sys.exit(1)

        csv_file = sorted(csv_files)[-1]  # 最新的

    if not os.path.exists(csv_file):
        print(f'错误: 文件不存在: {csv_file}')
        sys.exit(1)

    print('=' * 60)
    print('GPS-地图对齐工具')
    print('=' * 60)

    # 1. 加载轨迹
    lidar_points, gps_points, gps_quality = load_trajectory(csv_file)

    # 2. GPS转UTM
    utm_points, utm_zone, utm_band = gps_to_utm(gps_points, args.utm_zone)

    # 3. 计算对齐
    R, t, rms, num_points = align_trajectories(
        lidar_points, utm_points, gps_quality,
        min_quality=args.min_quality
    )

    # 4. 保存结果
    save_calibration(R, t, utm_zone, utm_band, rms, num_points, args.output)

    print('\n对齐完成！')


if __name__ == '__main__':
    main()
