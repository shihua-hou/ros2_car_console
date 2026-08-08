#!/usr/bin/env python3
"""clean_map: 用实时激光把地图里"已经挪走的障碍"擦掉。

【为什么需要这个工具】
nav2 的全局代价地图里, static_layer 把地图栅格直接写进主图, 而 obstacle_layer 的
清除射线**只作用于它自己那一层, 碰不到 static_layer**。所以建图时录进去的东西
即使实际已经搬走, 在全局规划眼里也永远还在。后果(2026-08-06 实测):

    车按局部图判断"这儿是空的"开进去 -> 一进到旧障碍的位置, 全局图认为车正站在
    障碍里 -> NavFn 起点非法 -> "GridBased: failed to create plan" 连续失败 ->
    恢复行为 spin/backup 也被判定 Collision Ahead -> Goal failed

`clear_entirely_global_costmap` 服务对这种情况是无效的(它只清 obstacle_layer)。
根治只有两条路: 重新建图, 或者像本工具这样按实时观测把 pgm 改掉。

【判据】一个"地图说是障碍"的格子被认定为已挪走, 需要同时满足:
  1. 被激光射线**穿过** >= min_pass 次(射线终点之前的格子, 留 0.15m 余量)
  2. **从来没有**成为过任何一帧的射线终点(哪怕一次都不行)
  3. 8邻域内也没有任何"终点"格(防定位抖动误伤墙面边缘)
条件2/3 是主要的安全阀: 真墙一定会被打中, 一旦打中就永久排除。

【用法】
  # 先干跑, 只报告不写盘:
  ros2 run wheeltec_fastlio clean_map.py --duration 60
  # 确认无误后写盘(自动备份原图, 并调用 map_server 热加载, 无需重启导航):
  ros2 run wheeltec_fastlio clean_map.py --duration 180 --write

运行期间**用遥控把车开过那些障碍已经搬走的区域**, 多角度看几眼。只在观测到的
地方生效, 没看过的地方不动。

【前提】定位必须是准的。运行时会打印 auto_relocalize 的健康度, 低于 0.5 建议先
重定位再跑, 否则可能擦错格子。
"""
import argparse
import math
import os
import shutil
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       qos_profile_sensor_data)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

OCC_THRESH = 65        # OccupancyGrid 里 >=65 视为障碍
END_MARGIN = 0.15      # 射线终点前留的余量(米), 防止把障碍本身当成"穿过"


class Cleaner(Node):
    def __init__(self, args):
        super().__init__('clean_map')
        self.args = args
        self.map = None
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        self.create_subscription(LaserScan, args.scan_topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.passes = None      # 每格被射线穿过的次数
        self.hits = None        # 每格作为射线终点的次数
        self.frames = 0
        self.last = 0.0
        self.poses = []

    def _on_map(self, m):
        if self.map is None:
            self.map = m
            self.passes = np.zeros((m.info.height, m.info.width), np.int32)
            self.hits = np.zeros((m.info.height, m.info.width), np.int32)
            self.get_logger().info(
                f'地图 {m.info.width}x{m.info.height} @{m.info.resolution:.3f}m, '
                f'障碍格 {int((np.array(m.data) >= OCC_THRESH).sum())}')

    def _on_scan(self, s):
        if self.map is None:
            return
        now = time.time()
        if now - self.last < 1.0 / self.args.rate:
            return
        try:
            tf = self.buf.lookup_transform('map', s.header.frame_id,
                                           rclpy.time.Time())
        except Exception:
            return
        self.last = now
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        info = self.map.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        H, W = info.height, info.width
        occ = self.occ_mask
        r = np.array(s.ranges)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > s.range_min) & (r < s.range_max)
        for rr, aa in zip(r[ok], a[ok]):
            th = yaw + aa
            c, sn = math.cos(th), math.sin(th)
            ei = int((t.x + rr * c - ox) / res)
            ej = int((t.y + rr * sn - oy) / res)
            if 0 <= ei < W and 0 <= ej < H:
                self.hits[ej, ei] += 1
            n = int(max(0.0, rr - END_MARGIN) / res)
            for k in range(2, n):
                pi = int((t.x + k * res * c - ox) / res)
                pj = int((t.y + k * res * sn - oy) / res)
                if not (0 <= pi < W and 0 <= pj < H):
                    break
                if occ[pj, pi]:
                    self.passes[pj, pi] += 1
        self.frames += 1
        self.poses.append((t.x, t.y))
        if self.frames % 20 == 0:
            self.get_logger().info(
                f'已累计 {self.frames} 帧, 车走过 {self._travelled():.1f}m, '
                f'当前候选 {len(self.stale()[0])} 格')

    def _travelled(self):
        d = 0.0
        for i in range(1, len(self.poses)):
            d += math.hypot(self.poses[i][0] - self.poses[i - 1][0],
                            self.poses[i][1] - self.poses[i - 1][1])
        return d

    @property
    def occ_mask(self):
        g = np.array(self.map.data, dtype=np.int16).reshape(
            self.map.info.height, self.map.info.width)
        if not hasattr(self, '_occ'):
            self._occ = g >= OCC_THRESH
        return self._occ

    def stale(self):
        """返回 (行索引, 列索引): 判定为"已挪走"的地图障碍格"""
        hit_any = self.hits > 0
        # 8邻域膨胀: 打中过的格子周围一圈也不许擦(防定位抖动误伤墙面)
        prot = hit_any.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                prot |= np.roll(np.roll(hit_any, dy, 0), dx, 1)
        m = self.occ_mask & (self.passes >= self.args.min_pass) & (~prot)
        return np.nonzero(m)


def write_map(node, rows, cols, args):
    info = node.map.info
    src_pgm = args.map_pgm
    if not os.path.exists(src_pgm):
        node.get_logger().error(f'找不到地图文件 {src_pgm}')
        return False
    stamp = time.strftime('%Y%m%d_%H%M%S')
    bak = f'{src_pgm}.bak_{stamp}'
    shutil.copy2(src_pgm, bak)
    node.get_logger().info(f'原图已备份到 {bak}')
    with open(src_pgm, 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = map(int, f.readline().split())
        f.readline()
        data = bytearray(f.read(w * h))
    if (w, h) != (info.width, info.height):
        node.get_logger().error(
            f'pgm 尺寸 {w}x{h} 与 /map {info.width}x{info.height} 不一致, 拒绝写入')
        return False
    n = 0
    for j, i in zip(rows, cols):
        # pgm 第一行对应地图最上方(y最大), 与栅格 y 轴相反
        idx = (h - 1 - j) * w + i
        if data[idx] != 0xFE:
            data[idx] = 0xFE          # 置为空闲(白)
            n += 1
    with open(src_pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(bytes(data))
    node.get_logger().info(f'已擦除 {n} 格并写回 {src_pgm}')
    return True


def reload_map(node, args):
    from nav2_msgs.srv import LoadMap
    cli = node.create_client(LoadMap, '/map_server/load_map')
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn('map_server/load_map 不可用, 请手动重启导航使新图生效')
        return
    req = LoadMap.Request()
    req.map_url = args.map_yaml
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    r = fut.result()
    if r and r.result == 0:
        node.get_logger().info('map_server 已热加载新图 (无需重启导航)')
    else:
        node.get_logger().warn(f'热加载失败(result={getattr(r, "result", "?")}), 请重启导航')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0, help='采集时长(秒)')
    ap.add_argument('--rate', type=float, default=5.0, help='每秒处理多少帧scan')
    ap.add_argument('--min-pass', type=int, default=15,
                    help='被射线穿过多少次才算数')
    ap.add_argument('--scan-topic', default='/scan',
                    help='用哪路scan(必须与建图的高度带一致, 默认/scan)')
    ap.add_argument('--write', action='store_true', help='真的写盘(否则只报告)')
    ap.add_argument('--map-pgm',
                    default='/home/cat/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC3D.pgm')
    ap.add_argument('--map-yaml',
                    default='/home/cat/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC3D.yaml')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Cleaner(args)
    node.get_logger().info(
        f'开始采集 {args.duration:.0f} 秒 —— 请用遥控把车开过那些"障碍已搬走"的区域')
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.map is None:
        node.get_logger().error('没收到 /map, 导航是否在运行?')
        rclpy.shutdown()
        return
    rows, cols = node.stale()
    res = node.map.info.resolution
    node.get_logger().info(
        f'\n采集 {node.frames} 帧, 车走过 {node._travelled():.1f}m\n'
        f'判定为"已挪走"的地图障碍格: {len(rows)} 个 = {len(rows) * res * res:.2f} m²\n'
        f'(判据: 被射线穿过>={args.min_pass}次, 且从未被打中, 且8邻域内无命中)')
    if len(rows) == 0:
        node.get_logger().info('没有可擦除的格子。可能是车没开到那些区域, 或采集时间太短。')
    elif args.write:
        if write_map(node, rows, cols, args):
            reload_map(node, args)
    else:
        node.get_logger().info('这是干跑(未写盘)。确认无误后加 --write 再跑一次。')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
