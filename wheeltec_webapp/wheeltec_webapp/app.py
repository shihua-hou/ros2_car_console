#!/usr/bin/env python3
"""
wheeltec_webapp: 小车 Web 控制台 (轻量级 web 版 rviz)

在小车上运行一个单端口服务(默认8080), 电脑/iPad 浏览器打开 http://小车IP:8080 即可:
  - 虚拟摇杆遥控 (/cmd_vel, 带0.6s安全看门狗)
  - 一键 建图 / 保存地图 / 导航 / 雷达测试 (托管 wheeltec_fastlio 各 launch,
    停止一律 SIGINT 等待优雅退出, 超时才升级 SIGKILL+清理 /dev/shm)
  - 画布实时显示: /map、/scan、/plan、机器人位姿(TF)、建图点云(/cloud_registered)与轨迹
  - 点图下发 Nav2 目标点(/goal_pose)、初始位姿(/initialpose)、手动重定位(/relocalize)
  - 电压/内存/磁盘/CPU温度 巡检 (4GB 内存机器必看)

用法: ros2 run wheeltec_webapp web [--port 8080]
"""
import argparse
import asyncio
import base64
import fcntl
import glob
import http
import io
import json
import math
import os
import pty
import re
import shutil
import signal
import struct
import subprocess
import termios
import threading
import time

import numpy as np
from PIL import Image

# 必须在 rclpy.init 之前设置: webapp 自身走纯UDP传输(见 udp_only.xml 注释),
# 免疫 launch/stop_nav.sh 的 /dev/shm 清理; 子 launch 会剔除此变量继续用SHM。
_PROFILE_XML = os.path.join(os.path.dirname(__file__), 'static', 'udp_only.xml')
if 'FASTRTPS_DEFAULT_PROFILES_FILE' not in os.environ:
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = _PROFILE_XML

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import websockets

HOME = os.path.expanduser('~')
PCD_DIR = os.path.join(HOME, 'wheeltec_ros2/src/FAST_LIO/PCD')
MAP_DIR = os.path.join(HOME, 'wheeltec_ros2/src/wheeltec_robot_nav2/map')
STATIC_DIR = os.path.join(os.path.dirname(__file__), 'static')

# 二进制消息类型 (server->client, 首字节)
B_MAP, B_SCAN, B_CLOUD, B_PLAN, B_TRAJ, B_COSTMAP, B_MAP_EDIT = 1, 2, 3, 4, 5, 6, 7
B_LOCAL_PLAN = 8            # 局部规划路径 (/local_plan, 控制器/MPPI 输出)

# 速度硬上限 (无论前端滑条设多少)
VX_CAP, WZ_CAP = 0.7, 1.5

# 传感器配置档: mid360(默认, MID360s+FAST-LIO) / n10plus(2D激光+slam_toolbox)
# / odin1(视觉+激光SLAM模组, 自带定位)。三者是完全独立模式, 同一时刻只能跑一个。
# 'save_map' 为 None 表示该传感器不走"launch文件"式存图, 走专门的WS消息处理
# (n10plus用map_saver_cli命令行工具, odin1用写命令文件触发SDK, 见handle_msg)。
SENSOR_PROFILES = {
    'mid360': {
        'label': 'MID360s',
        'package': 'wheeltec_fastlio',
        'launches': {'mapping': 'mapping.launch.py', 'save_map': 'save_map.launch.py',
                     'navigation': 'navigation.launch.py', 'lidar_test': 'lidar_test.launch.py'},
    },
    'n10plus': {
        'label': 'N10Plus',
        'package': 'wheeltec_n10plus',
        'launches': {'mapping': 'mapping.launch.py', 'navigation': 'navigation.launch.py'},
    },
    'odin1': {
        'label': 'Odin1',
        'package': 'wheeltec_odin1',
        'launches': {'mapping': 'mapping.launch.py', 'navigation': 'navigation.launch.py'},
    },
}
# MID360/N10Plus 共用同一份2D地图库(标准nav2 occupancy grid, 建成之后就是张
# 普通图, 不区分是哪个传感器建的)。install副本是导航实际加载的位置, src副本
# 是随源码走的备份。Odin1 的 .bin 是专有SDK格式, 单独一个库, 不能互通。
MAP_DIR_INSTALL = os.path.join(
    HOME, 'wheeltec_ros2/install/wheeltec_nav2/share/wheeltec_nav2/map')
ODIN1_MAP_DIR = os.path.join(HOME, 'wheeltec_ros2/src/wheeltec_odin1/map')
N10PLUS_MAP_PATH_SRC = os.path.join(MAP_DIR, 'N10PLUS_MAP')
N10PLUS_MAP_PATH_INSTALL = os.path.join(MAP_DIR_INSTALL, 'N10PLUS_MAP')
ODIN1_COMMAND_FILE = '/tmp/odin_command.txt'
_MAP_NAME_RE = re.compile(r'[A-Za-z0-9_\-一-鿿]{1,64}')
# 各传感器 launch 文件自带的默认地图名。前端不选图时用的就是这些, UI 要显示
# 具体名字而不是"默认地图" —— 否则用户看不出实际加载的到底是哪张。
DEFAULT_MAP_NAME = {'mid360': 'WHEELTEC3D', 'n10plus': 'N10PLUS_MAP',
                    'odin1': 'odin1_map'}


def _safe_map_name(name):
    return bool(name) and bool(_MAP_NAME_RE.fullmatch(name))


# ---------- 多点巡航航点的持久化 ----------
# 按"传感器桶/地图名"存每张图上设过的巡航航点, 下次打开页面选到这张图就能加载
# 回来。放在 HOME 下的独立文件(不放 install, 那里重新编译会被覆盖; 也不跟地图
# 文件同目录, 避免被地图相关的删除/同步逻辑波及)。
ROUTES_FILE = os.path.join(HOME, '.wheeltec', 'webapp_routes.json')


def _route_key(sensor, name):
    """mid360/n10plus 共用同一份2D地图库, 归到 'pgm' 桶; odin1 单独。
    跟前端 mapBucket() 保持一致。"""
    bucket = 'odin1' if sensor == 'odin1' else 'pgm'
    return '%s/%s' % (bucket, name)


def _load_routes():
    try:
        with open(ROUTES_FILE) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def _write_routes(d):
    try:
        os.makedirs(os.path.dirname(ROUTES_FILE), exist_ok=True)
        tmp = ROUTES_FILE + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(d, f, ensure_ascii=False)
        os.replace(tmp, ROUTES_FILE)   # 原子替换, 避免写一半掉电损坏
        return True
    except OSError:
        return False


def get_route(sensor, name):
    """返回 {'points':[{x,y,yaw},...], 'loop':bool}; 没有则 points 为空。"""
    r = _load_routes().get(_route_key(sensor, name))
    if not isinstance(r, dict):
        return {'points': [], 'loop': False}
    pts = r.get('points') if isinstance(r.get('points'), list) else []
    return {'points': pts, 'loop': bool(r.get('loop'))}


def save_route(sensor, name, points, loop):
    d = _load_routes()
    key = _route_key(sensor, name)
    if points:
        # 只留 x/y/yaw 三个字段, 且都转成 float, 防止前端塞进别的东西
        clean = [{'x': float(p['x']), 'y': float(p['y']),
                  'yaw': float(p.get('yaw', 0.0))}
                 for p in points if 'x' in p and 'y' in p]
        d[key] = {'points': clean, 'loop': bool(loop)}
    else:
        d.pop(key, None)   # 空航点等于清除这张图的巡航路线
    return _write_routes(d)


def delete_route(sensor, name):
    d = _load_routes()
    if d.pop(_route_key(sensor, name), None) is not None:
        _write_routes(d)


def list_2d_maps():
    """扫描 install 版地图目录, 列出可用的 <name>.yaml+.pgm 配对(mid360/n10plus共用)。"""
    maps = []
    if os.path.isdir(MAP_DIR_INSTALL):
        for f in sorted(os.listdir(MAP_DIR_INSTALL)):
            if not f.endswith('.yaml'):
                continue
            name = f[:-5]
            pgm = os.path.join(MAP_DIR_INSTALL, name + '.pgm')
            if not os.path.exists(pgm):
                continue
            st = os.stat(pgm)
            maps.append({'name': name, 'mtime': int(st.st_mtime),
                        'size_kb': round(st.st_size / 1024, 1)})
    maps.sort(key=lambda m: -m['mtime'])
    return maps


def list_odin1_maps():
    maps = []
    if os.path.isdir(ODIN1_MAP_DIR):
        for f in sorted(os.listdir(ODIN1_MAP_DIR)):
            if not f.endswith('.bin'):
                continue
            st = os.stat(os.path.join(ODIN1_MAP_DIR, f))
            maps.append({'name': f[:-4], 'mtime': int(st.st_mtime),
                        'size_kb': round(st.st_size / 1024, 1)})
    maps.sort(key=lambda m: -m['mtime'])
    return maps

# 外部实例探测特征 (与各 launch 文件自己的防重复启动逻辑一致, 覆盖三种传感器
# 各自的关键进程名, 缺一个就会漏判导致双实例同时抢串口/cmd_vel)
EXT_PATTERN = ('component_container_isolated|livox_ros_driver2_node|'
               'wheeltec_robot_node|fastlio_mapping|lslidar_driver_node|'
               'async_slam_toolbox_node|host_sdk_sample')
_LAUNCH_PKG_PATTERN = 'wheeltec_fastlio|wheeltec_n10plus|wheeltec_odin1'

# 网络: 板载网卡(wlan0)常连家里WiFi, USB网卡(wlan1)常开热点, 两块卡同时在线。
#
# 2026-07-24 架构变更: 原来是"一块网卡二选一切换"(wlan0 在 STA/AP 之间切),
# 但板载卡是 RTL8852BE + Realtek树外驱动 8852be.ko, 不走 mac80211, 向 cfg80211
# 上报的 interface combinations 是空的, 实测切AP会整机卡死(详见 CLAUDE.md)。
# 现在改成两块物理网卡各司其职: 是两个独立的 phy, 互不干涉, 不存在"切换"这回事,
# 热点开关也不会影响到走WiFi的SSH/网页连接。
STA_PROFILE = 'dmx-public'          # 板载卡 wlan0 连家里WiFi用的连接名
AP_PROFILE = 'wheeltec-ap-usb'      # USB卡 wlan1 常开热点用的连接名
AP_IFACE = 'wlan1'
AP_SSID = 'WHEELTEC-CAR'
AP_ADDR = '192.168.0.100'

# 传感器"是否已接入"探测用的硬件特征 (仅供网页UI提示/自动预选, 不代表驱动一定
# 能正常握手): N10Plus看udev生成的固定设备名, Odin1看官方文档给的USB VID:PID
# (host_sdk_sample.cpp里硬编码的同一对值), MID360s看配置文件里记录的雷达IP能否ping通。
MID360_CONFIG_PATH = os.path.join(
    HOME, 'wheeltec_ros2/src/wheeltec_fastlio/config/MID360s_config.json')
N10PLUS_DEV = '/dev/wheeltec_lidar'
ODIN1_USB_VENDOR, ODIN1_USB_PRODUCT = '2207', '0019'


def _mid360_lidar_ip():
    try:
        with open(MID360_CONFIG_PATH) as f:
            cfg = json.load(f)
        return cfg['lidar_configs'][0]['ip']
    except Exception:
        return None


def detect_sensors():
    """三种传感器各自的硬件在线探测, 每次调用 <1s, 供周期状态广播使用。"""
    det = {'mid360': False, 'n10plus': False, 'odin1': False}
    det['n10plus'] = os.path.exists(N10PLUS_DEV)
    try:
        for dev in glob.glob('/sys/bus/usb/devices/*'):
            try:
                with open(os.path.join(dev, 'idVendor')) as f:
                    vendor = f.read().strip()
                with open(os.path.join(dev, 'idProduct')) as f:
                    product = f.read().strip()
            except OSError:
                continue
            if vendor == ODIN1_USB_VENDOR and product == ODIN1_USB_PRODUCT:
                det['odin1'] = True
                break
    except OSError:
        pass
    ip = _mid360_lidar_ip()
    if ip:
        try:
            r = subprocess.run(['ping', '-c', '1', '-W', '0.3', ip],
                                capture_output=True, timeout=1.5)
            det['mid360'] = r.returncode == 0
        except Exception:
            pass
    return det


def _iface_ip(iface):
    try:
        r = subprocess.run(['ip', '-4', '-o', 'addr', 'show', iface],
                           capture_output=True, text=True, timeout=3)
        if r.stdout.strip():
            return r.stdout.split()[3].split('/')[0]
    except Exception:
        pass
    return ''


def _ap_client_count():
    """连在热点上的设备数。station dump 是驱动直接给的关联列表, 比数DHCP租约准
    (租约会残留已经走掉的设备)。"""
    try:
        r = subprocess.run(['iw', 'dev', AP_IFACE, 'station', 'dump'],
                           capture_output=True, text=True, timeout=3)
        return sum(1 for ln in r.stdout.splitlines()
                   if ln.startswith('Station'))
    except Exception:
        return 0


def wifi_status():
    """两块网卡各自的状态: 板载卡(STA, 连家里WiFi)和USB卡(AP, 热点)。
    两者互相独立, 可以同时在线, 也可以任一为空。"""
    sta = {'name': '', 'ip': '', 'iface': '', 'up': False}
    ap = {'name': AP_PROFILE, 'ip': '', 'iface': AP_IFACE, 'up': False,
          'ssid': AP_SSID, 'clients': 0, 'exists': False}
    try:
        r = subprocess.run(['nmcli', '-t', '-f', 'NAME,TYPE,DEVICE', 'con',
                            'show', '--active'],
                           capture_output=True, text=True, timeout=3)
        for line in r.stdout.strip().splitlines():
            parts = line.split(':')
            if len(parts) < 3 or parts[1] != '802-11-wireless':
                continue
            name, dev = parts[0], parts[2]
            if name == AP_PROFILE:
                ap.update(up=True, iface=dev, ip=_iface_ip(dev),
                          clients=_ap_client_count())
            elif not sta['up']:
                # AP_PROFILE 之外的活动无线连接就是站点模式(连着某个现有WiFi)
                sta.update(up=True, name=name, iface=dev, ip=_iface_ip(dev))
    except Exception:
        pass
    try:
        r = subprocess.run(['nmcli', '-t', '-f', 'NAME', 'con', 'show'],
                           capture_output=True, text=True, timeout=3)
        ap['exists'] = AP_PROFILE in r.stdout.split('\n')
    except Exception:
        pass
    return {'sta': sta, 'ap': ap}


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _despeckle(occ):
    """去掉8邻域内没有任何同伴的孤立障碍像素 (仅影响网页显示, 不改导航数据)。"""
    o = occ.astype(np.uint8)
    n = np.zeros(o.shape, np.uint8)
    n[1:, :] += o[:-1, :]; n[:-1, :] += o[1:, :]
    n[:, 1:] += o[:, :-1]; n[:, :-1] += o[:, 1:]
    n[1:, 1:] += o[:-1, :-1]; n[1:, :-1] += o[:-1, 1:]
    n[:-1, 1:] += o[1:, :-1]; n[:-1, :-1] += o[1:, 1:]
    return occ & (n > 0)


def _paint(free, occ, h, w):
    """浅色主题配色: 自由区纯白、障碍深蓝灰、未知半透明浅灰。"""
    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    rgba[:, :] = (154, 164, 178, 78)        # 未知区
    rgba[free] = (255, 255, 255, 255)
    rgba[_despeckle(occ)] = (39, 50, 68, 255)
    return rgba


def grid_to_png(msg: OccupancyGrid):
    """OccupancyGrid -> (meta json bytes, png bytes)。PNG首行=y最大的一行。"""
    w, h = msg.info.width, msg.info.height
    data = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
    occ = data >= 50
    free = (data >= 0) & ~occ
    rgba = np.flipud(_paint(free, occ, h, w))
    buf = io.BytesIO()
    Image.fromarray(rgba, 'RGBA').save(buf, 'PNG', compress_level=4)
    meta = {'res': msg.info.resolution,
            'ox': msg.info.origin.position.x,
            'oy': msg.info.origin.position.y,
            'w': w, 'h': h}
    return json.dumps(meta).encode(), buf.getvalue()


def pgm_to_png(pgm_path, yaml_path):
    """读取 save_map 产出的 pgm+yaml, 打包成与 /map 相同格式的地图帧(预览用)。"""
    import yaml as pyyaml
    info = pyyaml.safe_load(open(yaml_path))
    img = np.asarray(Image.open(pgm_path).convert('L'))
    h, w = img.shape
    occ = img < 100          # pgm: 0=占据 205=未知 254=自由
    free = img > 220
    rgba = _paint(free, occ, h, w)
    buf = io.BytesIO()
    Image.fromarray(rgba, 'RGBA').save(buf, 'PNG', compress_level=4)
    meta = {'res': float(info['resolution']),
            'ox': float(info['origin'][0]), 'oy': float(info['origin'][1]),
            'w': w, 'h': h, 'name': os.path.basename(pgm_path)}
    return json.dumps(meta).encode(), buf.getvalue()


def pgm_to_edit_png(pgm_path, yaml_path):
    """读取地图, 转成编辑专用的精确三色PNG(白=自由/黑=障碍/灰=未知)。
    跟 pgm_to_png 的展示配色(去噪/半透明)不同, 这里要求像素值精确对应
    PGM里的真实占据状态, 保存时才能无歧义地分类回写。"""
    import yaml as pyyaml
    info = pyyaml.safe_load(open(yaml_path))
    img = np.asarray(Image.open(pgm_path).convert('L'))
    h, w = img.shape
    occ = img < 100
    free = img > 220
    rgba = np.full((h, w, 4), (128, 128, 128, 255), dtype=np.uint8)
    rgba[free] = (255, 255, 255, 255)
    rgba[occ] = (0, 0, 0, 255)
    buf = io.BytesIO()
    Image.fromarray(rgba, 'RGBA').save(buf, 'PNG', compress_level=1)
    meta = {'res': float(info['resolution']),
            'ox': float(info['origin'][0]), 'oy': float(info['origin'][1]),
            'w': w, 'h': h}
    return json.dumps(meta).encode(), buf.getvalue()


def edit_png_to_pgm(png_bytes, dest_paths, orig_yaml_path):
    """把编辑后的三色PNG(白/黑/灰)按最近邻分类写回标准trinary PGM+YAML,
    分辨率/原点沿用原地图不变。dest_paths: [(pgm_path, yaml_path), ...] 可以
    同时写多份(install+src)。像素格式跟 pcd2pgm.cpp 的 save_map() 保持一致,
    避免同一份数据两套代码写出不兼容的文件。"""
    import yaml as pyyaml
    info = pyyaml.safe_load(open(orig_yaml_path))
    img = np.asarray(Image.open(io.BytesIO(png_bytes)).convert('RGB')).astype(np.int16)
    gray = img.mean(axis=2)
    h, w = gray.shape
    out = np.full((h, w), 0xCD, dtype=np.uint8)     # 默认未知(205)
    out[gray > 200] = 0xFE                          # 白->自由(254)
    out[gray < 80] = 0x00                           # 黑->占据(0)
    origin = info['origin']
    for pgm_path, yaml_path in dest_paths:
        os.makedirs(os.path.dirname(pgm_path), exist_ok=True)
        Image.fromarray(out, 'L').save(pgm_path)
        name = os.path.basename(pgm_path)
        with open(yaml_path, 'w') as f:
            f.write(f"image: {name}\nmode: trinary\n"
                    f"resolution: {info['resolution']}\n"
                    f"origin: [{origin[0]}, {origin[1]}, "
                    f"{origin[2] if len(origin) > 2 else 0.0}]\n"
                    f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")


def pack_map(meta_json, png, btype=B_MAP):
    return bytes([btype]) + struct.pack('<I', len(meta_json)) + meta_json + png


def costmap_to_png(msg: OccupancyGrid):
    """局部代价地图(避障膨胀区) -> 半透明热力图叠加层。
    costmap_2d_publisher的换算: 0=空闲, 1-98=膨胀衰减梯度, 99=内切碰撞, 100=致命障碍,
    -1=未知。空闲/未知全透明(不遮挡底图), 成本越高越不透明越偏红——
    直观回答"为什么会撞": 看红色范围有没有把真实障碍物包住、包多宽。"""
    w, h = msg.info.width, msg.info.height
    data = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
    t = np.clip(data, 0, 100).astype(np.float32) / 100.0
    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    rgba[..., 0] = 255
    rgba[..., 1] = (255 * (1 - t) * 0.8).astype(np.uint8)
    rgba[..., 2] = 0
    rgba[..., 3] = np.where(data > 0, (55 + t * 175).astype(np.uint8), 0)
    rgba = np.flipud(rgba)
    buf = io.BytesIO()
    Image.fromarray(rgba, 'RGBA').save(buf, 'PNG', compress_level=4)
    meta = {'res': msg.info.resolution,
            'ox': msg.info.origin.position.x,
            'oy': msg.info.origin.position.y,
            'w': w, 'h': h}
    return json.dumps(meta).encode(), buf.getvalue()


def pack_floats(btype, arr):
    return bytes([btype]) + np.asarray(arr, dtype='<f4').tobytes()


def cloud_xyz(msg: PointCloud2, max_pts=1200):
    """手动解析 PointCloud2 的 x,y,z (无 sensor_msgs_py 依赖), 均匀抽稀。"""
    off = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= off.keys():
        return None
    n = msg.width * msg.height
    if n == 0:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    step = max(1, n // max_pts)
    raw = raw[::step][:max_pts]
    out = np.empty((len(raw), 3), dtype=np.float32)
    for i, k in enumerate(('x', 'y', 'z')):
        out[:, i] = raw[:, off[k]:off[k] + 4].copy().view('<f4').ravel()
    good = np.isfinite(out).all(axis=1) & (out[:, 2] > -1.5) & (out[:, 2] < 3.0)
    return out[good]


class LaunchManager:
    """托管 wheeltec_fastlio 各 launch 的启停; 严格遵守项目规范:
    停止只发 SIGINT(等价Ctrl+C, 建图靠它保存PCD), 超时才 SIGKILL + 清 /dev/shm。"""

    def __init__(self, loop, on_line, on_exit):
        self.loop = loop
        self.on_line = on_line      # (str) -> None, 须在loop线程调用
        self.on_exit = on_exit      # (sensor, mode, returncode) -> None
        self.proc = None
        self.sensor = 'mid360'      # 上一次/当前使用的传感器档, 空闲时保留供UI显示
        self.mode = 'idle'
        self.stopping = False
        self.map_name = ''          # 本次任务真正加载的地图名(空=launch自带默认图)

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, sensor, mode, launch_args=None):
        profile = SENSOR_PROFILES.get(sensor)
        if not profile:
            return False, f'未知传感器 {sensor}'
        if mode not in profile['launches']:
            return False, f'{profile["label"]} 不支持 {mode} 模式'
        if self.alive():
            return False, f'当前有任务在运行({self.sensor}/{self.mode}), 请先停止'
        if self._external_pids():
            return False, '检测到外部启动的 ROS 实例, 请先停止它(可用"停止"按钮接管)'
        self.sensor = sensor
        self.mode = mode
        self.stopping = False
        # 记下这次真正加载的是哪张图, 供UI显示"当前导航用的地图"。
        # 前端那个"导航将使用"只是下次启动的预选值(存在localStorage里), 任务跑起来
        # 之后再点别的图它也会变, 但实际运行的还是启动时那张 —— 必须分开显示。
        la = launch_args or {}
        self.map_name = (la.get('map_name')
                         or (os.path.splitext(os.path.basename(la['map']))[0]
                             if la.get('map') else ''))
        # 分配伪终端: 个别节点(nav2_waypoint_cycle)会打开 /dev/tty 读键盘,
        # 无控制终端时直接崩; pty 同时让子进程行缓冲, 日志实时。
        master, slave = pty.openpty()

        def _preexec():
            os.setsid()
            fcntl.ioctl(0, termios.TIOCSCTTY, 0)

        env = os.environ.copy()
        if env.get('FASTRTPS_DEFAULT_PROFILES_FILE') == _PROFILE_XML:
            env.pop('FASTRTPS_DEFAULT_PROFILES_FILE')   # 子进程用默认SHM传输
        extra = [f'{k}:={v}' for k, v in (launch_args or {}).items()]
        self.proc = subprocess.Popen(
            ['ros2', 'launch', profile['package'], profile['launches'][mode]] + extra,
            stdin=slave, stdout=slave, stderr=slave,
            preexec_fn=_preexec, close_fds=True, env=env)
        os.close(slave)
        threading.Thread(target=self._reader, args=(self.proc, master, sensor, mode),
                         daemon=True).start()
        return True, f'{profile["label"]}/{mode} 已启动 (pid {self.proc.pid})'

    ANSI_RE = re.compile(r'\x1b\[[0-9;?]*[a-zA-Z]')

    def _reader(self, proc, master, sensor, mode):
        buf = b''
        last, repeat = None, 0
        while True:
            try:
                chunk = os.read(master, 4096)
            except OSError:        # 子进程退出后 pty 读到 EIO
                chunk = b''
            if not chunk:
                break
            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                line = self.ANSI_RE.sub(
                    '', raw.decode('utf-8', 'replace')).rstrip('\r ')
                if not line:
                    continue
                # 相同内容刷屏(如 imu data freeze)节流: 每50条放行1条
                if line == last:
                    repeat += 1
                    if repeat % 50:
                        continue
                    line += f'  (已重复{repeat}次)'
                else:
                    last, repeat = line, 0
                self.loop.call_soon_threadsafe(self.on_line, line)
        os.close(master)
        rc = proc.wait()
        self.loop.call_soon_threadsafe(self.on_exit, sensor, mode, rc)

    def _external_pids(self):
        """本进程管理之外的 ROS 实例 pid 列表。"""
        r = subprocess.run(['pgrep', '-f', EXT_PATTERN],
                           capture_output=True, text=True)
        pids = [int(p) for p in r.stdout.split()]
        if self.alive():
            # 排除自己孩子: 同进程组的都算自己人
            pgid = self.proc.pid
            own = []
            for p in pids:
                try:
                    if os.getpgid(p) == pgid:
                        own.append(p)
                except OSError:
                    own.append(p)
            pids = [p for p in pids if p not in own]
        return pids

    async def stop(self):
        """优雅停止: SIGINT -> 最多等22s -> SIGKILL + 清SHM。返回描述文本。"""
        if self.alive():
            self.stopping = True
            pgid = self.proc.pid
            try:
                os.killpg(pgid, signal.SIGINT)
            except ProcessLookupError:
                pass
            for i in range(44):                 # 22s, nav2容器bond死锁社区已知
                if not self.alive():
                    self.stopping = False
                    return '已正常停止 (SIGINT)'
                if i == 24:                     # 12s 再补一次 SIGINT
                    try:
                        os.killpg(pgid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                await asyncio.sleep(0.5)
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            await asyncio.sleep(1.0)
            self._cleanup_shm()
            self.stopping = False
            return '优雅退出超时, 已强杀并清理 /dev/shm'
        # 接管停止外部实例: 找 ros2 launch 主进程发 SIGINT
        ext = self._external_pids()
        if not ext:
            return '当前没有在运行的任务'
        r = subprocess.run(['pgrep', '-f', fr'ros2 launch ({_LAUNCH_PKG_PATTERN})'],
                           capture_output=True, text=True)
        launchers = [int(p) for p in r.stdout.split() if int(p) != os.getpid()]
        for p in launchers or ext:
            try:
                os.kill(p, signal.SIGINT)
            except ProcessLookupError:
                pass
        for _ in range(30):
            if not self._external_pids():
                return '外部实例已停止'
            await asyncio.sleep(0.5)
        for p in self._external_pids():
            try:
                os.kill(p, signal.SIGKILL)
            except ProcessLookupError:
                pass
        await asyncio.sleep(1.0)
        self._cleanup_shm()
        return '外部实例已强杀并清理 /dev/shm'

    def _cleanup_shm(self):
        """仅在无其它ROS进程时清理 FastDDS 残留 (fastrtps_* 与 sem.fastrtps_* 都要)。"""
        if self._external_pids():
            return
        for f in (glob.glob('/dev/shm/fastrtps_*') +
                  glob.glob('/dev/shm/sem.fastrtps_*') +
                  glob.glob('/dev/shm/fast_datasharing*')):
            try:
                os.remove(f)
            except OSError:
                pass


class Bridge(Node):
    """rclpy 节点: 订阅可视化话题(存最新), 发布控制话题。回调在ROS线程执行。"""

    def __init__(self):
        super().__init__('wheeltec_webapp')
        self.lock = threading.Lock()
        self.latest = {}            # scan/map/plan/odom/voltage 最新消息
        self.map_seq = 0
        self.costmap_seq = 0
        self._param_clients = {}
        self.cloud_out = None       # 已抽稀的点云 (numpy), 待广播
        self._last_cloud_t = 0.0

        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        sensor_qos = QoSProfile(depth=2,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(OccupancyGrid, '/map', self._cb('map'), map_qos)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self._cb('costmap'), map_qos)
        self.create_subscription(LaserScan, '/scan', self._cb('scan'), sensor_qos)
        self.create_subscription(Path, '/plan', self._cb('plan'), 5)
        # 局部规划路径: MPPI 控制器不发 /local_plan(那是DWB的), 它在 visualize:true 时
        # 发布 /transformed_global_plan(裁剪到局部代价图窗口、正在跟踪的那段近距离路径),
        # 且仅当有人订阅才发。这里订阅它作为"局部路径"显示。
        self.create_subscription(Path, '/transformed_global_plan',
                                 self._cb('local_plan'), 5)
        self.create_subscription(Odometry, '/Odometry', self._cb('odom'), 10)
        self.create_subscription(Float32, 'PowerVoltage', self._cb('voltage'), 5)
        self.create_subscription(PointCloud2, '/cloud_registered',
                                 self.cloud_cb, sensor_qos)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 2)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 2)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped,
                                              '/initialpose', 2)
        self.cli_reloc = self.create_client(Trigger, '/relocalize')
        self.cli_cancel = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.ac_nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.ac_wps = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self, spin_thread=False)

    def _cb(self, key):
        def f(msg):
            with self.lock:
                self.latest[key] = msg
                if key == 'map':
                    self.map_seq += 1
                elif key == 'costmap':
                    self.costmap_seq += 1
                elif key == 'odom':
                    self.odom_mono = time.monotonic()
        return f

    def param_client(self, node_fqn):
        """rcl_interfaces/SetParameters 客户端(懒加载复用): 用于运行时调整
        nav2 costmap/MPPI 参数(避障膨胀半径、导航速度上限), 仅本次导航会话有效,
        不写回yaml, 重启导航会恢复配置文件默认值。"""
        if node_fqn not in self._param_clients:
            self._param_clients[node_fqn] = self.create_client(
                SetParameters, f'{node_fqn}/set_parameters')
        return self._param_clients[node_fqn]

    def cloud_cb(self, msg):
        now = time.monotonic()
        if now - self._last_cloud_t < 1.0:   # 1Hz 足够看建图效果
            return
        self._last_cloud_t = now
        pts = cloud_xyz(msg)
        if pts is not None and len(pts):
            with self.lock:
                self.cloud_out = pts

    # ---- 控制 ----
    def send_vel(self, vx, wz):
        t = Twist()
        t.linear.x = max(-VX_CAP, min(VX_CAP, float(vx)))
        t.angular.z = max(-WZ_CAP, min(WZ_CAP, float(wz)))
        self.pub_vel.publish(t)

    def make_pose(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = yaw_to_quat(float(yaw))
        return m

    def send_initialpose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = \
            yaw_to_quat(float(yaw))
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.068
        m.pose.covariance = cov
        self.pub_init.publish(m)

    def robot_pose(self):
        """建图时(FAST-LIO /Odometry 在流)必须用它——与点云/轨迹同在 camera_init 系;
        导航时优先 map->base_footprint, AMCL未定位则退回 EKF 里程计。"""
        with self.lock:
            odom = self.latest.get('odom')
            fresh = time.monotonic() - getattr(self, 'odom_mono', 0) < 2.0
        if odom is not None and fresh:
            p = odom.pose.pose
            return {'x': p.position.x, 'y': p.position.y,
                    'yaw': quat_to_yaw(p.orientation), 'frame': 'lio'}
        for parent, frame in (('map', 'map'), ('odom_combined', 'odom')):
            try:
                tf = self.tf_buf.lookup_transform(parent, 'base_footprint',
                                                  rclpy.time.Time())
                t, q = tf.transform.translation, tf.transform.rotation
                return {'x': t.x, 'y': t.y, 'yaw': quat_to_yaw(q),
                        'frame': frame}
            except Exception:
                pass
        return None


class WebConsole:
    def __init__(self, node: Bridge, host, port):
        self.node = node
        self.host, self.port = host, port
        self.loop = None
        self.clients = set()
        self.log_buf = []           # 最近300行launch日志
        self.mgr = None
        self.map_cache = None       # 最近一次打包好的地图帧
        self.map_cache_seq = -1
        self.costmap_cache_seq = -1
        self.traj = []              # 建图轨迹 (x,y)
        self.last_scan_stamp = None
        self.last_plan_stamp = None
        self.last_local_plan_stamp = None
        self.reloc_mode = 'auto'     # 'auto'=自动重定位模式 / 'odom'=里程计模式
        self.cmd_active = False
        self.cmd_last_t = 0.0
        self.ext_running = False
        self.save_report = ''
        self.goal_handle = None     # 当前活动的 nav2 action goal
        self.nav_task = None
        self.nav_info = {'state': 'idle'}
        self.wps_loop = False
        self.wps_points = []
        self._fb_mono = 0.0
        self.wifi = {'sta': {'name': '', 'ip': '', 'iface': '', 'up': False},
                     'ap': {'name': AP_PROFILE, 'ip': '', 'iface': AP_IFACE,
                            'up': False, 'ssid': AP_SSID, 'clients': 0,
                            'exists': False}}
        self.detected = {'mid360': False, 'n10plus': False, 'odin1': False}
        self.map_saved_this_session = False   # n10plus/odin1: 本次建图是否已存过图
        self.pending_map_name = 'WHEELTEC3D'  # mid360 save_map 这次用的地图名
        self.current_odin1_map_name = 'odin1_map'  # odin1本次建图/导航用的地图名
        self._index_path = os.path.join(STATIC_DIR, 'index.html')
        self._index_cache = None      # (mtime, bytes)

    @property
    def index_html(self):
        """按文件修改时间热重载 index.html。
        原来是在 __init__ 里一次性读进内存, 结果 colcon build 出新前端后,
        不重启服务就一直发旧副本 —— 排查时很容易误判成"改动没生效/浏览器缓存"
        (2026-07-24 踩过)。stat 一次的开销可以忽略, 页面请求本来就不频繁。"""
        try:
            mtime = os.path.getmtime(self._index_path)
        except OSError:
            mtime = 0
        if self._index_cache is None or self._index_cache[0] != mtime:
            with open(self._index_path, 'rb') as f:
                self._index_cache = (mtime, f.read())
        return self._index_cache[1]

    # ---------- 广播 ----------
    def _send_all(self, data):
        for ws in list(self.clients):
            asyncio.ensure_future(self._safe_send(ws, data))

    async def _safe_send(self, ws, data):
        try:
            await ws.send(data)
        except Exception:
            pass

    def send_json(self, **kw):
        self._send_all(json.dumps(kw))

    # ---------- launch 管理回调 (loop线程) ----------
    def on_launch_line(self, line):
        self.log_buf.append(line)
        del self.log_buf[:-300]
        if '地面校平完成' in line or '地图已保存' in line or '离地高度' in line:
            self.save_report = line
        self.send_json(t='log', line=line)

    def on_launch_exit(self, sensor, mode, rc):
        label = SENSOR_PROFILES.get(sensor, {}).get('label', sensor)
        self.send_json(t='log', line=f'--- {label}/{mode} 进程退出 (code {rc}) ---')
        if mode == 'save_map':      # 只有 mid360 有这个launch模式
            self._push_saved_map(self.pending_map_name)
            self.send_json(t='maps', sensor='mid360', maps=list_2d_maps())
        if mode == 'mapping':
            if sensor == 'mid360':
                n = len(glob.glob(os.path.join(PCD_DIR, 'scans*.pcd')))
                self.send_json(t='toast',
                    msg=f'建图已结束, PCD分段 {n} 个; 可点击"保存地图"生成2D导航地图')
            elif self.map_saved_this_session:
                self.send_json(t='toast', msg=f'{label} 建图已结束 (本次已保存过地图)')
            else:
                self.send_json(t='toast', ok=False,
                    msg=f'{label} 建图已结束——注意: {label} 的"保存地图"必须在建图'
                        '运行中点击才有效(它是从建图会话里实时读取地图/触发存图指令,'
                        '不是像MID360那样单独一步), 这次没存过, 需要重新建图')
        if mode == 'navigation':
            self.wps_loop = False
            self.goal_handle = None
            self.set_nav(state='idle', kind=None)
        self.push_state()

    def _push_saved_map(self, name='WHEELTEC3D'):
        pgm = os.path.join(MAP_DIR_INSTALL, name + '.pgm')
        yml = os.path.join(MAP_DIR_INSTALL, name + '.yaml')
        if os.path.exists(pgm) and os.path.exists(yml):
            try:
                meta, png = pgm_to_png(pgm, yml)
                frame = pack_map(meta, png)
                self.map_cache, self.map_cache_seq = frame, -2
                self._send_all(frame)
                msg = '2D地图已生成并加载预览'
                if self.save_report:
                    msg += ' | ' + self.save_report.split('] ')[-1]
                self.send_json(t='toast', msg=msg)
            except Exception as e:
                self.send_json(t='toast', msg=f'读取新地图失败: {e}')

    # ---------- 状态 ----------
    def sys_stats(self):
        st = {}
        try:
            mem = {}
            with open('/proc/meminfo') as f:
                for ln in f:
                    k, v = ln.split(':', 1)
                    mem[k] = int(v.strip().split()[0])
            st['mem_avail'] = round(mem.get('MemAvailable', 0) / 1048576, 2)
            st['mem_total'] = round(mem.get('MemTotal', 0) / 1048576, 2)
        except Exception:
            pass
        try:
            s = os.statvfs('/')
            st['disk_free'] = round(s.f_bavail * s.f_frsize / 2**30, 1)
            st['disk_total'] = round(s.f_blocks * s.f_frsize / 2**30, 1)
        except Exception:
            pass
        try:
            st['cpu_temp'] = round(int(open(
                '/sys/class/thermal/thermal_zone0/temp').read()) / 1000, 1)
        except Exception:
            pass
        return st

    def state_dict(self):
        with self.node.lock:
            v = self.node.latest.get('voltage')
        mode = self.mgr.mode if self.mgr.alive() else 'idle'
        # 导航实际加载的地图名(没显式指定就是launch自带的默认图)。
        # 只有导航才谈得上"正在用哪张图"——建图是在造图, 概念不同, 给空串让前端隐藏。
        active_map = ''
        if self.mgr.alive() and mode == 'navigation':
            active_map = (self.mgr.map_name
                          or DEFAULT_MAP_NAME.get(self.mgr.sensor, ''))
        return dict(t='state', mode=mode, sensor=self.mgr.sensor,
                    active_map=active_map,
                    running=self.mgr.alive(), stopping=self.mgr.stopping,
                    external=self.ext_running and not self.mgr.alive(),
                    voltage=round(v.data, 1) if v else None,
                    pcd_count=len(glob.glob(os.path.join(PCD_DIR, 'scans*.pcd'))),
                    has_map=self.map_cache is not None,
                    wifi=self.wifi,
                    detected=self.detected,
                    **self.sys_stats())

    def push_state(self):
        self.send_json(**self.state_dict())

    # ---------- 周期任务 ----------
    async def task_state(self):
        while True:
            try:
                self.ext_running = bool(await asyncio.get_event_loop()
                                        .run_in_executor(None,
                                                         self.mgr._external_pids))
            except Exception:
                pass
            try:
                self.wifi = await asyncio.get_event_loop().run_in_executor(
                    None, wifi_status)
            except Exception:
                pass
            try:
                self.detected = await asyncio.get_event_loop().run_in_executor(
                    None, detect_sensors)
            except Exception:
                pass
            if self.clients:
                self.push_state()
            await asyncio.sleep(2.0)

    async def task_fast(self):
        """5Hz: 位姿; 4Hz: 扫描; 地图/路径按变化推送; 点云/轨迹随建图推送。"""
        tick = 0
        while True:
            if self.clients:
                pose = self.node.robot_pose()
                if pose:
                    self.send_json(t='pose', **pose)
                with self.node.lock:
                    scan = self.node.latest.get('scan')
                    plan = self.node.latest.get('plan')
                    lplan = self.node.latest.get('local_plan')
                    mseq = self.node.map_seq
                    mmsg = self.node.latest.get('map')
                    cmseq = self.node.costmap_seq
                    cmmsg = self.node.latest.get('costmap')
                    cloud = self.node.cloud_out
                    self.node.cloud_out = None
                    odom = self.node.latest.get('odom')
                if scan is not None and tick % 2 == 0 and \
                        scan.header.stamp != self.last_scan_stamp:
                    self.last_scan_stamp = scan.header.stamp
                    self._send_all(pack_floats(B_SCAN, self._scan_xy(scan)))
                if plan is not None and \
                        plan.header.stamp != self.last_plan_stamp:
                    self.last_plan_stamp = plan.header.stamp
                    pts = [(p.pose.position.x, p.pose.position.y)
                           for p in plan.poses]
                    step = max(1, len(pts) // 300)
                    self._send_all(pack_floats(B_PLAN, pts[::step]))
                if lplan is not None and \
                        lplan.header.stamp != self.last_local_plan_stamp:
                    self.last_local_plan_stamp = lplan.header.stamp
                    lpts = [(p.pose.position.x, p.pose.position.y)
                            for p in lplan.poses]
                    lstep = max(1, len(lpts) // 300)
                    self._send_all(pack_floats(B_LOCAL_PLAN, lpts[::lstep]))
                if mmsg is not None and mseq != self.map_cache_seq:
                    self.map_cache_seq = mseq
                    meta, png = grid_to_png(mmsg)
                    self.map_cache = pack_map(meta, png)
                    self._send_all(self.map_cache)
                if cmmsg is not None and cmseq != self.costmap_cache_seq:
                    self.costmap_cache_seq = cmseq
                    meta, png = costmap_to_png(cmmsg)
                    self._send_all(pack_map(meta, png, B_COSTMAP))
                if cloud is not None:
                    self._send_all(pack_floats(B_CLOUD, cloud))
                if odom is not None and tick % 3 == 0:
                    p = odom.pose.pose.position
                    if not self.traj or \
                            (p.x - self.traj[-1][0]) ** 2 + \
                            (p.y - self.traj[-1][1]) ** 2 > 0.0025:
                        self.traj.append((p.x, p.y))
                        del self.traj[:-20000]
                        self._send_all(pack_floats(B_TRAJ, self.traj[-1:]))
            tick += 1
            await asyncio.sleep(0.2)

    @staticmethod
    def _scan_xy(scan: LaserScan):
        r = np.asarray(scan.ranges, dtype=np.float32)
        n = len(r)
        a = scan.angle_min + scan.angle_increment * np.arange(n, dtype=np.float32)
        good = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        r, a = r[good][::2], a[good][::2]
        return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)

    async def task_cmd_watchdog(self):
        """摇杆断流0.6s自动刹车, 防止 WiFi 掉线小车跑飞。"""
        while True:
            if self.cmd_active and time.monotonic() - self.cmd_last_t > 0.6:
                self.cmd_active = False
                self.node.send_vel(0.0, 0.0)
                self.send_json(t='toast', msg='遥控信号中断, 已自动刹车')
            await asyncio.sleep(0.1)

    # ---------- 客户端消息 ----------
    async def handle_msg(self, ws, msg):
        t = msg.get('t')
        if t == 'cmd_vel':
            self.cmd_active = True
            self.cmd_last_t = time.monotonic()
            self.node.send_vel(msg.get('vx', 0.0), msg.get('wz', 0.0))
        elif t == 'cmd_stop':
            self.cmd_active = False
            self.node.send_vel(0.0, 0.0)
        elif t == 'start':
            sensor = msg.get('sensor', 'mid360')
            mode = msg.get('mode')
            map_name = (msg.get('map_name') or '').strip()
            # 任何新任务启动都清掉上一次会话的轨迹/点云/路径残留(建图轨迹是
            # camera_init系, 导航路径是map系, 混杂展示在新地图上容易让人误解)
            self.traj.clear()
            self.send_json(t='clear', what=mode)
            if mode == 'mapping':
                self.map_saved_this_session = False

            launch_args = {}
            if map_name and not _safe_map_name(map_name):
                self.send_json(t='toast', ok=False,
                    msg='地图名不合法(只能中英文/数字/下划线/中划线, 1-64字符)')
                return
            if mode == 'save_map':          # 只有mid360有这个launch模式
                self.pending_map_name = map_name or 'WHEELTEC3D'
                launch_args['map_name'] = self.pending_map_name
            elif mode == 'navigation' and sensor in ('mid360', 'n10plus'):
                if map_name:
                    yaml_path = os.path.join(MAP_DIR_INSTALL, map_name + '.yaml')
                    if not os.path.exists(yaml_path):
                        self.send_json(t='toast', ok=False,
                            msg=f'地图 "{map_name}" 不存在, 请先在地图库里确认名字')
                        return
                    launch_args['map'] = yaml_path
                # 不填map_name则用launch文件自带默认地图(WHEELTEC3D/N10PLUS_MAP)
            elif sensor == 'odin1' and mode in ('mapping', 'navigation'):
                # Odin1 的地图名必须在启动时给定(mapping时决定存哪, navigation时
                # 决定读哪), 不能像MID360/N10Plus那样等存图时才选
                self.current_odin1_map_name = map_name or 'odin1_map'
                launch_args['map_name'] = self.current_odin1_map_name

            ok, info = self.mgr.start(sensor, mode, launch_args)
            if ok:
                hint = {'navigation': ' | 启动约需30秒: 等日志出现"重定位成功"后地图上会出现小车',
                        'mapping': ' | 等待雷达就绪约10秒, 然后用摇杆慢速遥控',
                        'save_map': ' | 转换约需十几秒, 完成后自动加载新地图',
                        }.get(mode, '')
                if sensor in ('n10plus', 'odin1') and mode == 'mapping':
                    hint += ' | 存图必须在这次建图运行中点"保存地图", 停止后就存不了了'
                info += hint
            self.send_json(t='toast', msg=info, ok=ok)
            self.push_state()
        elif t == 'stop':
            self.send_json(t='toast', msg='正在停止 (SIGINT, 建图会先保存数据, 最长等22秒)...')
            self.push_state()
            info = await self.mgr.stop()
            self.send_json(t='toast', msg=info)
            self.push_state()
        elif t == 'goal':
            goal = NavigateToPose.Goal()
            goal.pose = self.node.make_pose(msg['x'], msg['y'], msg['yaw'])
            self.send_json(t='toast',
                           msg=f"目标点已下发 ({msg['x']:.2f}, {msg['y']:.2f})")
            self._spawn_nav(self.node.ac_nav, goal, 'single', 1)
        elif t == 'follow':
            pts = msg.get('points') or []
            if not pts:
                self.send_json(t='toast', msg='航点列表为空', ok=False)
                return
            self.wps_points = pts
            self.wps_loop = bool(msg.get('loop'))
            goal = FollowWaypoints.Goal()
            goal.poses = [self.node.make_pose(p['x'], p['y'], p['yaw'])
                          for p in pts]
            self.send_json(t='toast',
                           msg=f'多点导航已下发: {len(pts)} 个航点'
                               + (' (循环)' if self.wps_loop else ''))
            self._spawn_nav(self.node.ac_wps, goal, 'follow', len(pts))
        elif t == 'initialpose':
            self.node.send_initialpose(msg['x'], msg['y'], msg['yaw'])
            self.send_json(t='toast', msg='初始位姿已下发')
        elif t == 'cancel_goal':
            self.wps_loop = False
            await self._cancel_active()
        elif t == 'relocalize':
            await self._call_relocalize()
        elif t == 'reloc_mode':
            # 里程计模式(odom): 关掉自动重定位看门狗, 侧重里程计, 避免长走廊等
            # 相似场景里频繁误重定位; 自动重定位模式(auto): 看门狗照常工作。
            mode = 'odom' if msg.get('mode') == 'odom' else 'auto'
            ok, err = await self._set_params('/auto_relocalize',
                                             {'watchdog_en': (mode == 'auto')})
            if ok:
                self.reloc_mode = mode
                self.send_json(t='toast', msg=(
                    '已切到自动重定位模式' if mode == 'auto' else
                    '已切到里程计模式 (自动重定位已停, 需要时点"手动重定位")'))
            else:
                self.send_json(t='toast', msg='切换失败: ' + err, ok=False)
            self.send_json(t='reloc_mode', mode=self.reloc_mode)
        elif t == 'estop':
            self.cmd_active = False
            self.wps_loop = False
            self.node.send_vel(0.0, 0.0)
            await self._cancel_active(quiet=True)
            await self._call_cancel(quiet=True)   # 兜底: 取消他处(RViz)发的目标
            self.send_json(t='toast', msg='急停: 已刹车并取消导航目标')
        elif t == 'nav_speed':
            vx = max(0.05, min(1.0, float(msg['vx_max'])))
            wz = max(0.1, min(2.5, float(msg['wz_max'])))
            ok, err = await self._set_params('/controller_server', {
                'FollowPath.vx_max': vx, 'FollowPath.vx_min': -vx,
                'FollowPath.wz_max': wz})
            self.send_json(t='toast', ok=ok,
                msg=(f'导航速度已更新: 线速度上限{vx:.2f}m/s 角速度上限{wz:.2f}rad/s '
                     '(仅本次导航会话有效, 重启导航恢复默认值)') if ok
                    else f'导航速度设置失败: {err}')
        elif t == 'obstacle_params':
            r = max(0.1, min(1.0, float(msg['inflation_radius'])))
            c = max(0.5, min(20.0, float(msg['cost_scaling_factor'])))
            # Humble版MPPI要求ObstaclesCritic与local_costmap的inflation_layer
            # 完全一致(见CLAUDE.md 5.3-4), 三处必须一起改, 否则避障距离换算错乱
            ok1, e1 = await self._set_params('/controller_server', {
                'FollowPath.ObstaclesCritic.inflation_radius': r,
                'FollowPath.ObstaclesCritic.cost_scaling_factor': c})
            ok2, e2 = await self._set_params('/local_costmap/local_costmap', {
                'inflation_layer.inflation_radius': r,
                'inflation_layer.cost_scaling_factor': c})
            ok3, e3 = await self._set_params('/global_costmap/global_costmap', {
                'inflation_layer.inflation_radius': r,
                'inflation_layer.cost_scaling_factor': c})
            ok = ok1 and ok2 and ok3
            self.send_json(t='toast', ok=ok,
                msg=(f'避障参数已更新: 膨胀半径{r:.2f}m 衰减系数{c:.1f} '
                     '(仅本次导航会话有效)') if ok else
                    '避障参数设置失败: ' + '; '.join(filter(None, [e1, e2, e3])))
        elif t == 'ap_power':
            # 热点开关。做在USB网卡(wlan1)上, 跟板载卡(wlan0)完全独立, 所以
            # 开关热点不会影响走WiFi过来的网页/SSH连接 —— 这跟2026-07-23那套
            # "一块卡切模式"的做法有本质区别, 不再需要超时兜底和失联警告。
            on = bool(msg.get('on'))
            verb = '开启' if on else '关闭'
            self.send_json(t='toast', msg=f'正在{verb}热点...')
            print(f'[ap_power] {verb} {AP_PROFILE} @ {time.strftime("%H:%M:%S")}',
                  flush=True)
            try:
                r = await asyncio.get_event_loop().run_in_executor(
                    None, lambda: subprocess.run(
                        ['nmcli', 'con', 'up' if on else 'down', AP_PROFILE],
                        capture_output=True, text=True, timeout=60))
                ok = r.returncode == 0
                err = (r.stderr or r.stdout or '').strip()[:200]
                print(f'[ap_power] rc={r.returncode} stderr={r.stderr!r}',
                      flush=True)
            except subprocess.TimeoutExpired:
                ok, err = False, '60秒未完成'
                print('[ap_power] 超时', flush=True)
            except Exception as e:
                ok, err = False, f'内部异常: {e}'
                print(f'[ap_power] 异常: {e!r}', flush=True)
            self.wifi = await asyncio.get_event_loop().run_in_executor(
                None, wifi_status)
            if ok and on:
                tip = (f'热点已开启: {AP_SSID} — '
                       f'连上后访问 http://{self.wifi["ap"]["ip"] or AP_ADDR}:8080')
            elif ok:
                tip = '热点已关闭'
            else:
                tip = (f'{verb}失败: {err} '
                       f'(热点连接 {AP_PROFILE} 是否已创建? 见 wifi_ap_setup.sh)')
            self.send_json(t='toast', ok=ok, msg=tip)
            self.push_state()
        elif t == 'save_map_n10plus':
            # N10Plus 用 slam_toolbox 的实时 /map, 必须在建图仍在运行时存,
            # 跟MID360那种"先停止建图再单独转换PCD"的流程不一样
            if not (self.mgr.alive() and self.mgr.sensor == 'n10plus'
                    and self.mgr.mode == 'mapping'):
                self.send_json(t='toast', ok=False,
                    msg='请先启动 N10Plus 建图, 且必须在建图运行中才能存图')
                return
            name = (msg.get('name') or '').strip() or 'N10PLUS_MAP'
            if not _safe_map_name(name):
                self.send_json(t='toast', ok=False,
                    msg='地图名不合法(只能中英文/数字/下划线/中划线, 1-64字符)')
                return
            map_path_install = os.path.join(MAP_DIR_INSTALL, name)
            map_path_src = os.path.join(MAP_DIR, name)
            self.send_json(t='toast', msg='正在保存地图...')
            r = await asyncio.get_event_loop().run_in_executor(
                None, lambda: subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                     '-f', map_path_install,
                     '--ros-args', '-p', 'save_map_timeout:=10.0'],
                    capture_output=True, text=True, timeout=20))
            out = (r.stdout or '') + (r.stderr or '')
            ok = r.returncode == 0 and 'Map saved successfully' in out
            for line in out.splitlines():
                if line.strip():
                    self.send_json(t='log', line=line.strip())
            if ok:
                try:
                    os.makedirs(os.path.dirname(map_path_src), exist_ok=True)
                    shutil.copy(map_path_install + '.pgm', map_path_src + '.pgm')
                    shutil.copy(map_path_install + '.yaml', map_path_src + '.yaml')
                except OSError:
                    pass
                self.map_saved_this_session = True
                self.send_json(t='maps', sensor='n10plus', maps=list_2d_maps())
                self.send_json(t='toast', msg=f'N10Plus 地图已保存 ({name})')
            else:
                self.send_json(t='toast', ok=False,
                    msg=f'保存失败: {out.strip()[-200:] or "无输出, 见日志"}')
        elif t == 'save_map_odin1':
            # Odin1 的存图是往一个命令文件里写指令, 由驱动内部的SDK异步处理,
            # 同样必须在建图仍在运行时触发。结果只能从下方日志里driver自己的
            # 输出确认(没有像map_saver_cli那样的同步返回值)。
            if not (self.mgr.alive() and self.mgr.sensor == 'odin1'
                    and self.mgr.mode == 'mapping'):
                self.send_json(t='toast', ok=False,
                    msg='请先启动 Odin1 建图, 且必须在建图运行中才能存图')
                return
            try:
                with open(ODIN1_COMMAND_FILE, 'w') as f:
                    f.write('set save_map 1\n')
                self.map_saved_this_session = True   # 乐观标记(结果需看日志确认)
                self.send_json(t='toast',
                    msg='已发送存图指令, 请看下方日志确认Odin1是否报告保存成功'
                        f'(结果文件在 wheeltec_odin1/map/{self.current_odin1_map_name}.bin)')
            except OSError as e:
                self.send_json(t='toast', ok=False, msg=f'写入指令文件失败: {e}')
        elif t == 'list_maps':
            sensor = msg.get('sensor', 'mid360')
            maps = list_odin1_maps() if sensor == 'odin1' else list_2d_maps()
            self.send_json(t='maps', sensor=sensor, maps=maps)
        elif t == 'get_route':
            # 前端选中某张图时来拉这张图存过的巡航航点
            sensor = msg.get('sensor', 'mid360')
            name = (msg.get('name') or '').strip()
            r = get_route(sensor, name) if _safe_map_name(name) \
                else {'points': [], 'loop': False}
            self.send_json(t='route', sensor=sensor, name=name,
                           points=r['points'], loop=r['loop'])
        elif t == 'save_route':
            sensor = msg.get('sensor', 'mid360')
            name = (msg.get('name') or '').strip()
            if not _safe_map_name(name):
                self.send_json(t='toast', ok=False,
                    msg='请先在地图库选中一张要绑定巡航路线的地图')
                return
            pts = msg.get('points') or []
            ok = save_route(sensor, name, pts, msg.get('loop'))
            self.send_json(t='toast', ok=ok,
                msg=(f'巡航路线已保存到地图 "{name}" ({len(pts)}点)' if pts
                     else f'已清除地图 "{name}" 的巡航路线') if ok
                    else '保存巡航路线失败(磁盘写入错误)')
        elif t == 'delete_map':
            sensor = msg.get('sensor', 'mid360')
            name = (msg.get('name') or '').strip()
            if not _safe_map_name(name):
                self.send_json(t='toast', ok=False, msg='非法地图名')
                return
            try:
                if sensor == 'odin1':
                    p = os.path.join(ODIN1_MAP_DIR, name + '.bin')
                    if os.path.exists(p):
                        os.remove(p)
                else:
                    for d in (MAP_DIR_INSTALL, MAP_DIR):
                        for ext in ('.pgm', '.yaml'):
                            p = os.path.join(d, name + ext)
                            if os.path.exists(p):
                                os.remove(p)
                delete_route(sensor, name)   # 地图删了, 绑在它上面的巡航路线也一起删
                self.send_json(t='toast', msg=f'地图 "{name}" 已删除')
            except OSError as e:
                self.send_json(t='toast', ok=False, msg=f'删除失败: {e}')
            maps = list_odin1_maps() if sensor == 'odin1' else list_2d_maps()
            self.send_json(t='maps', sensor=sensor, maps=maps)
        elif t == 'load_map_edit':
            name = (msg.get('name') or '').strip()
            if not _safe_map_name(name):
                self.send_json(t='toast', ok=False, msg='非法地图名')
                return
            if msg.get('sensor') == 'odin1':
                self.send_json(t='toast', ok=False,
                    msg='Odin1 的 .bin 地图不是图片格式, 不支持编辑')
                return
            pgm = os.path.join(MAP_DIR_INSTALL, name + '.pgm')
            yml = os.path.join(MAP_DIR_INSTALL, name + '.yaml')
            if not (os.path.exists(pgm) and os.path.exists(yml)):
                self.send_json(t='toast', ok=False, msg=f'地图 "{name}" 不存在')
                return
            try:
                meta, png = pgm_to_edit_png(pgm, yml)
                self._send_all(pack_map(meta, png, B_MAP_EDIT))
            except Exception as e:
                self.send_json(t='toast', ok=False, msg=f'加载失败: {e}')
        elif t == 'save_map_edit':
            name = (msg.get('name') or '').strip()
            png_b64 = msg.get('png_b64') or ''
            if not _safe_map_name(name) or not png_b64:
                self.send_json(t='toast', ok=False, msg='参数缺失')
                return
            yml = os.path.join(MAP_DIR_INSTALL, name + '.yaml')
            if not os.path.exists(yml):
                self.send_json(t='toast', ok=False,
                    msg=f'原地图 "{name}" 不存在, 无法确定分辨率/原点')
                return
            try:
                png_bytes = base64.b64decode(png_b64)
                dest = [(os.path.join(MAP_DIR_INSTALL, name + '.pgm'),
                        os.path.join(MAP_DIR_INSTALL, name + '.yaml')),
                       (os.path.join(MAP_DIR, name + '.pgm'),
                        os.path.join(MAP_DIR, name + '.yaml'))]
                edit_png_to_pgm(png_bytes, dest, yml)
                meta, png = pgm_to_png(dest[0][0], dest[0][1])
                self.map_cache, self.map_cache_seq = pack_map(meta, png), -2
                self._send_all(self.map_cache)
                self.send_json(t='toast', msg=f'"{name}" 编辑已保存')
                self.send_json(t='maps', sensor='mid360', maps=list_2d_maps())
            except Exception as e:
                self.send_json(t='toast', ok=False, msg=f'保存失败: {e}')

    async def _set_params(self, node_fqn, kv):
        cli = self.node.param_client(node_fqn)
        if not cli.service_is_ready():
            return False, f'{node_fqn} 服务不在线(导航未启动?)'
        req = SetParameters.Request()

        def _mk(k, v):
            # bool 要在 int 之前判断(Python 里 bool 是 int 的子类)
            if isinstance(v, bool):
                return Parameter(k, Parameter.Type.BOOL, v).to_parameter_msg()
            if isinstance(v, int):
                return Parameter(k, Parameter.Type.INTEGER, v).to_parameter_msg()
            return Parameter(k, Parameter.Type.DOUBLE, float(v)).to_parameter_msg()
        req.parameters = [_mk(k, v) for k, v in kv.items()]
        fut = cli.call_async(req)
        try:
            await asyncio.wait_for(self._wrap_future(fut), 5)
        except asyncio.TimeoutError:
            return False, f'{node_fqn} 设置超时'
        ok = all(r.successful for r in fut.result().results)
        return ok, ('' if ok else f'{node_fqn} 拒绝了部分参数')

    # ---------- nav2 action 导航 ----------
    def set_nav(self, **kw):
        self.nav_info = {**self.nav_info, **kw}
        self.send_json(t='nav', **self.nav_info)

    def _spawn_nav(self, client, goal, kind, total):
        if self.nav_task and not self.nav_task.done():
            old = self.nav_task
            self.nav_task = asyncio.ensure_future(
                self._chain_nav(old, client, goal, kind, total))
        else:
            self.nav_task = asyncio.ensure_future(
                self._run_nav(client, goal, kind, total))

    async def _chain_nav(self, old, client, goal, kind, total):
        await self._cancel_active(quiet=True)
        try:
            await asyncio.wait_for(asyncio.shield(old), 10)
        except Exception:
            pass
        await self._run_nav(client, goal, kind, total)

    async def _run_nav(self, client, goal, kind, total):
        if not client.server_is_ready():
            self.set_nav(state='idle')
            self.send_json(t='toast', ok=False,
                           msg='导航动作服务不在线 (导航未启动或nav2未激活完成)')
            return
        sf = client.send_goal_async(
            goal, feedback_callback=lambda fb: self._on_nav_fb(kind, fb))
        try:
            await asyncio.wait_for(self._wrap_future(sf), 5)
        except asyncio.TimeoutError:
            self.send_json(t='toast', msg='下发目标超时', ok=False)
            return
        gh = sf.result()
        if gh is None or not gh.accepted:
            self.set_nav(state='rejected', kind=kind)
            self.send_json(t='toast', msg='目标被 nav2 拒绝', ok=False)
            return
        self.goal_handle = gh
        self.set_nav(state='active', kind=kind, wp_index=0, wp_total=total,
                     dist=None, time=0, recov=0)
        rf = gh.get_result_async()
        await self._wrap_future(rf)
        self.goal_handle = None
        status = rf.result().status
        name = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                GoalStatus.STATUS_CANCELED: 'canceled',
                GoalStatus.STATUS_ABORTED: 'aborted'}.get(status, 'idle')
        self.set_nav(state=name, kind=kind)
        text = {'succeeded': '✅ 导航到达目标',
                'canceled': '导航已取消',
                'aborted': '❌ 导航失败 (nav2 aborted, 看日志排查)'}.get(name, name)
        if kind == 'follow' and name == 'succeeded':
            missed = list(getattr(rf.result().result, 'missed_waypoints', []))
            if missed:
                text = f'⚠ 多点导航完成, 但错过航点 {[i+1 for i in missed]}'
        self.send_json(t='toast', msg=text, ok=name == 'succeeded')
        # 循环模式: 全部到达后从头再来
        if kind == 'follow' and name == 'succeeded' and self.wps_loop \
                and self.wps_points:
            await asyncio.sleep(1.0)
            if self.wps_loop:
                g = FollowWaypoints.Goal()
                g.poses = [self.node.make_pose(p['x'], p['y'], p['yaw'])
                           for p in self.wps_points]
                self.send_json(t='toast', msg='🔁 循环模式: 重新开始航点巡航')
                self._spawn_nav(self.node.ac_wps, g, 'follow',
                                len(self.wps_points))

    def _on_nav_fb(self, kind, fb):
        """ROS线程回调: 节流0.5s转发导航反馈。"""
        now = time.monotonic()
        if now - self._fb_mono < 0.5:
            return
        self._fb_mono = now
        f = fb.feedback
        if kind == 'single':
            kw = dict(dist=round(f.distance_remaining, 2),
                      time=f.navigation_time.sec,
                      recov=f.number_of_recoveries)
        else:
            kw = dict(wp_index=f.current_waypoint)
        self.loop.call_soon_threadsafe(lambda: self.set_nav(**kw))

    async def _cancel_active(self, quiet=False):
        gh = self.goal_handle
        if gh is None:
            if not quiet:
                await self._call_cancel()   # 可能是他处(RViz)发的目标
            return
        try:
            await asyncio.wait_for(
                self._wrap_future(gh.cancel_goal_async()), 5)
            if not quiet:
                self.send_json(t='toast', msg='已请求取消当前导航')
        except asyncio.TimeoutError:
            if not quiet:
                self.send_json(t='toast', msg='取消请求超时', ok=False)

    async def _call_relocalize(self):
        if not self.node.cli_reloc.service_is_ready():
            self.send_json(t='toast', msg='/relocalize 服务不在线 (导航未启动?)', ok=False)
            return
        fut = self.node.cli_reloc.call_async(Trigger.Request())
        try:
            await asyncio.wait_for(self._wrap_future(fut), timeout=30.0)
            r = fut.result()
            self.send_json(t='toast', msg=f'重定位: {r.message}', ok=r.success)
        except asyncio.TimeoutError:
            self.send_json(t='toast', msg='重定位超时', ok=False)

    async def _call_cancel(self, quiet=False):
        if not self.node.cli_cancel.service_is_ready():
            if not quiet:
                self.send_json(t='toast', msg='导航未启动, 无目标可取消', ok=False)
            return
        req = CancelGoal.Request()      # 全零 = 取消全部目标
        fut = self.node.cli_cancel.call_async(req)
        try:
            await asyncio.wait_for(self._wrap_future(fut), timeout=5.0)
            if not quiet:
                self.send_json(t='toast', msg='已取消当前导航目标')
        except asyncio.TimeoutError:
            if not quiet:
                self.send_json(t='toast', msg='取消目标超时', ok=False)

    async def _wrap_future(self, ros_future):
        ev = asyncio.Event()
        loop = asyncio.get_event_loop()
        ros_future.add_done_callback(
            lambda _: loop.call_soon_threadsafe(ev.set))
        await ev.wait()

    # ---------- websocket ----------
    async def ws_handler(self, ws, path):
        self.clients.add(ws)
        try:
            await ws.send(json.dumps(self.state_dict()))
            await ws.send(json.dumps({'t': 'nav', **self.nav_info}))
            if self.map_cache:
                await ws.send(self.map_cache)
            if self.traj:
                await ws.send(pack_floats(B_TRAJ, self.traj))
            for line in self.log_buf[-100:]:
                await ws.send(json.dumps({'t': 'log', 'line': line}))
            async for raw in ws:
                if isinstance(raw, str):
                    try:
                        await self.handle_msg(ws, json.loads(raw))
                    except Exception as e:
                        await self._safe_send(
                            ws, json.dumps({'t': 'toast',
                                            'msg': f'指令错误: {e}', 'ok': False}))
        finally:
            self.clients.discard(ws)
            if not self.clients and self.cmd_active:
                self.cmd_active = False
                self.node.send_vel(0.0, 0.0)

    async def http_handler(self, path, headers):
        if headers.get('Upgrade', '').lower() == 'websocket':
            return None
        p = path.split('?')[0]
        if p in ('/', '/index.html'):
            return (http.HTTPStatus.OK,
                    [('Content-Type', 'text/html; charset=utf-8'),
                     ('Cache-Control', 'no-cache')],
                    self.index_html)
        # 预生成的语音播报音频 (供红米平板等无 Web Speech API 的系统浏览器播放)。
        # 只允许 [A-Za-z0-9_-].mp3, 防目录穿越。运行时纯静态, 不再依赖浏览器 TTS。
        m = re.fullmatch(r'/voice/([A-Za-z0-9_-]+\.mp3)', p)
        if m:
            fp = os.path.join(STATIC_DIR, 'voice', m.group(1))
            if os.path.isfile(fp):
                with open(fp, 'rb') as f:
                    return (http.HTTPStatus.OK,
                            [('Content-Type', 'audio/mpeg'),
                             ('Cache-Control', 'max-age=86400')],
                            f.read())
        return http.HTTPStatus.NOT_FOUND, [], b'404'

    async def main(self):
        self.loop = asyncio.get_event_loop()
        self.mgr = LaunchManager(self.loop, self.on_launch_line,
                                 self.on_launch_exit)
        # 启动时预览地图库里最近修改的一张(只是画布预览, 不代表导航会用它——
        # 导航实际用哪张现在由启动时选择的地图名决定, 见地图库UI)
        try:
            maps = list_2d_maps()
            if maps:
                name = maps[0]['name']
                pgm = os.path.join(MAP_DIR_INSTALL, name + '.pgm')
                yml = os.path.join(MAP_DIR_INSTALL, name + '.yaml')
                meta, png = pgm_to_png(pgm, yml)
                self.map_cache = pack_map(meta, png)
        except Exception:
            pass
        async with websockets.serve(self.ws_handler, self.host, self.port,
                                    process_request=self.http_handler,
                                    compression=None, max_size=2 ** 20,
                                    ping_interval=10, ping_timeout=20):
            print(f'* Web控制台已启动: http://<小车IP>:{self.port}  (Ctrl+C 退出)')
            await asyncio.gather(self.task_state(), self.task_fast(),
                                 self.task_cmd_watchdog())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-p', '--port', type=int, default=8080)
    ap.add_argument('--host', default='0.0.0.0')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Bridge()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    console = WebConsole(node, args.host, args.port)
    try:
        asyncio.run(console.main())
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前把托管的launch优雅停掉, 不留孤儿进程
        if console.mgr and console.mgr.alive():
            try:
                os.killpg(console.mgr.proc.pid, signal.SIGINT)
                console.mgr.proc.wait(timeout=20)
            except Exception:
                try:
                    os.killpg(console.mgr.proc.pid, signal.SIGKILL)
                except Exception:
                    pass
        node.send_vel(0.0, 0.0)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
