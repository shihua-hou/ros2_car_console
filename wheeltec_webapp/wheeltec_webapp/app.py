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

# 必须在 rclpy.init 之前设置: webapp 自身只走本机回环 UDP(见 localhost_only.xml 注释);
# 子 launch 剔除此变量、改设 ROS_LOCALHOST_ONLY=1(共享内存 + 回环 UDP), 全车 ROS 通信只在本机,
# 不受 Wi-Fi 换网/改 IP 影响, 也不往 Wi-Fi 上发任何 ROS 数据(2026-09-22)。
_PROFILE_XML = os.path.join(os.path.dirname(__file__), 'static', 'localhost_only.xml')
if 'FASTRTPS_DEFAULT_PROFILES_FILE' not in os.environ:
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = _PROFILE_XML

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image as ImageMsg, Imu, LaserScan, PointCloud2, NavSatFix
from std_msgs.msg import Float32, String, UInt8
from std_srvs.srv import Trigger
try:
    from nav2_msgs.srv import LoadMap        # 编辑完热加载进运行中的导航用
except ImportError:                          # nav2 没装/没编时不至于整个 webapp 起不来
    LoadMap = None
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from nav2_msgs.msg import SpeedLimit
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import websockets

HOME = os.path.expanduser('~')
PCD_DIR = os.path.join(HOME, 'wheeltec_ros2/src/FAST_LIO/PCD')

# 户外模式的障碍带上限(米)。室内用 launch 默认的 0.45(=小车可通行高度),
# 户外空旷草坪上 0.45 以下几乎没有可匹配结构, 放开到 2.0 把楼房树木纳入。
# 改这个值必须同时改 outdoor_navigation.launch.py 的 scan_max_z 默认值,
# 地图和 /scan 不同带会让 AMCL 的似然场把差集当失配。
OUTDOOR_MAX_Z = 2.0

# 户外障碍带下限(米)。室内 0.10 是为了抓桌椅腿; 草坪上草高就有 0.1~0.3m,
# 用 0.10 会把整片草判成障碍 —— 2026-09-15 实测: 0.10->0.40 可通行格 +86%、
# 障碍 3.5%->2.4%。取 0.30 折中: 清掉大部分草, 又不至于漏掉太矮的真实障碍。
# 若场地草很高可调大, 若需要识别矮路沿则调小。
OUTDOOR_MIN_Z = 0.30

# 户外存图时"一格里至少几个点才算障碍"的默认值。
# 保持 5 = pcd2pgm 自带默认, 也是室内路径用的值。
#
# 2026-09-15 一度改成 10, 当天测完改回来 —— 方向是反的:
# 同一份户外点云里 30.1% 的点(约110万)本就落在 0.3~2m 障碍带内, 结构是有的;
# 但 0.10m 分辨率下 occ=1 -> 障碍 5.2%, occ=5 -> 2.6%, occ=40 -> 0.5%。
# 调高这个阈值删的是**真实结构**, 户外扫描本就稀疏, 越调图越空。
#
# "大片地面变黑"是另一个问题(坡地上地面被判成障碍), 已由局部地面网格
# (local_ground / ground_cell)解决, 不要再拿这个阈值去压。
OUTDOOR_OCC_PTS = 5

# 户外出图时只保留离建图轨迹这么远(米)以内的点。
# MID360 的 det_range 是 100m, 空旷处能收到 30~40m 外楼房的回波, 包围盒被撑到
# 77x80m, 而那些远点是掠射来的 —— 又稀又不可靠(2026-09-15 实测: 包围盒
# 73.7x77.6m, 自由区仅 4.6%)。裁掉让地图聚焦在真正走过的范围。
# 注意这只影响**出图**; 远点照常参与 FAST-LIO 里程计, 空旷草坪上恰恰要靠远处
# 楼房做几何约束, 所以不能改 det_range。
# 2026-09-15 实测(一次 24m 直线来回的户外建图, 0.05m 分辨率)扫描结果:
#   r=20 -> 自由 22.1%, 原始自由格 161479   r=15 -> 29.2%, 154152
#   r=12 -> 34.7%, 149664                  r=8  -> 40.1%, 127306
# 看**绝对自由格**(= 真正观测到的可通行面积): 20->12 只多丢 7%, 12->8 一下丢 15%
# —— 拐点在 12 和 8 之间, 所以 12~15 都在安全区。取 15: 比 20 密得多, 又比 12
# 保守, 对形状不同的路线更稳。注意这组数是在"直线型轨迹"上调的; 绕环建图时
# 环内本来就被覆盖, 半径的影响会小很多。
OUTDOOR_TRAJ_CROP = 15.0
MAP_DIR = os.path.join(HOME, 'wheeltec_ros2/src/wheeltec_robot_nav2/map')
STATIC_DIR = os.path.join(os.path.dirname(__file__), 'static')

# 二进制消息类型 (server->client, 首字节)
B_MAP, B_SCAN, B_CLOUD, B_PLAN, B_TRAJ, B_COSTMAP, B_MAP_EDIT = 1, 2, 3, 4, 5, 6, 7
B_LOCAL_PLAN = 8            # 局部规划路径 (/local_plan, 控制器/MPPI 输出)
B_CAM = 9                   # Astra 彩色相机画面 (JPEG, 前端要了才推)
B_GPS = 10                  # GPS 数据 (NavSatFix)

# 网页实时画面: 推流帧率/宽度/JPEG质量 (见 _cam_cb 里的权衡说明)
CAM_FPS, CAM_W, CAM_Q = 12, 512, 70

# 【实景模式】点云推送帧率与每帧点数。5Hz x 2500点 x 12字节 ≈ 150KB/s,
# 和相机流(约300KB/s)加起来还在 2.4G 热点扛得住的范围。嫌卡降 SCENE_HZ,
# 嫌稀降 SCENE_HZ 提 SCENE_PTS —— 带宽是两者的乘积。
SCENE_HZ, SCENE_PTS = 5.0, 2500

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
    'outdoor': {
        'label': 'MID360s 户外',
        'package': 'wheeltec_outdoor_nav',
        'launches': {'mapping': 'outdoor_mapping.launch.py', 'save_map': None,
                     'navigation': 'outdoor_navigation.launch.py', 'lidar_test': None},
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


# ---------- 回充点的持久化 ----------
# 与巡航航点同构: 按"传感器桶/地图名"存每张图上的回充点(单点 {x,y,yaw})。
# yaw 是"车到达回充点时应朝向"的朝向(即正对二维码), 让 Nav2 尽量正对停靠,
# 减少到达后 SEARCH_TAG 角度步进搜索的耗时。
DOCK_FILE = os.path.join(HOME, '.wheeltec', 'webapp_dock.json')


def _load_dock():
    try:
        with open(DOCK_FILE) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def _write_dock(d):
    try:
        os.makedirs(os.path.dirname(DOCK_FILE), exist_ok=True)
        tmp = DOCK_FILE + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(d, f, ensure_ascii=False)
        os.replace(tmp, DOCK_FILE)   # 原子替换
        return True
    except OSError:
        return False


def get_dock_point(sensor, name):
    """返回 {'x','y','yaw'} 或 None。"""
    v = _load_dock().get(_route_key(sensor, name))
    if not isinstance(v, dict) or 'x' not in v or 'y' not in v:
        return None
    try:
        return {'x': float(v['x']), 'y': float(v['y']),
                'yaw': float(v.get('yaw', 0.0))}
    except (TypeError, ValueError):
        return None


def save_dock_point(sensor, name, x, y, yaw):
    d = _load_dock()
    key = _route_key(sensor, name)
    d[key] = {'x': float(x), 'y': float(y), 'yaw': float(yaw)}
    return _write_dock(d)


def delete_dock_point(sensor, name):
    d = _load_dock()
    if d.pop(_route_key(sensor, name), None) is not None:
        _write_dock(d)


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
# ekf_node / gps_map_odom 是 2026-09-17 加的: 户外 gps 模式新增的全局 EKF
# 若残留, 会一直发 map->odom_combined 和下次启动的定位抢 TF。
EXT_PATTERN = ('component_container_isolated|livox_ros_driver2_node|'
               'wheeltec_robot_node|fastlio_mapping|lslidar_driver_node|'
               'async_slam_toolbox_node|host_sdk_sample|astra_camera_node|'
               'nav2_waypoint_cycle|dual_rtk_driver|ekf_node|gps_map_odom')
# astra_camera_node 是 2026-09-15 补的: mapping/navigation.launch.py 的防重复
# 启动一直在查它, 这里却漏了。残留的相机节点对 webapp 隐形 -> "停止"按钮不亮、
# 启动前的拦截也不触发, 用户在界面上完全无法自救, 只能 SSH 进去 pkill。
# 两边的进程名清单必须保持一致, 加新节点时记得同步。
_LAUNCH_PKG_PATTERN = 'wheeltec_fastlio|wheeltec_n10plus|wheeltec_odin1|wheeltec_outdoor_nav'

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
    sta = {'name': '', 'ip': '', 'iface': '', 'up': False, 'signal': None}
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
    # 站点模式的信号强度(0-100), 供网页用格数图标显示。
    # 走 `nmcli -f IN-USE,SIGNAL dev wifi` 只取带 * 的那一行 —— 注意这条命令在
    # 某些驱动上会触发一次扫描而变慢, 所以放在 2 秒一次的状态轮询里, 且带 timeout。
    if sta['up']:
        try:
            r = subprocess.run(['nmcli', '-t', '-f', 'IN-USE,SIGNAL', 'dev', 'wifi'],
                               capture_output=True, text=True, timeout=4)
            for ln in r.stdout.split('\n'):
                f = ln.split(':')
                if len(f) >= 2 and f[0].strip() == '*':
                    sta['signal'] = int(f[1])
                    break
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
            'w': w, 'h': h,
            # 编辑页要显示"正在编哪张"、保存时也要回传, 名字得跟着图一起走
            'name': os.path.splitext(os.path.basename(pgm_path))[0]}
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


MAP_BACKUP_KEEP = 5          # 每张图最多留几份历史, 32GB 小盘经不起无限攒


def backup_map(name):
    """覆盖前把 install 目录里的旧图存一份到 map/_backup/。

    编辑是不可逆的 —— 撤销栈只活在浏览器里, 页面一关就没了, 手滑抹掉一堵墙
    没有别的退路。备份放子目录而不是同级改名: list_2d_maps() 是按同级目录里
    "<x>.yaml + <x>.pgm 配对"列图的, 备份放同级会混进地图库列表。
    返回备份文件名(没有可备份的返回 None)。"""
    pgm = os.path.join(MAP_DIR_INSTALL, name + '.pgm')
    yml = os.path.join(MAP_DIR_INSTALL, name + '.yaml')
    if not (os.path.exists(pgm) and os.path.exists(yml)):
        return None                       # 新名字, 没有旧图要备份
    bdir = os.path.join(MAP_DIR_INSTALL, '_backup')
    os.makedirs(bdir, exist_ok=True)
    tag = f'{name}-{time.strftime("%m%d-%H%M%S")}'
    shutil.copy2(pgm, os.path.join(bdir, tag + '.pgm'))
    shutil.copy2(yml, os.path.join(bdir, tag + '.yaml'))
    old = sorted(glob.glob(os.path.join(bdir, name + '-*.pgm')))
    for p in old[:-MAP_BACKUP_KEEP]:      # 只留最近几份
        for q in (p, p[:-4] + '.yaml'):
            try:
                os.remove(q)
            except OSError:
                pass
    return tag + '.pgm'


def pack_map(meta_json, png, btype=B_MAP):
    return bytes([btype]) + struct.pack('<I', len(meta_json)) + meta_json + png


def costmap_to_png(msg: OccupancyGrid, tf_map=None):
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
    # 局部代价地图发布在 **odom_combined** 系(param_S100_diff.yaml 里
    # local_costmap.global_frame 已改成它, 为了避免AMCL修正位姿后旧障碍标记
    # 落到车身底下)。而网页画布是 map 系, 所以这里必须把栅格原点变换过去,
    # 并把两系之间的偏航角一起传给前端 —— odom 和 map 之间不只有平移还有转角,
    # 只平移不转的话地图一转弯就整片错开(这就是"局部代价地图对不上"的原因)。
    ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
    yaw = 0.0
    if tf_map is not None:
        t = tf_map.transform.translation
        q = tf_map.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, sn = math.cos(yaw), math.sin(yaw)
        ox, oy = t.x + c * ox - sn * oy, t.y + sn * ox + c * oy
    meta = {'res': msg.info.resolution, 'ox': ox, 'oy': oy,
            'yaw': yaw, 'w': w, 'h': h}
    return json.dumps(meta).encode(), buf.getvalue()


def pack_floats(btype, arr):
    return bytes([btype]) + np.asarray(arr, dtype='<f4').tobytes()


def cloud_xyz(msg: PointCloud2, max_pts=1200, clip=True):
    """手动解析 PointCloud2 的 x,y,z (无 sensor_msgs_py 依赖), 均匀抽稀。

    clip=False 时不做 z 值裁剪 —— 地面拟合需要原始分布, 而 camera_init 是斜的,
    按 z 裁会把地面自己裁掉一部分(见 ground_plane 的说明)。
    """
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
    good = np.isfinite(out).all(axis=1)
    if clip:
        good &= (out[:, 2] > -1.5) & (out[:, 2] < 3.0)
    return out[good]


# 雷达安装角的**标称值**, 与 wheeltec_fastlio 两个 launch 的 lidar_pitch/roll/z 对应。
# 只当作地面拟合的初值和兜底 —— 真正用于显示的是 GroundFitter 实测出来的平面。
# (2026-08-07 实测: launch 里写的 0.191rad 与实际差了一倍, 靠死数是靠不住的。)
NOMINAL_PITCH, NOMINAL_ROLL, NOMINAL_Z = 0.191, 0.006, 0.28


def nominal_up(pitch=NOMINAL_PITCH, roll=NOMINAL_ROLL):
    """按安装角推出的地面法向在 camera_init 系里的方向。

    FAST-LIO 的 camera_init **不是重力对齐的**(IMU_Processing 初始化时 rot=I),
    世界系直接取开机瞬间的 IMU 姿态, 雷达怎么装这个系就怎么歪。
    base_footprint->livox 的旋转是 Ry(pitch)·Rx(roll)(pitch>0=下倾), 所以
    地面法向 (0,0,1) 落到雷达系是 Rx(-roll)·Ry(-pitch)·ẑ = (-sinθ, sinφcosθ, cosφcosθ)。
    **x 分量是负的** —— 2026-08-07 用 IMU 重力法实测 (-0.0998, +0.0168, +0.9949) 证实,
    与 pcd2pgm.cpp 的约定一致。webapp 之前这里取了反号, 结果把倾角**加倍**了。
    """
    sp, cp = math.sin(pitch), math.cos(pitch)
    sr, cr = math.sin(roll), math.cos(roll)
    return np.array([-sp, sr * cp, cr * cp], dtype=np.float64)


class GroundFitter:
    """实时估计"地面在 camera_init 里是哪个平面", 供网页把 3D 视图校平。

    【为什么不能用 launch 里的 lidar_pitch/roll 了事】那是死数, 而且实测会过期:
    2026-08-07 launch 写的是 0.191rad(10.94°), 而 IMU 重力法和点云拟合一致给出
    约 0.097rad(5.6°) —— 差了一倍。倾角错 5°, 网格和地面在 4m 外就差 35cm,
    "网格就是地面"这句话直接不成立。

    【两个量分开估, 各用各最靠谱的手段】

    1) **法向**: 取 /livox/imu 的加速度(静止时=重力反方向, 即"上"), 用 /Odometry
       的姿态转到 camera_init。FAST-LIO 的 body 系就是 IMU 系, 所以直接乘就行。
       实测 0.1 秒就稳定到 0.013°, 35 秒内漂移 0.000°, 人为加 ±0.05g 的运动噪声
       也只动 0.01°。**镜面地面上一个地面点都收不到时它照样准**, 这是关键。
       运动加速度是零均值的, 长时间 EMA 自然滤掉; 幅值明显偏离 1g 的样本直接丢。

    2) **高度**: 沿上面那个法向投影, 在雷达下方 [-1.5, -0.05]m 做直方图, 取
       **最低的那个强峰**当地面(不是最高峰 —— 地面是最低的那块大平面, 取最低强峰
       才不会被贴地的矮台面顶掉)。实测 2 秒(10帧)就收敛到 0.2697m, 88 秒后
       0.2685m, 全程抖动 ±1mm。

    ⚠️ **不要退回"直接对点云做 RANSAC 平面拟合"**: /cloud_registered 是
    filter_size_surf=0.5 体素降采样过的, 每帧只有 600 点左右, 而且这块场地地面
    是镜面的(见 CLAUDE.md 5.6)。2026-08-07 试过, 用标称法向当种子时地面在直方图里
    整个糊开(峰锐度 1.85x), 迭代拟合被墙面带跑, 一次都没收敛。换成 IMU 法向之后
    同一批数据的峰锐度是 4.57x。

    估不出来就一直退回标称值, 界面上会显示"标称"提醒。
    """

    ACC_TC = 4.0          # 重力方向 EMA 时间常数(秒)
    MAX_PTS = 60000       # 高度估计用的点缓冲上限(约 100 帧, 够用且不涨内存)
    MIN_BIN = 120         # 直方图峰至少这么多点才认

    def __init__(self):
        self.n = nominal_up()          # 地面法向(camera_init 系, 指向上)
        self.z = NOMINAL_Z             # 雷达光心离地高度
        self.n_src = 'nom'             # 'imu' | 'nom'
        self.z_src = 'nom'             # 'cloud' | 'nom'
        self.inliers = 0
        self._acc = None               # 重力方向的 EMA(camera_init 系, 未归一)
        self._mag = None               # |acc| 的 EMA, 用来卡运动加速度
        self._acc_t = 0.0
        self._buf = []                 # [(N,3)] 相对雷达位置的点
        self._buf_n = 0
        self._last_fit = 0.0
        self._steady = 0               # 法向连续多少次没动
        self._samples = 0

    # ---------- 法向: IMU 重力 ----------
    def feed_imu(self, acc, R=None):
        """acc: (ax,ay,az) body 系; R: body->camera_init 的 3x3(取自 /Odometry)。"""
        a = np.asarray(acc, dtype=np.float64)
        m = float(np.linalg.norm(a))
        if not np.isfinite(m) or m < 1e-6:
            return
        # 幅值明显不是 1g 的样本 = 车在颠/在急加速, 丢掉。首个样本无条件收下当基准。
        if self._mag is None:
            self._mag = m
        elif abs(m - self._mag) > 0.25 * self._mag:
            return
        now = time.monotonic()
        dt = min(1.0, max(0.0, now - self._acc_t)) if self._acc_t else 1.0
        self._acc_t = now
        k = 1.0 - math.exp(-dt / self.ACC_TC)
        self._mag += (m - self._mag) * k
        v = a / m
        if R is not None:
            v = R @ v                  # body -> camera_init
        self._acc = v if self._acc is None else self._acc + (v - self._acc) * k
        u = self._acc / (np.linalg.norm(self._acc) or 1.0)
        # 和标称差 40° 以上多半是话题接错/车翻了, 宁可不用
        if float(u @ nominal_up()) > math.cos(math.radians(40)):
            # 收敛判据: 连续若干次方向几乎不动。实测 0.1 秒就到 0.013°, 这里给足余量。
            if self.n_src == 'imu' and float(np.dot(u, self.n)) > math.cos(
                    math.radians(0.02)):
                self._steady += 1
            else:
                self._steady = 0
            self.n, self.n_src = u, 'imu'
        self._samples += 1

    @property
    def normal_settled(self):
        """法向已经稳住了 —— 调用方可以把 IMU 采样降下来。

        地面法向在一次建图会话里是**常量**(camera_init 固定, 车在平地上跑),
        没必要一直按 20Hz 喂。实测 0.1 秒就稳定, 之后 35 秒漂移 0.000°。
        稳住之后降到 1Hz, 既能跟上"车被抬起来放到斜坡上"这种慢变化, 又几乎不花 CPU。
        """
        return self._steady >= 40 and self._samples >= 60

    # ---------- 高度: 沿法向的直方图最低强峰 ----------
    def feed_cloud(self, pts, sensor=None):
        """pts: (N,3) camera_init 系; sensor: 雷达当前位置(取自 /Odometry)。

        高度必须相对**雷达当前位置**量, 不能相对 camera_init 原点 —— 车开出去
        十几米后原点早就不在脚下了。
        """
        if pts is None or len(pts) < 50:
            return False
        q = np.asarray(pts, dtype=np.float64)
        if sensor is not None:
            q = q - np.asarray(sensor, dtype=np.float64)
        self._buf.append(q)
        self._buf_n += len(q)
        while self._buf_n > self.MAX_PTS and len(self._buf) > 1:
            self._buf_n -= len(self._buf.pop(0))
        now = time.monotonic()
        if now - self._last_fit < 2.0:
            return False
        self._last_fit = now
        return self._estimate_z(np.vstack(self._buf))

    def _estimate_z(self, Q):
        h = Q @ self.n
        m = (h >= -1.5) & (h < -0.05)          # 只在雷达下方找, 桌面/天花板不参与
        if int(m.sum()) < self.MIN_BIN:
            return False
        hist, ed = np.histogram(h[m], bins=58, range=(-1.5, -0.05))
        peak = int(hist.max())
        if peak < self.MIN_BIN:
            return False
        # 地面 = **最低**的强峰。取最高峰的话, 贴地的矮台面/床沿点更密就会顶掉地面。
        idx = int(np.flatnonzero(hist >= 0.7 * peak)[0])
        c = (ed[idx] + ed[idx + 1]) * 0.5
        sel = np.abs(h - c) < 0.06
        k = int(sel.sum())
        if k < self.MIN_BIN:
            return False
        z = -float(h[sel].mean())
        # 首次直接采用, 之后 EMA —— 高度是会话常量, 慢一点无所谓, 稳定更重要
        a = 1.0 if self.z_src != 'cloud' else 0.25
        self.z = self.z * (1 - a) + z * a
        self.z_src, self.inliers = 'cloud', k
        return True

    def reset(self):
        """新的建图会话 = 新的 camera_init, 之前估的平面完全作废。"""
        self.__init__()

    def as_dict(self):
        n = self.n
        return {'nx': round(float(n[0]), 5), 'ny': round(float(n[1]), 5),
                'nz': round(float(n[2]), 5), 'z': round(float(self.z), 4),
                # fit=True 表示法向已经是实测的(网格才真的贴地); 高度另有 zsrc
                'fit': self.n_src == 'imu', 'nsrc': self.n_src, 'zsrc': self.z_src,
                'inl': self.inliers,
                # 换算成人看得懂的安装角, 只用于界面显示
                'pitch': round(float(-math.atan2(n[0], n[2])), 4),
                'roll': round(float(math.atan2(n[1], n[2])), 4)}


class GpsMonitor:
    """户外模式下单独托管一个 RTK 驱动进程。

    为什么需要: RTK 驱动原本只在 outdoor_mapping/outdoor_navigation 这两个
    launch 里起。没跑任务时没人发 /gps/fix, 界面只能显示"无信号" —— 2026-09-14
    用户在户外反复以为是天线/信号问题, 实际只是驱动没启动。
    所以切到户外模式就把它拉起来, 随时能看到定位状态。

    与任务的冲突: 两个驱动抢同一个串口会互相偷字节。所以任务启动前先停掉本
    进程(launch 自带驱动), 任务退出后再拉回来。
    """

    CMD = ['ros2', 'run', 'wheeltec_dual_rtk_driver', 'dual_rtk_driver_node',
           '--ros-args', '-p', 'port:=/dev/wheeltec_gnss', '-p', 'baud:=115200']

    # 驱动的输出落到这里。别再用 DEVNULL —— 2026-09-16 驱动起来后又自己死掉,
    # 因为输出全被丢弃, 连 traceback 都没有, 只能看到"GPS 又没了"。
    LOG = os.path.join(HOME, '.ros', 'wheeltec_gps_monitor.log')
    LOG_MAX = 2 * 1024 * 1024        # 盘只剩 3G, 超过就截断重来

    # 自动重试: 户外模式下驱动意外退出, 隔 RETRY_MIN 秒再拉, 连续失败每次翻倍到
    # RETRY_MAX; 一次拉起后稳定跑满 60 秒就复位。
    RETRY_MIN = 10.0
    RETRY_MAX = 120.0

    def __init__(self):
        self.proc = None
        self.last_start = 0.0
        self.retry_s = self.RETRY_MIN

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def _open_log(self):
        os.makedirs(os.path.dirname(self.LOG), exist_ok=True)
        try:
            if os.path.getsize(self.LOG) > self.LOG_MAX:
                os.replace(self.LOG, self.LOG + '.1')
        except OSError:
            pass
        f = open(self.LOG, 'ab', buffering=0)
        f.write(('\n===== %s 启动 RTK 驱动 =====\n'
                 % time.strftime('%Y-%m-%d %H:%M:%S')).encode())
        return f

    def start(self):
        if self.alive():
            return
        self.last_start = time.time()
        f = None
        try:
            f = self._open_log()
            self.proc = subprocess.Popen(
                self.CMD, stdin=subprocess.DEVNULL,
                stdout=f, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid, close_fds=True)
        except Exception:
            # 启动失败原本也是静默的(直接 self.proc = None), 界面上只表现为
            # "GPS未启动", 没法区分"没启动"和"启动了但失败"
            self.proc = None
            try:
                import traceback
                with open(self.LOG, 'a', encoding='utf-8') as g:
                    g.write('启动失败:\n' + traceback.format_exc())
            except Exception:
                pass
        finally:
            if f is not None:
                f.close()    # 子进程已经继承了 fd, 父进程这份可以关

    def supervise(self, allowed):
        """周期调用: 该有驱动(allowed)却没在跑, 就按退避间隔再拉一次。

        以前驱动一退出就没人管: 2026-09-22 车刚上电、接收机还没搜到星时切到户外,
        驱动 15 秒拿不到定位退出了, 等搜到星也没人再拉, 界面一直"GPS未启动"。"""
        if self.alive():
            if time.time() - self.last_start > 60.0:
                self.retry_s = self.RETRY_MIN
            return
        if not allowed or time.time() - self.last_start < self.retry_s:
            return
        self.start()
        self.retry_s = min(self.retry_s * 2, self.RETRY_MAX)

    def stop(self):
        """先 SIGINT 让驱动自己收尾(它注册了 SIGINT handler), 收不掉再 SIGKILL。
        注意: UM982Serial 的读串口线程不是守护线程, 只发 SIGINT 有概率留下占着
        串口的僵尸进程(2026-09-14 实际发生过), 所以超时必须补 SIGKILL。"""
        if not self.alive():
            self.proc = None
            return
        pgid = os.getpgid(self.proc.pid)
        try:
            os.killpg(pgid, signal.SIGINT)
        except Exception:
            pass
        for _ in range(30):
            if self.proc.poll() is not None:
                break
            time.sleep(0.1)
        if self.proc.poll() is None:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass
        self.proc = None


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
        # 共享内存 + 回环 UDP, 与 webapp 的 localhost_only.xml 一致(只在本机通信)
        env['ROS_LOCALHOST_ONLY'] = '1'
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
        """本进程管理之外的 ROS 实例 pid 列表。

        "自己人"有两拨, 都要排掉:
          ① 当前任务那棵进程树(同一个进程组);
          ② webapp 自己托管的 GPS 监控(GpsMonitor setsid 过, 自成一组)。
        第②条是 2026-09-15 加的: 清单里补了 dual_rtk_driver 之后, 不排掉监控
        的话, 户外模式一开界面就常亮"有残留进程"。
        """
        r = subprocess.run(['pgrep', '-f', EXT_PATTERN],
                           capture_output=True, text=True)
        pids = [int(p) for p in r.stdout.split()]
        own_pgids = set()
        if self.alive():
            own_pgids.add(self.proc.pid)
        hook = getattr(self, 'own_pgid_hook', None)
        if hook:
            try:
                g = hook()
                if g:
                    own_pgids.add(g)
            except Exception:
                pass          # 钩子坏了不能拖垮残留检测
        if not own_pgids:
            return pids
        out = []
        for p in pids:
            try:
                if os.getpgid(p) not in own_pgids:
                    out.append(p)
            except OSError:
                pass          # 查不到 = 刚退出, 不算残留
        return out

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
        """清理被强杀进程残留的 FastDDS 共享内存/信号量文件。

        进程被 kill -9 或段错误退出后, /dev/shm 会残留已锁定的
        sem.fastrtps_portXXXX_mutex, 新启动的节点(尤其nav2容器)尝试加锁时
        会在 futex 上永久死锁(表现为100%CPU且无日志)。

        只删没有任何活进程在用的文件(2026-09-18 改)。原来按进程名猜"还有没有别的
        ROS 进程在跑", 名单总有漏的(RTK驱动/gps_map_odom/ros2 daemon/手敲的命令),
        会删掉活进程的文件使其通信断开; FastDDS 还靠 *_el 锁文件判断端口/段的主人
        是否活着, 锁文件被删, 后来的进程会把活端口当空闲占用, 两个进程共用一个端口。
        现在直接问内核: 被某个进程映射(/proc/*/maps)或打开(/proc/*/fd)就是在用。
        同一段/端口的几个文件(本体、_el 锁、sem.*_mutex)算一组, 组内有一个在用或
        10 秒内新建(防止和正在启动的进程赛跑), 整组保留。
        """
        import time
        groups = {}
        for f in (glob.glob('/dev/shm/fastrtps_*') +
                  glob.glob('/dev/shm/sem.fastrtps_*') +
                  glob.glob('/dev/shm/fast_datasharing*')):
            key = os.path.basename(f)
            if key.startswith('sem.'):
                key = key[4:]
            for suffix in ('_mutex', '_el', '_sl'):
                if key.endswith(suffix):
                    key = key[:-len(suffix)]
                    break
            groups.setdefault(key, []).append(f)
        if not groups:
            return
        used = set()  # 在用文件的 inode
        for pid in os.listdir('/proc'):
            if not pid.isdigit():
                continue
            try:
                with open('/proc/%s/maps' % pid) as fh:
                    for line in fh:
                        if ' /dev/shm/' in line:
                            used.add(line.split()[4])
                fds = os.listdir('/proc/%s/fd' % pid)
            except OSError:
                continue
            for fd in fds:
                link = '/proc/%s/fd/%s' % (pid, fd)
                try:
                    if os.readlink(link).startswith('/dev/shm/'):
                        used.add(str(os.stat(link).st_ino))
                except OSError:
                    pass
        now = time.time()
        for files in groups.values():
            try:
                stats = [os.stat(f) for f in files]
            except OSError:
                continue
            if any(str(s.st_ino) in used or now - max(s.st_mtime, s.st_ctime) < 10.0
                   for s in stats):
                continue
            for f in files:
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
        # 【实景模式】前端打开"实景"开关时置 True: 点云推送提速到 SCENE_HZ、
        # 每帧点数加到 SCENE_PTS, 让 3D 视图看起来是实时的而不是一秒一跳。
        # 关掉就退回 1Hz/1200 点(只够看建图覆盖), 省无线带宽。
        self.scene_want = False
        self.ground = GroundFitter()
        self._last_imu_t = 0.0
        # Astra 彩色画面: 只有前端把"实时画面"卡片打开(cam_want=True)才编码,
        # 否则连 JPEG 都不压——320x240@15 的原始帧走 DDS 已经进来了, 白压是浪费CPU
        self.cam_want = False
        self.cam_jpeg = None
        self.cam_seq = 0
        self._last_cam_t = 0.0

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
        # 订阅 /Laser_map 作为备用（FAST-LIO 在某些配置下只发布这个话题）
        self.create_subscription(PointCloud2, '/Laser_map',
                                 self.cloud_cb, sensor_qos)
        # 雷达内置 IMU: 只用来定"哪边是上"(见 GroundFitter)。200Hz 全收会白烧 CPU,
        # 回调里按 20Hz 抽。建图没起时这个话题根本不存在, 订阅着也没开销。
        self.create_subscription(Imu, '/livox/imu', self.imu_cb, sensor_qos)
        self.create_subscription(ImageMsg, '/camera/color/image_raw',
                                 self.image_cb, sensor_qos)
        # GPS 数据订阅 (NavSatFix)
        self.create_subscription(NavSatFix, '/gps/fix', self._cb('gps'), 10)
        self.create_subscription(UInt8, '/gps/quality', self._cb('gps_quality'), 10)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 2)
        # 【MPPI 速度上限的 workaround】见 handle_msg 里 nav_speed 的长注释:
        # 光改参数不生效, 还得往这个话题发一条"无限速", 逼 MPPI 把 base_constraints
        # 抄进真正生效的 constraints。QoS 要和 controller_server 的订阅一致(默认 10)。
        self.pub_speed_limit = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 2)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped,
                                              '/initialpose', 2)
        self.cli_reloc = self.create_client(Trigger, '/relocalize')
        # 停靠/泊出服务客户端 (tagdocking)。服务不在线时 call_async 会超时, 调用方
        # 用 service_is_ready() 先探一遍并给 toast 提示。
        self.cli_dock_start = self.create_client(
            Trigger, '/docking_node/start_docking')
        self.cli_dock_undock = self.create_client(
            Trigger, '/docking_node/start_undock')
        self.dock_state_seq = 0
        self.create_subscription(
            String, '/docking_node/state', self._cb_dock_state, 10)
        self.cli_loadmap = (self.create_client(LoadMap, '/map_server/load_map')
                            if LoadMap else None)
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

    def _cb_dock_state(self, msg):
        """停靠状态回调: 只在状态字符串真正变化时递增序号(触发推送)。

        /docking_node/state 是 20Hz 连续发布(终态如 timeout 也一直重复发),
        若每条都推给前端, 前端会在同一终态上反复弹 toast。只在状态跳变时推。
        """
        with self.lock:
            if self.latest.get('dock_state') == msg.data:
                return
            self.latest['dock_state'] = msg.data
            self.dock_state_seq += 1

    def param_client(self, node_fqn):
        """rcl_interfaces/SetParameters 客户端(懒加载复用): 用于运行时调整
        nav2 costmap/MPPI 参数(避障膨胀半径、导航速度上限), 仅本次导航会话有效,
        不写回yaml, 重启导航会恢复配置文件默认值。"""
        if node_fqn not in self._param_clients:
            self._param_clients[node_fqn] = self.create_client(
                SetParameters, f'{node_fqn}/set_parameters')
        return self._param_clients[node_fqn]

    def get_param_client(self, node_fqn):
        """同上, 但是读参数用的 GetParameters 客户端。"""
        key = node_fqn + '/get'
        if key not in self._param_clients:
            self._param_clients[key] = self.create_client(
                GetParameters, f'{node_fqn}/get_parameters')
        return self._param_clients[key]

    def _odom_pose(self):
        """(位置, body->camera_init 旋转矩阵); 没有**新鲜**里程计就 (None, None)。

        必须卡新鲜度: /Odometry 是 FAST-LIO 发的, 导航模式下根本没有这个话题,
        而 latest['odom'] 会一直留着上一次建图会话的最后一帧 —— 拿那个陈旧姿态去
        转 IMU 重力, 算出来的"上"是错的。
        取不到时按单位阵处理, 恰好也是对的: 建图刚起、还没有第一帧里程计时,
        camera_init 就等于 body 系。
        """
        if time.monotonic() - getattr(self, 'odom_mono', 0) > 2.0:
            return None, None
        with self.lock:
            od = self.latest.get('odom')
        if od is None:
            return None, None
        p = od.pose.pose.position
        q = od.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return np.array([p.x, p.y, p.z]), R

    def imu_cb(self, msg):
        # /livox/imu 是 200Hz。刚起来时按 20Hz 喂, 法向稳住之后降到 1Hz ——
        # 地面法向是会话常量, 稳住了就没必要一直算(见 normal_settled)。
        now = time.monotonic()
        period = 1.0 if self.ground.normal_settled else 0.05
        if now - self._last_imu_t < period:
            return
        self._last_imu_t = now
        try:
            _, R = self._odom_pose()
            a = msg.linear_acceleration
            self.ground.feed_imu((a.x, a.y, a.z), R)
        except Exception:
            pass                                 # 估不出来就退回标称值, 不能影响推流

    def cloud_cb(self, msg):
        now = time.monotonic()
        hz = SCENE_HZ if self.scene_want else 1.0    # 1Hz 只够看建图覆盖
        if now - self._last_cloud_t < 1.0 / hz:
            return
        self._last_cloud_t = now
        # 地面高度估计吃的是**未裁剪**的点: camera_init 是斜的, 按原始 z 裁会把地面
        # 自己削掉一角, 估出来的高度就偏了
        raw = cloud_xyz(msg, max_pts=4000, clip=False)
        if raw is not None and len(raw):
            try:
                pos, _ = self._odom_pose()
                self.ground.feed_cloud(raw, pos)
            except Exception:
                pass
        pts = cloud_xyz(msg, max_pts=SCENE_PTS if self.scene_want else 1200)
        if pts is not None and len(pts):
            with self.lock:
                self.cloud_out = pts

    def image_cb(self, msg):
        """Astra 彩色帧 -> JPEG。用 PIL 而不是 cv2: 进程里已经有 PIL(画地图用),
        再拉 cv2 光 import 就多占 100MB+ 内存, 本机只有 4GB。"""
        if not self.cam_want:
            return
        # 【分辨率与流畅度的权衡】相机采集 640x480@30。
        # 直接按原分辨率满帧推, 走 2.4G 热点带宽吃不消(约 1.2MB/s);
        # 原来这里写死 5Hz, 结果无论下游怎么改都只有 4~5fps(2026-08-07 用户反馈)。
        # 折中: 缩到 512x384 + 质量70 + 12Hz ≈ 300KB/s —— 比原来的 320x240 更清晰,
        # 帧率是原来的近 3 倍。嫌卡就降 CAM_FPS, 嫌糊就调 CAM_W。
        now = time.monotonic()
        if now - self._last_cam_t < 1.0 / CAM_FPS:
            return
        self._last_cam_t = now
        try:
            raw = bytes(msg.data)
            if msg.encoding in ('rgb8', 'bgr8'):
                im = Image.frombytes('RGB', (msg.width, msg.height), raw)
                if msg.encoding == 'bgr8':
                    b, g, r = im.split()
                    im = Image.merge('RGB', (r, g, b))
            elif msg.encoding in ('mono8', '8UC1'):
                im = Image.frombytes('L', (msg.width, msg.height), raw)
            else:
                return
            if im.width > CAM_W:      # 等比缩放, BILINEAR 比 LANCZOS 省不少 CPU
                im = im.resize((CAM_W, round(im.height * CAM_W / im.width)),
                               Image.BILINEAR)
            buf = io.BytesIO()
            im.save(buf, format='JPEG', quality=CAM_Q)
            with self.lock:
                self.cam_jpeg = buf.getvalue()
                self.cam_seq += 1
        except Exception:
            pass

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

    def pose_at(self, stamp):
        """取指定时刻的车体位姿(map<-base_footprint)。查不到就退回最近可用的一帧。

        用于把同一时刻的 /scan 摆到地图上 —— 用最新位姿会有一帧的角度误差,
        车旋转时表现为激光整体偏转。"""
        for parent in ('map', 'odom_combined'):
            for t in (stamp, rclpy.time.Time()):
                try:
                    tf = self.tf_buf.lookup_transform(parent, 'base_footprint', t)
                    tr, q = tf.transform.translation, tf.transform.rotation
                    return {'x': tr.x, 'y': tr.y, 'yaw': quat_to_yaw(q)}
                except Exception:
                    continue
        return None

    def robot_pose(self):
        """建图时(FAST-LIO /Odometry 在流)必须用它——与点云/轨迹同在 camera_init 系;
        导航时优先 map->base_footprint, AMCL未定位则退回 EKF 里程计。"""
        with self.lock:
            odom = self.latest.get('odom')
            fresh = time.monotonic() - getattr(self, 'odom_mono', 0) < 2.0
        if odom is not None and fresh:
            p = odom.pose.pose
            return {'x': p.position.x, 'y': p.position.y, 'z': p.position.z,
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
        self.cam_seq_sent = -1       # 已推给前端的相机帧序号
        self.dock_seq_sent = -1      # 已推给前端的停靠状态序号
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
        # 新版双模式界面(2026-08-06 起开发中), 挂在 /v2。
        # 单独一个文件而不是原地改: 改造涉及整页重构(双模式框架、导航模式浮层、
        # 建图模式的3D点云), 中途必然有不可用的状态, 而这台车的控制台要一直能用。
        # 验收通过后再把 / 指过去。两个页面说的是同一套 WS 协议, 后端不用分叉。
        self._v2_path = os.path.join(STATIC_DIR, 'v2.html')
        self._v2_cache = None

    def _cached_file(self, path, cache_attr):
        """按 mtime 热重载静态页面(见 index_html 的说明)。"""
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return None
        cache = getattr(self, cache_attr)
        if cache is None or cache[0] != mtime:
            with open(path, 'rb') as f:
                setattr(self, cache_attr, (mtime, f.read()))
        return getattr(self, cache_attr)[1]

    @property
    def v2_html(self):
        return self._cached_file(self._v2_path, '_v2_cache')

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
        # 任务结束, 户外模式下把常驻 RTK 驱动拉回来
        if self.gps_mon_want:
            self.gps_mon.start()
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

    def _set_static_map(self, frame):
        """把一张静态图(保存后/换图预览/编辑保存)设为当前底图并推给前端。

        缓存序号记成"已收到的最后一帧 /map"的序号: 之后只有真来了新 /map(如启动导航后
        map_server 发的)才会替换它。以前记成 -2, 推送循环见"序号不等"就把内存里残留的
        上一次导航的 /map 重推一遍, 0.2 秒内盖掉刚推的图 —— 表现为保存完/换地图后地图
        "加载不出来", 只在本次 webapp 跑过导航之后出现(2026-09-22)。"""
        with self.node.lock:
            seq = self.node.map_seq
        self.map_cache, self.map_cache_seq = frame, seq
        self._send_all(frame)

    def _push_saved_map(self, name='WHEELTEC3D'):
        pgm = os.path.join(MAP_DIR_INSTALL, name + '.pgm')
        yml = os.path.join(MAP_DIR_INSTALL, name + '.yaml')
        if os.path.exists(pgm) and os.path.exists(yml):
            try:
                meta, png = pgm_to_png(pgm, yml)
                self._set_static_map(pack_map(meta, png))
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
        try:
            st['uptime'] = int(float(open('/proc/uptime').read().split()[0]))
        except Exception:
            pass
        # CPU 占用: /proc/stat 是**累计值**, 必须和上一次采样做差, 直接读是没有意义的
        try:
            f = [int(x) for x in open('/proc/stat').readline().split()[1:8]]
            total, idle = sum(f), f[3] + f[4]
            prev = getattr(self, '_cpu_prev', None)
            self._cpu_prev = (total, idle)
            if prev and total > prev[0]:
                st['cpu_pct'] = round(
                    100.0 * (1 - (idle - prev[1]) / (total - prev[0])), 1)
        except Exception:
            pass
        st['now'] = time.time()      # 小车的系统时间(不是浏览器的)
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
                    # 前端据此判断后端是不是新版: 静态页按 mtime 热重载, 而 app.py
                    # 不会 —— 只 colcon build 没重启服务时, 新前端会配着旧后端跑,
                    # "保存前自动备份/另存为/热加载"三样都还没有。有这个标志就不会
                    # 在对话框里承诺一个后端做不到的事。
                    caps=['map_edit_backup', 'map_edit_saveas', 'map_reload',
                          'pcd_confirm', 'gps_monitor', 'save_occ_pts'],
                    # gps_pub: 当前有几个节点在发 /gps/fix。用来区分"驱动没启动"
                    # 和"驱动在跑但搜不到星" —— 以前两种都显示"GPS无信号", 害人。
                    gps_pub=self.node.count_publishers('/gps/fix'),
                    gps_mon=self.gps_mon.alive(),
                    running=self.mgr.alive(), stopping=self.mgr.stopping,
                    external=self.ext_running and not self.mgr.alive(),
                    voltage=round(v.data, 1) if v else None,
                    pcd_count=len(glob.glob(os.path.join(PCD_DIR, 'scans*.pcd'))),
                    has_map=self.map_cache is not None,
                    ground=self.node.ground.as_dict(),
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
            # 户外模式、没有任务(自己的或外部的, 它们自带驱动)时, 保证 RTK 驱动在跑
            self.gps_mon.supervise(self.gps_mon_want and not self.mgr.alive()
                                   and not self.ext_running)
            if self.clients:
                self.push_state()
            await asyncio.sleep(2.0)

    async def task_cam(self):
        """相机单独一路推流。

        原来它挤在 task_fast 里, 而那个循环是 0.2s 一轮 —— 相机再怎么调分辨率和
        帧率, 网页上也只有 5fps, 看起来就是"模糊又卡顿"(2026-08-07 用户反馈)。
        拆出来按 15Hz 推; 前端没要画面时(cam_want=False)只是空转, 开销可忽略。
        """
        while True:
            if self.clients and self.node.cam_want:
                with self.node.lock:
                    cseq, jpg = self.node.cam_seq, self.node.cam_jpeg
                if jpg is not None and cseq != self.cam_seq_sent:
                    self.cam_seq_sent = cseq
                    self._send_all(bytes([B_CAM]) + jpg)
            await asyncio.sleep(1 / (CAM_FPS * 1.5))

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
                    dstate = self.node.latest.get('dock_state')
                    dseq = self.node.dock_state_seq
                    gps = self.node.latest.get('gps')
                if dstate is not None and dseq != self.dock_seq_sent:
                    self.dock_seq_sent = dseq
                    self.send_json(t='dock_state', state=dstate)
                if scan is not None and tick % 2 == 0 and \
                        scan.header.stamp != self.last_scan_stamp:
                    self.last_scan_stamp = scan.header.stamp
                    # /scan 的点是 base_footprint 系的, 前端要把它摆到地图上就得
                    # 乘一个车体位姿。**不能用"最新位姿"** —— 位姿5Hz、扫描4Hz,
                    # 两者各推各的, 车一转起来就会用错一帧的角度去摆这一帧的点,
                    # 看起来就是激光整体持续偏转(2026-08-07 用户反馈"一直向右转")。
                    # 这里按**这帧扫描自己的时间戳**查 TF, 和点一起发下去。
                    sp = self.node.pose_at(scan.header.stamp)
                    if sp:
                        self.send_json(t='scanpose', **sp)
                    self._send_all(pack_floats(B_SCAN, self._scan_xy(scan)))
                if plan is not None and \
                        plan.header.stamp != self.last_plan_stamp:
                    self.last_plan_stamp = plan.header.stamp
                    pts = self._path_xy_in_map(plan)
                    step = max(1, len(pts) // 300)
                    self._send_all(pack_floats(B_PLAN, pts[::step]))
                if lplan is not None and \
                        lplan.header.stamp != self.last_local_plan_stamp:
                    self.last_local_plan_stamp = lplan.header.stamp
                    lpts = self._path_xy_in_map(lplan)
                    lstep = max(1, len(lpts) // 300)
                    self._send_all(pack_floats(B_LOCAL_PLAN, lpts[::lstep]))
                if mmsg is not None and mseq != self.map_cache_seq:
                    self.map_cache_seq = mseq
                    meta, png = grid_to_png(mmsg)
                    self.map_cache = pack_map(meta, png)
                    self._send_all(self.map_cache)
                if cmmsg is not None and cmseq != self.costmap_cache_seq:
                    self.costmap_cache_seq = cmseq
                    # 取 map <- 代价地图所在系 的变换(通常是 odom_combined)。
                    # 取不到就按恒等处理(退化成旧行为), 不至于整块图不显示。
                    tf_map = None
                    src_frame = cmmsg.header.frame_id or 'odom_combined'
                    if src_frame not in ('map', ''):
                        try:
                            tf_map = self.node.tf_buf.lookup_transform(
                                'map', src_frame, rclpy.time.Time())
                        except Exception:
                            tf_map = None
                    meta, png = costmap_to_png(cmmsg, tf_map)
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
                # GPS 数据推送 (约 2Hz)
                if gps is not None and tick % 3 == 0:
                    self.send_json(
                        t='gps',
                        lat=gps.latitude,
                        lon=gps.longitude,
                        alt=gps.altitude,
                        status=gps.status.status,
                        service=gps.status.service,
                        cov=list(gps.position_covariance[:3]),  # 只发经纬度和高度的协方差
                        # 原始GGA质量位, 界面据此显示准确的定位类型
                        quality=getattr(
                            self.node.latest.get('gps_quality'), 'data', None)
                    )
            tick += 1
            await asyncio.sleep(0.2)

    def _path_xy_in_map(self, path):
        """把 nav_msgs/Path 的点统一转到 map 系再发给前端。

        **不能直接用原始坐标**: /plan 由 planner 发在 map 系, 但 MPPI 的
        /transformed_global_plan(网页"局部路径"的来源)发在**局部代价地图的 frame**,
        而那个已经从 map 改成了 odom_combined(param_S100_diff.yaml, 为避免 AMCL
        修正位姿后旧障碍标记落到车身底下)。两系之间既有平移也有偏航, 直接当 map 系
        画的话局部路径就整条错位 —— 这和局部代价地图错位是同一个根因。
        按消息自带的 header.frame_id 查变换, 不写死系名, 以后再改 global_frame
        也不用动这里。
        """
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        src = path.header.frame_id
        if not pts or not src or src == 'map':
            return pts
        try:
            tf = self.node.tf_buf.lookup_transform('map', src, rclpy.time.Time())
        except Exception:
            return pts          # 查不到就按原样发, 总比整条不显示强
        t = tf.transform.translation
        yaw = quat_to_yaw(tf.transform.rotation)
        c, sn = math.cos(yaw), math.sin(yaw)
        return [(t.x + c * x - sn * y, t.y + sn * x + c * y) for x, y in pts]

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
        elif t == 'cam':
            # 前端开/关"实时画面"卡片。关掉就彻底不编码, 省CPU和无线带宽。
            self.node.cam_want = bool(msg.get('on'))
        elif t == 'scene':
            # 前端开/关建图页的"实景"开关: 只影响点云推送的帧率和密度
            self.node.scene_want = bool(msg.get('on'))
        elif t == 'gps_monitor':
            # 切到户外模式就常驻一个 RTK 驱动, 切回室内就停掉。
            # 有任务在跑时不启动 —— launch 自带驱动, 两个抢串口会互相偷字节。
            self.gps_mon_want = bool(msg.get('on'))
            if self.gps_mon_want:
                if not self.mgr.alive():
                    self.gps_mon.start()
            else:
                self.gps_mon.stop()
        elif t == 'cmd_stop':
            self.cmd_active = False
            self.node.send_vel(0.0, 0.0)
        elif t == 'start':
            # launch 自带 RTK 驱动, 先让开串口
            self.gps_mon.stop()
            sensor = msg.get('sensor', 'mid360')
            mode = msg.get('mode')
            map_name = (msg.get('map_name') or '').strip()
            outdoor = msg.get('outdoor', False)  # 户外模式标志

            # 任何新任务启动都清掉上一次会话的轨迹/点云/路径残留(建图轨迹是
            # camera_init系, 导航路径是map系, 混杂展示在新地图上容易让人误解)
            self.traj.clear()
            self.send_json(t='clear', what=mode)
            if mode == 'mapping':
                self.map_saved_this_session = False
                # 新会话 = 新的 camera_init(FAST-LIO 每次重新初始化世界系),
                # 上一次拟合的地面平面完全作废, 必须重来
                self.node.ground.reset()

            launch_args = {}
            # 建图: 只有前端弹窗确认过"清理上次PCD分段"才带这个参数。
            # 不带的话 launch 里遇到遗留分段会直接拒绝启动, 避免新旧点云混成废图。
            if mode == 'mapping' and msg.get('clear_pcd'):
                launch_args['clear_pcd'] = 'true'
            if map_name and not _safe_map_name(map_name):
                self.send_json(t='toast', ok=False,
                    msg='地图名不合法(只能中英文/数字/下划线/中划线, 1-64字符)')
                return

            # 户外模式：使用 wheeltec_outdoor_nav 功能包
            if outdoor and mode in ('mapping', 'navigation'):
                sensor = 'outdoor'  # 使用特殊标识，后面会映射到 outdoor 配置

            if mode == 'save_map':          # 只有mid360有这个launch模式
                self.pending_map_name = map_name or 'WHEELTEC3D'
                launch_args['map_name'] = self.pending_map_name
                # 户外: 障碍带提到 2.0m, 把楼房/树木纳入地图。必须和
                # outdoor_navigation 里 /scan 的 scan_max_z 保持一致。
                if outdoor:
                    launch_args['max_z'] = str(OUTDOOR_MAX_Z)
                    launch_args['min_z'] = str(OUTDOOR_MIN_Z)
                    # 散点过滤强度由前端逐次给; 旧前端不带这个字段, 退回默认。
                    # 必须夹范围: 传 0 会让所有非空格子都变障碍, 传太大真实
                    # 墙面也被滤掉, 两头都是废图, 不能让前端字段直接下发。
                    try:
                        occ = int(msg.get('occ_pts') or OUTDOOR_OCC_PTS)
                    except (TypeError, ValueError):
                        occ = OUTDOOR_OCC_PTS
                    launch_args['occupied_min_points'] = str(min(max(occ, 1), 50))
                    launch_args['traj_crop_radius'] = str(OUTDOOR_TRAJ_CROP)
                    # 存完图顺手做 GPS 地理配准, 结果绑定到这张地图。
                    # 数据不合格时 georef_map.py 会拒绝写文件, 结果是
                    # "没有标定"(守卫不激活)而不是"错的标定", 所以可以常开。
                    launch_args['georef'] = 'true'
            elif mode == 'navigation' and sensor in ('mid360', 'n10plus', 'outdoor'):
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
        elif t == 'dock':
            # 回充: 导航到回充点 → 到达后触发停靠
            await self._start_recharge(
                msg.get('sensor', 'mid360'), msg.get('name', ''))
        elif t == 'undock':
            await self._call_undock()
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
            vn = max(0.05, min(1.0, abs(float(msg.get('vx_min', -vx)))))
            wz = max(0.1, min(2.5, float(msg['wz_max'])))
            ok, err = await self._set_params('/controller_server', {
                'FollowPath.vx_max': vx, 'FollowPath.vx_min': -vn,
                'FollowPath.wz_max': wz})
            # 【为什么还要再发一条 speed_limit —— 这是"改了参数车速纹丝不动"的根因】
            # humble 版 nav2_mppi_controller 的动态参数只写进 settings_.base_constraints,
            # 而真正裁剪输出速度的是 settings_.constraints(optimizer.cpp 的
            # xt::clip(..., s.constraints.vx_min, s.constraints.vx_max))。
            # 两者只在 getParams() 里同步过一次(`s.constraints = s.base_constraints;`),
            # 参数变更后挂的 post callback 是 reset(), **它不做这个拷贝**。
            # 所以改完参数 base 变了、constraints 没变 = 完全不生效。
            # 而且 ParametersHandler::dynamicParamsCallback 无论如何都返回
            # successful=true, 连报错都没有, 界面还显示"已下发"。
            #
            # 唯一会做这个拷贝的是 Optimizer::setSpeedLimit(NO_SPEED_LIMIT):
            #   s.constraints.vx_max = s.base_constraints.vx_max; ...
            # 而 ControllerServer 正是通过 speed_limit 话题调它。所以这里补发一条
            # "无限速"(speed_limit=0.0 即 NO_SPEED_LIMIT), 把新值刷进 constraints。
            # 这样不用改 nav2 源码、不用重编 MPPI(xtensor 那套在本机编很久)。
            if ok:
                try:
                    self.node.pub_speed_limit.publish(
                        SpeedLimit(percentage=False, speed_limit=0.0))
                except Exception:
                    pass
            self.send_json(t='toast', ok=ok,
                msg=(f'导航速度已更新: 前进{vx:.2f} 后退{vn:.2f} 旋转{wz:.2f}rad/s '
                     '(仅本次导航会话有效, 重启导航恢复默认值)') if ok
                    else f'导航速度设置失败: {err}')
        elif t == 'get_nav_params':
            # 弹窗打开时把**当前真实值**读回来, 而不是每次都显示写死的默认值 ——
            # 否则用户看到的滑条位置和车上实际参数根本不是一回事
            vals = {}
            got = await self._get_params('/controller_server', [
                'FollowPath.vx_max', 'FollowPath.vx_min', 'FollowPath.wz_max',
                'FollowPath.ObstaclesCritic.inflation_radius',
                'FollowPath.ObstaclesCritic.cost_scaling_factor'])
            vals.update(got)
            self.send_json(t='nav_params', params=vals,
                           live=bool(got))
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
        elif t == 'preview_map':
            # 【选中地图就把它画出来】以前选地图只是把名字记进 localStorage,
            # 画布上什么都不变 —— 用户点了半天"地图加载不出来"。
            # 这里把选中那张的 pgm 推给前端当底图, 和导航真正加载的是同一张图,
            # 所以可以先看清楚再决定要不要用它启动导航。
            # 注意: 只是**预览**, 不影响 nav2 实际加载哪张(那个仍由启动参数决定)。
            name = (msg.get('name') or '').strip()
            if not _safe_map_name(name):
                self.send_json(t='toast', ok=False, msg='非法地图名')
                return
            # 导航跑着的时候不要拿静态图去盖实时 /map —— 那张才是当前真正在用的
            if self.mgr.alive() and self.mgr.mode == 'navigation':
                self.send_json(t='toast',
                    msg=f'导航运行中, 画布显示的是正在使用的地图; "{name}" 已选中, '
                        f'停止导航后重新启动才会加载它')
                return
            pgm = os.path.join(MAP_DIR_INSTALL, name + '.pgm')
            yml = os.path.join(MAP_DIR_INSTALL, name + '.yaml')
            if not (os.path.exists(pgm) and os.path.exists(yml)):
                self.send_json(t='toast', ok=False,
                               msg=f'地图 "{name}" 的文件不完整(缺 pgm 或 yaml)')
                return
            try:
                meta, png = pgm_to_png(pgm, yml)
                self._set_static_map(pack_map(meta, png))
                self.send_json(t='map_preview', name=name)
            except Exception as e:
                self.send_json(t='toast', ok=False, msg=f'加载地图失败: {e}')
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
        elif t == 'get_dock_point':
            sensor = msg.get('sensor', 'mid360')
            name = (msg.get('name') or '').strip()
            p = get_dock_point(sensor, name) if _safe_map_name(name) else None
            self.send_json(t='dock_point', sensor=sensor, name=name, point=p)
        elif t == 'save_dock_point':
            sensor = msg.get('sensor', 'mid360')
            name = (msg.get('name') or '').strip()
            if not _safe_map_name(name):
                self.send_json(t='toast', ok=False,
                    msg='请先在地图库选中一张要绑定回充点的地图')
                return
            ok = save_dock_point(sensor, name, msg['x'], msg['y'],
                                 msg.get('yaw', 0.0))
            self.send_json(t='toast', ok=ok,
                msg=(f'回充点已保存到地图 "{name}"'
                     if ok else '保存回充点失败(磁盘写入错误)'))
        elif t == 'clear_dock_point':
            sensor = msg.get('sensor', 'mid360')
            name = (msg.get('name') or '').strip()
            if _safe_map_name(name):
                delete_dock_point(sensor, name)
            self.send_json(t='toast', msg=f'已清除地图 "{name}" 的回充点')
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
            # name = 存成哪张图(可以是个新名字 = 另存为), src = 分辨率/原点从哪张图来。
            # 分开是因为另存为时新名字还没有 yaml, 元数据只能沿用被编辑的那张。
            name = (msg.get('name') or '').strip()
            src = (msg.get('src') or name).strip()
            png_b64 = msg.get('png_b64') or ''
            if not _safe_map_name(name) or not _safe_map_name(src) or not png_b64:
                self.send_json(t='toast', ok=False, msg='参数缺失或地图名非法')
                return
            yml = os.path.join(MAP_DIR_INSTALL, src + '.yaml')
            if not os.path.exists(yml):
                self.send_json(t='toast', ok=False,
                    msg=f'原地图 "{src}" 不存在, 无法确定分辨率/原点')
                return
            try:
                png_bytes = base64.b64decode(png_b64)
                dest = [(os.path.join(MAP_DIR_INSTALL, name + '.pgm'),
                        os.path.join(MAP_DIR_INSTALL, name + '.yaml')),
                       (os.path.join(MAP_DIR, name + '.pgm'),
                        os.path.join(MAP_DIR, name + '.yaml'))]
                bak = backup_map(name)
                edit_png_to_pgm(png_bytes, dest, yml)
                meta, png = pgm_to_png(dest[0][0], dest[0][1])
                self._set_static_map(pack_map(meta, png))
                tip = f'"{name}" 已保存'
                if bak:
                    tip += f' (旧图备份: _backup/{bak})'
                self.send_json(t='toast', msg=tip)
                self.send_json(t='maps', sensor='mid360', maps=list_2d_maps())
                # active=true 时前端才提示"要不要热加载" —— 编的正好是导航
                # 此刻在用的那张图, 不然重载没有意义
                self.send_json(t='map_saved', name=name,
                               active=(self.mgr.alive()
                                       and self.mgr.mode == 'navigation'
                                       and name == (self.mgr.map_name or
                                           DEFAULT_MAP_NAME.get(self.mgr.sensor, ''))))
            except Exception as e:
                self.send_json(t='toast', ok=False, msg=f'保存失败: {e}')
        elif t == 'reload_map':
            # 让运行中的导航热加载磁盘上的图, 不必重启整套 nav2。
            # 只有 map_server 会换图; AMCL 的似然场是订阅 /map 更新的, 会跟着变。
            name = (msg.get('name') or '').strip()
            if not _safe_map_name(name):
                self.send_json(t='toast', ok=False, msg='地图名非法')
                return
            if not (self.node.cli_loadmap
                    and self.node.cli_loadmap.service_is_ready()):
                self.send_json(t='toast', ok=False,
                    msg='/map_server/load_map 不在线 (导航没在跑), 下次启动导航时自动生效')
                return
            req = LoadMap.Request()
            req.map_url = os.path.join(MAP_DIR_INSTALL, name + '.yaml')
            fut = self.node.cli_loadmap.call_async(req)
            try:
                await asyncio.wait_for(self._wrap_future(fut), timeout=15.0)
                r = fut.result()
                ok = (r.result == LoadMap.Response.RESULT_SUCCESS)
                self.send_json(t='toast', ok=ok,
                    msg=(f'导航已换用 "{name}"' if ok else f'热加载失败(code={r.result})'))
                # 换了图, AMCL 手里那个位姿是按旧图算的, 多半已经不成立
                # —— 必须重定位, 否则车会以为自己在一个新图上不存在的地方
                if ok and msg.get('reloc'):
                    await self._call_relocalize()
            except asyncio.TimeoutError:
                self.send_json(t='toast', ok=False, msg='热加载超时')

    async def _get_params(self, node_fqn, names):
        """读回节点当前的参数值。取不到就返回空 dict(界面退回默认值显示)。"""
        cli = self.node.get_param_client(node_fqn)
        if not cli.service_is_ready():
            return {}
        req = GetParameters.Request()
        req.names = list(names)
        fut = cli.call_async(req)
        try:
            await asyncio.wait_for(self._wrap_future(fut), 5)
        except asyncio.TimeoutError:
            return {}
        out = {}
        for name, pv in zip(names, fut.result().values):
            if pv.type == 3:            # PARAMETER_DOUBLE
                out[name] = round(pv.double_value, 4)
            elif pv.type == 2:          # PARAMETER_INTEGER
                out[name] = pv.integer_value
        return out

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
        # 回充编排第二步: 到达回充点后触发停靠 (docking 节点须已启动)
        if kind == 'dock' and name == 'succeeded':
            await self._call_start_docking()
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
        # 'dock' 也走 NavigateToPose (ac_nav), 反馈字段与 'single' 一致;
        # 只有 'follow' 是 FollowWaypoints, 才有 current_waypoint。
        if kind in ('single', 'dock'):
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
            # 户外 RTK 定位模式下 auto_relocalize 是故意关的: 位置由 GPS 直接给出,
            # 点云配准重定位只会给出错误坐标。只有全局 EKF 发 /odometry/global,
            # 据此区分"故意关了"和"导航没起来", 别让用户误以为导航挂了。
            if self.node.count_publishers('/odometry/global') > 0:
                self.send_json(t='toast', ok=True,
                    msg='RTK 定位模式: 位置由 GPS 直接给出, 不需要重定位')
            else:
                self.send_json(t='toast', msg='/relocalize 服务不在线 (导航未启动?)', ok=False)
            return
        fut = self.node.cli_reloc.call_async(Trigger.Request())
        try:
            await asyncio.wait_for(self._wrap_future(fut), timeout=30.0)
            r = fut.result()
            self.send_json(t='toast', msg=f'重定位: {r.message}', ok=r.success)
        except asyncio.TimeoutError:
            self.send_json(t='toast', msg='重定位超时', ok=False)

    async def _call_start_docking(self):
        """触发 AprilTag 停靠。Trigger 只表示"已启动", 完成信号靠 /docking_node/state
        话题(见 task_fast 里的 dock_state 推送)。

        停靠节点已不再自带 pre-dock 导航(tagdocking 移除了 Nav2 预停靠):
        start_docking 直接 IDLE→SEARCH_TAG 搜码。回充时网页已用 Nav2 把车开到回充点
        并正对二维码, 到达后直接触发即可, 无需(也无法)再设 navigation.enable。
        """
        if not self.node.cli_dock_start.service_is_ready():
            self.send_json(t='toast', ok=False,
                msg='停靠节点未启动 (请先 ros2 launch tagdocking docking.launch.py)')
            return
        fut = self.node.cli_dock_start.call_async(Trigger.Request())
        try:
            await asyncio.wait_for(self._wrap_future(fut), timeout=10.0)
            r = fut.result()
            self.send_json(t='toast', msg=f'停靠: {r.message}', ok=r.success)
        except asyncio.TimeoutError:
            self.send_json(t='toast', msg='停靠服务超时', ok=False)

    async def _call_undock(self):
        if not self.node.cli_dock_undock.service_is_ready():
            self.send_json(t='toast', ok=False,
                msg='停靠节点未启动 (请先 ros2 launch tagdocking docking.launch.py)')
            return
        fut = self.node.cli_dock_undock.call_async(Trigger.Request())
        try:
            await asyncio.wait_for(self._wrap_future(fut), timeout=10.0)
            r = fut.result()
            self.send_json(t='toast', msg=f'泊出: {r.message}', ok=r.success)
        except asyncio.TimeoutError:
            self.send_json(t='toast', msg='泊出服务超时', ok=False)

    async def _start_recharge(self, sensor='mid360', name=''):
        """回充编排: 导航到回充点(带朝向) → 到达后由 _run_nav 触发停靠。

        回充点是"pre-docking"点: 坐标 + yaw 让 Nav2 把车开到点且尽量正对二维码,
        这样到达后 SEARCH_TAG 几乎不用旋转搜索, 停泊时间最短。
        """
        # 1) 导航必须已在运行
        if not (self.mgr.alive() and self.mgr.mode == 'navigation'):
            self.send_json(t='toast', ok=False, msg='导航未启动，无法回充')
            return
        # 2) 已有导航任务 → 什么也不做
        if self.goal_handle is not None or \
                (self.nav_task and not self.nav_task.done()):
            self.send_json(t='toast', ok=False, msg='已有导航任务，忽略回充')
            return
        # 2.5) 停靠/泊出进行中也不要再触发回充 (到达后的停靠不会走状态机, 得在这里拦)
        ds = self.node.latest.get('dock_state')
        if ds in ('navigating', 'search_tag', 'align', 'approach',
                  'final_servo', 'undocking'):
            self.send_json(t='toast', ok=False, msg='停靠/泊出进行中，忽略回充')
            return
        # 3) 回充点必须已设置
        name = (name or '').strip()
        if not _safe_map_name(name):
            self.send_json(t='toast', ok=False, msg='请先在地图库选中一张地图')
            return
        p = get_dock_point(sensor, name)
        if p is None:
            self.send_json(t='toast', ok=False, msg='请先设置回充点')
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.node.make_pose(p['x'], p['y'], p['yaw'])
        self.send_json(t='toast', msg=f'回充：导航到回充点 ({p["x"]:.2f}, '
                                      f'{p["y"]:.2f})，朝向 {math.degrees(p["yaw"]):.0f}°')
        self._spawn_nav(self.node.ac_nav, goal, 'dock', 1)

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
            if not self.clients:
                self.node.cam_want = False   # 没人看了就停止编码画面
                self.node.scene_want = False  # 实景高帧率推送也一并退回 1Hz
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
        if p in ('/v2', '/v2.html'):
            body = self.v2_html
            if body is None:
                return (http.HTTPStatus.NOT_FOUND, [], b'v2.html not found')
            return (http.HTTPStatus.OK,
                    [('Content-Type', 'text/html; charset=utf-8'),
                     ('Cache-Control', 'no-cache')],
                    body)
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
        self.gps_mon = GpsMonitor()
        # 让残留检测认得自家的 GPS 监控(它跑的也是 dual_rtk_driver),
        # 否则户外模式下会把它误报成残留进程
        self.mgr.own_pgid_hook = (
            lambda: self.gps_mon.proc.pid if self.gps_mon.alive() else None)
        self.gps_mon_want = False   # 前端是否处于户外模式(任务结束后据此决定要不要拉回来)
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
            await asyncio.gather(self.task_state(), self.task_fast(), self.task_cam(),
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
