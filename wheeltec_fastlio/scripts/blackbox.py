#!/usr/bin/env python3
"""黑匣子: 每秒把机器状态 append 到磁盘并 fsync, 硬断电也留得住。

【为什么要有它】2026-08-07 整机卡死过一次, 事后完全查不出原因:
  - journald 是带缓冲的, 硬断电丢掉最后几分钟 —— 而那几分钟恰好是关键
  - /proc/cmdline 里没有 ramoops=, pstore 没后端, 内核 panic 也留不下痕迹
  - 硬件看门狗默认关着(RuntimeWatchdogUSec=0)
所以卡死前的内存/温度/负载全是空白。这个脚本就是补这一段: **每条都 fsync**,
断电最多丢当前这一秒。

【怎么用】
    ros2 run wheeltec_fastlio blackbox.py &            # 手动跑
    ros2 run wheeltec_fastlio blackbox.py --install    # 装成 systemd 服务(开机自启)
    ros2 run wheeltec_fastlio blackbox.py --report     # 看上次断电前发生了什么

日志在 ~/blackbox.log, 自动轮转(超过 MAX_MB 就砍掉前一半), 一天约 3MB。
每次开机会写一条 BOOT 分隔行, --report 就是把最后一个 BOOT 之前的尾巴打出来。

【读法】卡死前几秒如果:
  - avail 一路掉到几十 MB      -> 内存不够, 八成是 OOM/换页风暴(见 CLAUDE.md 第6节)
  - temp 冲到 85000 以上        -> 过热降频, 严重时会挂
  - load 飙升但 avail 正常      -> 某个进程跑飞了, 看 top3
  - 什么都正常, 日志戛然而止    -> 更像内核/驱动层面的死锁(本机 Realtek 无线驱动
                                   有前科, 见 CLAUDE.md "📡 网络架构")
"""
import os
import shutil
import subprocess
import sys
import time

LOG = os.path.expanduser('~/blackbox.log')
PERIOD = 1.0
MAX_MB = 8
SERVICE = '/etc/systemd/system/wheeltec-blackbox.service'

UNIT = """[Unit]
Description=WHEELTEC blackbox (1Hz fsync'd machine-state recorder)
After=multi-user.target

[Service]
Type=simple
User={user}
ExecStart={python} {script}
Restart=always
RestartSec=5
Nice=10

[Install]
WantedBy=multi-user.target
"""


def _read(path, default=''):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


def _mem():
    """(可用MB, 交换已用MB) —— 看 MemAvailable 而不是 free, free 会被缓存吓到。"""
    avail = swtot = swfree = 0
    for line in _read('/proc/meminfo').splitlines():
        k, _, v = line.partition(':')
        v = v.strip().split(' ')[0]
        if k == 'MemAvailable':
            avail = int(v) // 1024
        elif k == 'SwapTotal':
            swtot = int(v) // 1024
        elif k == 'SwapFree':
            swfree = int(v) // 1024
    return avail, swtot - swfree


def _temp():
    t = _read('/sys/class/thermal/thermal_zone0/temp', '0').strip()
    try:
        return int(t)
    except ValueError:
        return 0


def _top3():
    """吃内存最多的三个进程。卡死现场最想知道的就是"谁在涨"。"""
    out = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        st = _read(f'/proc/{pid}/statm').split()
        if len(st) < 2:
            continue
        rss = int(st[1]) * 4 // 1024          # 页 -> MB
        if rss < 40:
            continue
        name = _read(f'/proc/{pid}/comm').strip()
        out.append((rss, name))
    out.sort(reverse=True)
    return ','.join(f'{n}:{r}' for r, n in out[:3])


def _rotate(f):
    if os.path.getsize(LOG) < MAX_MB * (1 << 20):
        return f
    f.close()
    with open(LOG) as src:
        lines = src.readlines()
    with open(LOG, 'w') as dst:
        dst.writelines(lines[len(lines) // 2:])
    return open(LOG, 'a')


def run():
    f = open(LOG, 'a')
    boot = _read('/proc/uptime').split()[0]
    f.write(f'==== BOOT {time.strftime("%F %T")} (uptime {boot}s) ====\n')
    f.flush()
    os.fsync(f.fileno())
    n = 0
    while True:
        try:
            avail, swused = _mem()
            la = _read('/proc/loadavg').split()[:3]
            root = shutil.disk_usage('/')
            line = (f'{time.strftime("%F %T")} avail={avail}M swap={swused}M '
                    f'load={la[0]} temp={_temp()} disk={root.free >> 20}M')
            if n % 10 == 0:                    # top3 每10秒记一次, 省开销
                line += f' top={_top3()}'
            f.write(line + '\n')
            f.flush()
            # 【关键】必须 fsync。只 flush 的话数据还在页缓存里, 硬断电照样丢 ——
            # 那就跟 journald 一个下场, 这个脚本也就白写了。
            os.fsync(f.fileno())
            n += 1
            if n % 60 == 0:
                f = _rotate(f)
        except Exception:
            time.sleep(5)                      # 出错也别退出, 黑匣子要一直在
        time.sleep(PERIOD)


def report():
    if not os.path.exists(LOG):
        print(f'还没有 {LOG} —— 黑匣子没跑过。'
              f'\n装上: ros2 run wheeltec_fastlio blackbox.py --install')
        return
    lines = _read(LOG).splitlines()
    boots = [i for i, l in enumerate(lines) if l.startswith('==== BOOT')]
    if len(boots) < 2:
        print('只有一次开机记录, 还没有"上次断电前"可看。')
        print('\n'.join(lines[-15:]))
        return
    # 最后一个 BOOT 之前的 30 行 = 上次机器停止响应之前的最后半分钟
    tail = lines[max(0, boots[-1] - 30):boots[-1]]
    print(f'=== 上次关机/卡死前最后 {len(tail)} 秒 ===')
    print('\n'.join(tail))
    print(f'\n=== 本次开机 ===\n' + '\n'.join(lines[boots[-1]:boots[-1] + 3]))
    print('\n读法: avail 掉到几十M=内存不够; temp>85000=过热; '
          'load 飙升=有进程跑飞; 全正常但戛然而止=更像内核/驱动死锁')


def install():
    unit = UNIT.format(user=os.environ.get('USER', 'cat'),
                       python=sys.executable, script=os.path.abspath(__file__))
    tmp = '/tmp/wheeltec-blackbox.service'
    with open(tmp, 'w') as f:
        f.write(unit)
    print(f'已生成 {tmp}\n需要 sudo 才能装成开机自启, 请手动执行:\n')
    print(f'  sudo cp {tmp} {SERVICE}')
    print('  sudo systemctl daemon-reload')
    print('  sudo systemctl enable --now wheeltec-blackbox')
    print('\n之后卡死重启, 用这个看断电前发生了什么:')
    print('  ros2 run wheeltec_fastlio blackbox.py --report')


if __name__ == '__main__':
    if '--report' in sys.argv:
        report()
    elif '--install' in sys.argv:
        install()
    else:
        run()
