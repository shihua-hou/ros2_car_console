#!/bin/bash
# wheeltec_webapp: 双网卡网络配置脚本 (板载卡连WiFi + USB卡常开热点)
# 需要 sudo 权限运行一次: sudo bash ~/wheeltec_ros2/src/wheeltec_webapp/wifi_ap_setup.sh
#
# 架构 (2026-07-24 变更, 详见 CLAUDE.md):
#   板载网卡 wlan0 (RTL8852BE): 保持站点模式, 连家里的WiFi, 本脚本不动它
#   USB 网卡  wlan1 (RTL8821CU): 常开热点 WHEELTEC-CAR, 供现场没有WiFi时直连小车
#   两块卡是两个独立的 phy, 可以同时在线, 不存在"切换"这回事, 开关热点也不会
#   影响到走WiFi的网页/SSH连接。
#
#   为什么不用板载卡做热点: RTL8852BE 用的是 Realtek 树外驱动 8852be.ko, 不走
#   mac80211, 向 cfg80211 上报的 interface combinations 是空的, 实测切AP会整机
#   卡死到只能硬重启。USB 卡 (8821cu) 上报的是
#     #{ managed, P2P-client } <= 2, #{ AP, P2P-GO } <= 1
#   有合法的 AP 组合, 是正常可用的。
#
# 做两件事:
#   1) 创建绑定在 USB 网卡上的热点连接 "wheeltec-ap-usb"
#      (SSID=WHEELTEC-CAR, WPA2密码=CHANGE_ME, 固定IP 192.168.0.100/24)
#   2) 装一条 polkit 规则, 让当前用户($SUDO_USER)能在网页里一键开关热点而
#      不用每次输密码(仅"启停网络连接"这一项权限, 不开放其它系统级操作)。
set -e

RUN_USER="${SUDO_USER:-cat}"
STA_PROFILE="dmx-public"        # 板载卡连的现有WiFi连接名, 仅用于末尾提示
AP_PROFILE="wheeltec-ap-usb"    # 跟 app.py 里的 AP_PROFILE 必须一致
AP_IFACE="wlan1"                # 跟 app.py 里的 AP_IFACE 必须一致
SSID="WHEELTEC-CAR"
PASSWORD="CHANGE_ME"
AP_IP="192.168.0.100/24"
AP_CHANNEL="6"                  # 2.4G 固定信道, 不用 0(=ACS自动选), 见下方说明

if [ "$(id -u)" != "0" ]; then
  echo "请用 sudo 运行: sudo bash $0" >&2
  exit 1
fi

echo "== 0) 检查 USB 网卡 $AP_IFACE =="
if [ ! -d "/sys/class/net/$AP_IFACE" ]; then
  echo "   ✗ 找不到 $AP_IFACE, USB无线网卡没插好或驱动没加载。" >&2
  echo "     排查: lsusb | grep -i realtek ; ip -br link ; dmesg | tail -30" >&2
  exit 1
fi
AP_PHY="$(basename "$(readlink -f /sys/class/net/$AP_IFACE/phy80211)")"
if iw phy "$AP_PHY" info 2>/dev/null | grep -q "interface combinations are not supported"; then
  echo "   ✗ $AP_IFACE ($AP_PHY) 向 cfg80211 上报了零个接口组合, 这张卡做AP会有" >&2
  echo "     整机卡死风险(板载 RTL8852BE 就是这样), 拒绝继续。" >&2
  exit 1
fi
echo "   OK: $AP_IFACE -> $AP_PHY, 接口组合:"
iw phy "$AP_PHY" info 2>/dev/null | sed -n '/valid interface combinations/,+2p' | sed 's/^/     /'

echo "== 1) 创建热点连接配置 $AP_PROFILE (绑定 $AP_IFACE) =="
if nmcli -t -f NAME con show | grep -qx "$AP_PROFILE"; then
  echo "   已存在, 跳过创建(如需改密码/SSID请先 nmcli con delete $AP_PROFILE 再重跑)"
else
  nmcli con add type wifi ifname "$AP_IFACE" con-name "$AP_PROFILE" \
    ssid "$SSID" 802-11-wireless.mode ap \
    ipv4.method shared ipv4.addresses "$AP_IP"
  echo "   已创建: SSID=$SSID 密码=$PASSWORD 固定IP=$AP_IP"
fi

echo "== 1b) 应用热点参数 =="
# 这一段对已存在的旧 profile 同样执行(nmcli con modify 幂等), 所以改了上面的
# 变量或者想修正老配置, 直接重跑本脚本即可, 不需要先 delete。
nmcli con modify "$AP_PROFILE" \
  connection.interface-name "$AP_IFACE" \
  connection.autoconnect yes \
  connection.autoconnect-priority 10 \
  802-11-wireless.band bg \
  802-11-wireless.channel "$AP_CHANNEL" \
  ipv4.never-default yes \
  ipv6.method ignore \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "$PASSWORD" \
  wifi-sec.proto rsn \
  wifi-sec.pairwise ccmp \
  wifi-sec.group ccmp
#   autoconnect yes : 热点常开, 开机自动起来(这是双网卡架构的核心 —— 不再是
#                     "平时关着, 要用才切过去", 而是两条路一直都在)。
#   never-default   : 热点不抢默认路由, 本机出网继续走WiFi/有线。
#   band bg + 固定信道: USB卡的 regdomain 跟随全局 country 00, 该域下 5GHz 全是
#                     PASSIVE-SCAN(禁止主动发射), 所以热点只能做在 2.4G ch1-11。
#                     信道写死可跳过 ACS 全band扫描, 起得更快。
#   proto/pairwise/group: 写死 WPA2+CCMP, 不给厂商驱动走 TKIP 老代码路径的机会。
echo "   已应用: $AP_IFACE / 信道$AP_CHANNEL / 开机自启 / WPA2-CCMP / 不抢默认路由"

echo "== 2) 安装 polkit 授权规则, 允许 $RUN_USER 免密开关热点 =="
# 本机 polkit 是 0.105(Ubuntu 22.04 自带), 这个版本原生用 .pkla 格式
# (localauthority), 不是较新版本的 JS .rules 格式, 目录也不会预先建好。
mkdir -p /etc/polkit-1/localauthority/50-local.d
cat > /etc/polkit-1/localauthority/50-local.d/50-wheeltec-nm.pkla <<EOF
[wheeltec_webapp 允许 ${RUN_USER} 免密切换网络连接]
Identity=unix-user:${RUN_USER}
Action=org.freedesktop.NetworkManager.network-control;org.freedesktop.NetworkManager.wifi.share.protected
ResultAny=yes
ResultInactive=yes
ResultActive=yes
EOF
systemctl restart polkit
echo "   已写入 /etc/polkit-1/localauthority/50-local.d/50-wheeltec-nm.pkla 并重启 polkit"

echo
echo "完成。现在两块网卡各司其职, 网页"网络"卡片会分别显示两者状态:"
echo "  板载卡 wlan0  : 站点模式, 连 $STA_PROFILE (本脚本不动它)"
echo "  USB卡 $AP_IFACE   : 热点 $SSID, 密码 $PASSWORD"
echo
echo "现在启用热点:"
echo "  sudo nmcli con up $AP_PROFILE"
echo "然后手机连 \"$SSID\" 访问 http://${AP_IP%/*}:8080"
echo "(热点是常开的, 开机会自动起来; 网页上也能随时开关)"
