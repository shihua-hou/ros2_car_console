# coding=utf-8
from pyproj import CRS, Transformer
import threading
import traceback
import serial
import time
import math



def crc_table():
    table = []
    for i in range(256):
        crc = i
        for j in range(8, 0, -1):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
        table.append(crc)
    return table

NMEA_EXPEND_CRC_TABLE = crc_table()

def open_serial_with_retry(port, baudrate, retry=5, delay=1):
    for i in range(retry):
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            #print(f"串口{port}打开成功")
            return ser
        except serial.SerialException as e:
            #print(f"[尝试{i+1}] 串口 {port} 打开失败: {e}")
            time.sleep(delay)
    raise serial.SerialException(f"串口 {port} 在 {retry} 次尝试后仍无法打开")
    
def nmea_expend_crc(nmea_expend_sentence):
    def calculate_crc32(data):
        crc = 0
        for byte in data:
            crc = NMEA_EXPEND_CRC_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    try:
        sentence, crc = nmea_expend_sentence[1:].split("*")
        crc = crc[:8]
    except:
        return False
    calculated_crc = calculate_crc32(sentence.encode())
    return crc.lower() == format(calculated_crc, '08x')

def nmea_crc(nmea_sentence):
    # 移除起始的'$'和'*'及之后的校验和部分
    try:
        sentence, crc = nmea_sentence[1:].split("*")
        crc = crc[:2]
    except:
        return False
    calculated_checksum = 0
    # 对字符串中的每个字符进行异或运算
    for char in sentence:
        calculated_checksum ^= ord(char)
    # 将计算得到的校验和转换为十六进制格式，并大写
    calculated_checksum_hex = format(calculated_checksum, 'X')
    # 校验和比较
    return calculated_checksum_hex.zfill(2) == crc.upper()

def msg_seperate(msg:str):
    return msg[1:msg.find('*')].split(',')

def PVTSLN_solver(msg:str):
    parts = msg_seperate(msg)
    bestpos_hgt    = float(parts[3+7])          # 海拔高，单位：米
    bestpos_lat    = float(parts[4+7])          # 纬度，单位：度 （输出小数点后 11 位）
    bestpos_lon    = float(parts[5+7])          # 经度，单位：度 （输出小数点后 11 位）
    bestpos_hgtstd = float(parts[6+7])          # 高程标准差，单位：米
    bestpos_latstd = float(parts[7+7])          # 纬度标准差，单位：米
    bestpos_lonstd = float(parts[8+7])          # 经度标准差，单位：米
    fix = (bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd)
    return fix


def GNHPR_solver(msg:str):
    parts = msg_seperate(msg)
    heading = float(parts[3-1])
    pitch   = float(parts[4-1])
    roll    = float(parts[5-1])
    orientation = (heading, pitch, roll)
    return orientation


def BESTNAV_solver(msg:str):
    parts = msg_seperate(msg)
    vel_hor_std = float(parts[-1])  # 水平速度标准差，单位 m/s
    vel_ver_std = float(parts[-2])  # 高程速度标准差，单位 m/s
    vel_ver     = float(parts[-3])  # 垂直速度， m/s，正值表示高度增加（向上），负值表示高度下降（向下）
    vel_heading = float(parts[-4])  # 相对于真北的实际对地运动方向（相对地面轨迹）， deg
    vel_hor     = float(parts[-5])  # 对地水平速度， m/s
    vel_north   = vel_hor * math.cos(math.radians(vel_heading))     # 分解得到北方向速度
    vel_east    = vel_hor * math.sin(math.radians(vel_heading))     # 分解得到东方向速度
    return (vel_east, vel_north, vel_ver, vel_hor_std, vel_hor_std, vel_ver_std)


# GGA fix quality -> 定位标准差估计（米），quality 含义见 NMEA0183: 0=无效 1=单点 2=DGPS 4=RTK固定 5=RTK浮点 6=航位推算
_GGA_QUALITY_STD_M = {
    1: 5.0,
    2: 1.0,
    4: 0.02,
    5: 0.5,
}

# GGA fix quality -> gps_localization_guard 使用的质量等级：-1=无效 0=单点 1=差分 2=固定
_GGA_QUALITY_TO_GUARD_STATUS = {
    0: -1,
    1: 0,
    2: 1,
    4: 2,
    5: 1,
    6: -1,
}


def gga_quality_to_guard_status(fix_quality):
    return _GGA_QUALITY_TO_GUARD_STATUS.get(fix_quality, -1)


def _gga_coord_to_decimal(raw, hemisphere, deg_digits):
    """把 GGA 的 ddmm.mmmm / dddmm.mmmm 转成十进制度，raw 为空（无定位）时返回 None"""
    if not raw:
        return None
    degrees = float(raw[:deg_digits])
    minutes = float(raw[deg_digits:])
    value = degrees + minutes / 60.0
    if hemisphere in ('S', 'W'):
        value = -value
    return value


def GNGGA_solver(msg:str):
    """解析 $GNGGA，返回 (fix六元组或None, fix_quality)。无定位时 fix 为 None。"""
    parts = msg_seperate(msg)
    fix_quality = int(parts[6]) if parts[6] else 0
    lat = _gga_coord_to_decimal(parts[2], parts[3], 2)
    lon = _gga_coord_to_decimal(parts[4], parts[5], 3)
    if fix_quality <= 0 or lat is None or lon is None or not parts[9]:
        return None, fix_quality
    bestpos_hgt = float(parts[9])
    std = _GGA_QUALITY_STD_M.get(fix_quality, 999.0)
    fix = (bestpos_hgt, lat, lon, std, std, std)
    return fix, fix_quality



def create_utm_trans(lat, lon):
    """构建转换器，用于将WGS84地理坐标系下的点转换为UTM坐标系下的点。

    Args:
        lon (float): 点的经度。
        lat (float): 点的纬度。

    Returns:
        transformer: 转换器
    """
    # UTM区号是根据经度确定的，从-180度开始每6度一个区间。
    zone_number            = int((lon + 180) / 6) + 1
    # 北半球是赤道（纬度0度）以上的区域。
    isnorth                = lat >= 0
    # 定义WGS84坐标系
    wgs84_crs              = CRS("epsg:4326")
    # 根据是否位于北半球，选择合适的UTM EPSG代码
    utm_crs_str            = f"epsg:326{zone_number}" if isnorth else f"epsg:327{zone_number}"
    utm_crs                = CRS(utm_crs_str)
    # 创建坐标转换器，从WGS84转换到UTM
    transformer            = Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)
    return transformer


def utm_trans(transformer, lon, lat):
    """将WGS84地理坐标系下的点转换为UTM坐标系下的点。

    Args:
        transformer: 转换器
        lon (float): 点的经度。
        lat (float): 点的纬度。

    Returns:
        tuple: 一个元组，包含转换后的UTM坐标系下的x（东坐标）和y（北坐标）。
    """
    # 进行坐标转换
    utm_x, utm_y           = transformer.transform(lon, lat)
    return (utm_x, utm_y)


class UM982Serial(threading.Thread):
    def __init__(self, port, band):
        super().__init__()
        # 异常限流: {位置: 上次打印时刻} / {位置: 累计次数}
        self._exc_last = {}
        self._exc_count = {}
        # 打开串口
        #self.ser            = serial.Serial(port, band)
        self.ser = open_serial_with_retry(port, band)

        # 设置运行标志位
        self.isRUN          = True
        # 数据
        self.fix            = None   # sensor_msgs/NavSatFix 所需要的数据
        self.fix_quality    = 0      # GGA fix quality，供上层映射为 NavSatStatus
        self.orientation    = None   # 航向角
        self.vel            = None   # 速度
        self.utmpos         = None
        self.transformer    = None   # UTM 转换器, 拿到第一个定位才建(分带看经度)
        self.frames_ok      = 0      # 校验通过的报文数: 区分"串口没数据"和"有数据没定位"
        self.gga_seen       = False  # 收到过 GGA(哪怕质量位是 0), 说明接收机活着
        # 读初始数据: 等到拿到定位, 或确认接收机在出数据但还没定位。
        # 以前要求 15 秒内必须有定位, 否则整个驱动退出 —— 刚上电/还在楼边搜星时切到
        # 户外模式, 驱动就这么死了, 等搜到星也没人再拉起来, 界面一直"GPS未启动"
        # (2026-09-22 实际发生)。没定位不是故障: 驱动照常跑, 界面显示"无信号·搜星中",
        # 定位后自动开始发布。只有 15 秒一条有效报文都没有(端口不对/接收机没电)才失败。
        fix_wait_start = time.time()
        fix_wait_timeout = 15.0  # 秒
        while self.fix is None and not self.gga_seen and \
                (time.time() - fix_wait_start) < fix_wait_timeout:
            self.read_frame()
        if self.frames_ok == 0:
            raise RuntimeError(f'{fix_wait_timeout}s 内串口没有任何有效报文(GNGGA/PVTSLNA/GNHPR), '
                               f'检查端口或接收机供电')
        if self.fix is None:
            print(f'[um982] 串口有数据但接收机还没定位(GGA 质量位 {self.fix_quality}), '
                  f'驱动照常运行, 定位后自动开始发布', flush=True)

        # wgs84转utm(启动时还没定位的话, 由 run() 在第一次定位时建)
        if self.fix is not None:
            bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.fix
            self.transformer = create_utm_trans(bestpos_lat, bestpos_lon)
            self.utmpos      = utm_trans(self.transformer, bestpos_lon, bestpos_lat)


    def stop(self):
        """ 结束运行 """
        self.isRUN = False
        time.sleep(0.1)
        self.ser.close()


#    def read_frame(self):
#        frame = self.ser.readline().decode('utf-8')
#        if frame.startswith("#PVTSLNA") and nmea_expend_crc(frame):
#            self.fix = PVTSLN_solver(frame)
#        elif frame.startswith("$GNHPR") and nmea_crc(frame):
#            self.orientation = GNHPR_solver(frame)
#        elif frame.startswith("#BESTNAVA") and nmea_expend_crc(frame):
#            self.vel = BESTNAV_solver(frame)

    def read_frame(self):
        try:
            raw = self.ser.readline()
            frame = raw.decode('utf-8', errors='ignore').strip()
            ok = True
            if frame.startswith("#PVTSLNA") and nmea_expend_crc(frame):
                self.fix = PVTSLN_solver(frame)
            elif frame.startswith("$GNHPR") and nmea_crc(frame):
                self.orientation = GNHPR_solver(frame)
            elif frame.startswith("#BESTNAVA") and nmea_expend_crc(frame):
                self.vel = BESTNAV_solver(frame)
            elif frame.startswith("$GNGGA") and nmea_crc(frame):
                fix, fix_quality = GNGGA_solver(frame)
                self.fix_quality = fix_quality
                self.gga_seen = True
                if fix is not None:
                    self.fix = fix
            else:
                ok = False
            if ok:
                self.frames_ok += 1
        except Exception:
            if not self.isRUN:
                return    # stop() 关了串口, 读线程正卡在 readline 上: 正常收尾, 不是故障
            # 原来只 print(str(e)) —— 一行 "'NoneType' object cannot be
            # interpreted as an integer" 根本定位不到是哪个 solver 抛的。
            # 限流是因为串口一旦进入坏状态会每帧都抛, 不限流会瞬间刷满日志。
            self._log_exc('read_frame')

    def _log_exc(self, where):
        """带限流的异常记录: 同一处 30 秒最多打一次完整 traceback。"""
        now = time.time()
        last = self._exc_last.get(where, 0.0)
        self._exc_count[where] = self._exc_count.get(where, 0) + 1
        if now - last < 30.0:
            return
        self._exc_last[where] = now
        n = self._exc_count[where]
        print(f"[um982] {where} 异常 (累计 {n} 次):", flush=True)
        traceback.print_exc()

    def run(self):
        """读串口线程体。

        这里**必须**兜住所有异常: 这是 threading.Thread 的 run(), 抛出去线程
        就结束了 —— 而进程还活着、还占着串口、ROS 节点还在 spin, 表现为 GPS
        毫无征兆地不再更新, 外部还查不出来(2026-09-16 实际发生, 一次把测试
        打断)。宁可一直重试, 也不能让线程死。
        """
        while self.isRUN:
            try:
                self.read_frame()
                if self.fix is None:
                    continue          # 还没拿到有效定位, 别解包 None
                (bestpos_hgt, bestpos_lat, bestpos_lon,
                 bestpos_hgtstd, bestpos_latstd, bestpos_lonstd) = self.fix
                if self.transformer is None:
                    self.transformer = create_utm_trans(bestpos_lat, bestpos_lon)
                self.utmpos = utm_trans(self.transformer,
                                        bestpos_lon, bestpos_lat)
            except Exception:
                self._log_exc('run')
                time.sleep(0.02)      # 坏状态下别空转把 CPU 吃满




if __name__ == "__main__":
    um982 = UM982Serial("/dev/wheeltec_gnss", 115200)
    um982.start()

