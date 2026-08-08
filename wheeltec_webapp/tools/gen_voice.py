#!/usr/bin/env python3
"""预生成语音播报音频片段 (供红米平板等无 Web Speech API 的系统浏览器使用)。

背景: 网页原来用浏览器自带的 speechSynthesis 发声, 但红米平板自带浏览器
(以及很多安卓 WebView 内核) 没有 TTS 引擎, 静默不出声。改为后端预生成音频
文件、浏览器只播 <audio>, 就跟浏览器有没有 TTS 引擎无关了。

本脚本用 edge-tts (微软在线神经网络 TTS, 中文自然度高) 一次性把固定播报词
生成成 mp3, 放到 static/voice/。**只有生成时需要联网, 运行时完全离线。**
以后新增/修改播报词, 改下面 PHRASES 再重跑本脚本即可。

用法:
  pip3 install edge-tts
  python3 tools/gen_voice.py           # 只补生成缺的文件(已有的不动)
  python3 tools/gen_voice.py --force   # 全部重新生成
  python3 tools/gen_voice.py --list    # 只列清单, 不联网
然后 colcon build --packages-select wheeltec_webapp 让文件进 install/。

**默认只补缺失的**: 老版界面 index.html 按"文本->文件名"查表播放, 它认的是
2026-07-24 那批文件实际念出来的词; 无差别重生成会让老版界面的播报内容悄悄变掉。
真要改某条的措辞, 改下面的文本再单独 --force 重跑。
"""
import argparse
import asyncio
import os
import sys

VOICE = "zh-CN-XiaoxiaoNeural"   # 自然女声; 备选 zh-CN-YunxiNeural(男)
OUT_DIR = os.path.join(os.path.dirname(__file__), '..',
                       'wheeltec_webapp', 'static', 'voice')

# ---------------------------------------------------------------------------
# 老版界面 index.html 用的一批 (文件名, 中文)。文件名须与 index.html 的
# Voice._files 一致。这些文件 2026-07-24 已生成, 默认不会被覆盖。
# ---------------------------------------------------------------------------
PHRASES = [
    ('arrived',  '已到达目标'),
    ('failed',   '导航失败'),
    ('canceled', '导航已取消'),
    ('rejected', '目标被拒绝'),
    # 念 "重新定位成功": "重" 单独在 "重定位" 里 edge-tts 会误读成 zhòng, 用 "重新"
    # (chóngxīn, 无歧义) 强制读二声, 语义一致。文件名/前端 key 仍是 reloc/重定位成功。
    ('reloc',    '重新定位成功'),
    ('estop',    '急停'),
    ('voice_on', '语音播报已开启'),
]
# 多点巡航航点进度: 到达第N个航点。预生成 1..MAX_WP, 覆盖常见巡航点数。
MAX_WP = 30
for i in range(1, MAX_WP + 1):
    PHRASES.append((f'wp_{i}', f'到达第{i}个航点'))
# 超出 MAX_WP 时的兜底短语 (前端匹配不到 wp_N 时退回这条)
PHRASES.append(('wp_generic', '已到达航点'))

# ---------------------------------------------------------------------------
# 新版控制台 /v2 用的一批 (2026-08-07 新增)。
# v2 是按 **key** 直接取 /voice/<key>.mp3, 不再走"文本->文件名"查表,
# 所以这里的文件名必须和 v2.html 里 SPEAK 表的键**逐字对应**。
#
# 其中 start_<mode> / stop_<mode> 是任务状态跃迁时自动播的, mode 取值来自
# app.py 的 SENSOR_PROFILES['mid360']['launches']: mapping / save_map /
# navigation / lidar_test —— 四个模式的起停共八条, 少一条那次跃迁就是哑的。
#
# 有几条和上面老版那批**内容重复**(nav_failed/failed、nav_canceled/canceled、
# nav_rejected/rejected、sound_on/voice_on), 是故意的: 老版按文本查表、新版按 key
# 取文件, 各自的命名不好互相迁就。与其加一层 key->文件名 的别名表(以后每次新增
# 播报词都得想"这条要不要复用老文件"), 不如让规则简单到没有例外 ——
# **v2 的 key 就是文件名**。代价是多几个 10KB 的 mp3。
# key 和老文件名恰好一致的(arrived / estop)直接复用, 不重复生成。
# ---------------------------------------------------------------------------
PHRASES += [
    # 任务起停 (S.mode 的四种取值 × 起/停)
    ('start_navigation', '导航已启动'),
    ('stop_navigation',  '导航已关闭'),
    ('start_mapping',    '建图已启动'),
    ('stop_mapping',     '建图已结束'),
    ('start_save_map',   '开始保存地图'),
    ('stop_save_map',    '地图已保存'),
    ('start_lidar_test', '雷达测试已启动'),
    ('stop_lidar_test',  '雷达测试已结束'),
    # 导航过程
    ('goal_sent',    '目标已下发'),
    ('go_home',      '正在返回起点'),
    ('nav_failed',   '导航失败'),
    ('nav_canceled', '导航已取消'),
    ('nav_rejected', '目标被拒绝'),
    ('cruise_start', '开始巡航'),
    ('cruise_done',  '巡航完成'),
    # 状态提示
    ('save_map',    '开始保存地图'),
    ('map_saved',   '地图已保存'),
    ('relocalized', '重新定位成功'),      # "重定位" 的 "重" 会被误读, 同上
    ('reloc_lost',  '定位丢失, 正在重新定位'),
    ('low_battery', '电量偏低, 请及时充电'),
    ('sound_on',    '语音播报已开启'),
    ('obstacle',    '前方有障碍'),
]


def out_path(name):
    return os.path.join(OUT_DIR, name + '.mp3')


async def gen_one(name, text):
    import edge_tts
    await edge_tts.Communicate(text, VOICE).save(out_path(name))
    print(f'  + {name}.mp3  <-  "{text}"')


async def main(force):
    os.makedirs(OUT_DIR, exist_ok=True)
    todo = [(n, t) for n, t in PHRASES
            if force or not os.path.exists(out_path(n))]
    skipped = len(PHRASES) - len(todo)
    print(f'生成到: {os.path.abspath(OUT_DIR)}  (voice={VOICE})')
    print(f'共 {len(PHRASES)} 条, 需生成 {len(todo)} 条, 已存在跳过 {skipped} 条')
    for name, text in todo:
        await gen_one(name, text)
    print('完成。别忘了 colcon build --packages-select wheeltec_webapp')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--force', action='store_true', help='已存在的也重新生成')
    ap.add_argument('--list', action='store_true', help='只列清单, 不联网')
    a = ap.parse_args()
    if a.list:
        for n, t in PHRASES:
            mark = ' ' if os.path.exists(out_path(n)) else '*'
            print(f'{mark} {n:<18} {t}')
        sys.exit(0)
    asyncio.run(main(a.force))
