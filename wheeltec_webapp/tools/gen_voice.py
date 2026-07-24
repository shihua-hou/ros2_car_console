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
  python3 tools/gen_voice.py
然后 colcon build --packages-select wheeltec_webapp 让文件进 install/。
"""
import asyncio
import os
import edge_tts

VOICE = "zh-CN-XiaoxiaoNeural"   # 自然女声; 备选 zh-CN-YunxiNeural(男)
OUT_DIR = os.path.join(os.path.dirname(__file__), '..',
                       'wheeltec_webapp', 'static', 'voice')

# (文件名(不含扩展名), 要念的中文) —— 文件名须与 index.html 里 VOICE_FILES 一致
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


async def gen_one(name, text):
    path = os.path.join(OUT_DIR, name + '.mp3')
    await edge_tts.Communicate(text, VOICE).save(path)
    print(f'  {name}.mp3  <-  "{text}"')


async def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f'生成到: {os.path.abspath(OUT_DIR)}  (voice={VOICE})')
    for name, text in PHRASES:
        await gen_one(name, text)
    print(f'完成, 共 {len(PHRASES)} 个片段。')


if __name__ == '__main__':
    asyncio.run(main())
