"""从随报告归档的诊断 JSON 重绘项目流程和检索诊断图。需要 Pillow。"""
from pathlib import Path
import json
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
INK, MUTED = '#152b43', '#52677e'


def font(size):
    return ImageFont.truetype(FONT, size)


def text(draw, xy, value, size=28, fill=INK, anchor=None):
    draw.text(xy, value, font=font(size), fill=fill, anchor=anchor)


def box(draw, rect, lines, fill='#e7f3ef', outline='#388773'):
    draw.rounded_rectangle(rect, radius=18, fill=fill, outline=outline, width=3)
    x0, y0, x1, y1 = rect
    step = 40
    for i, line in enumerate(lines):
        text(draw, ((x0+x1)/2, (y0+y1)/2+(i-(len(lines)-1)/2)*step), line, 28, anchor='mm')


def arrow(draw, start, end, fill='#536d87', both=False):
    import math
    draw.line([start, end], fill=fill, width=5)
    def head(tip, origin):
        a = math.atan2(tip[1]-origin[1], tip[0]-origin[0])
        draw.polygon([tip, (tip[0]-19*math.cos(a-.45), tip[1]-19*math.sin(a-.45)),
                     (tip[0]-19*math.cos(a+.45), tip[1]-19*math.sin(a+.45))], fill=fill)
    head(end, start)
    if both:
        head(start, end)


def architecture():
    im = Image.new('RGB', (2000, 1110), '#f4f7fb')
    d = ImageDraw.Draw(im)
    text(d, (65, 25), '双 Odin1 / 双 PC：当前实现与联调位置', 48)
    text(d, (65, 98), '主路径：SDK 本地定位 + RGB / 实测深度关键帧 → Kimera-Multi 分布式模块', 29, MUTED)
    for x, title, addr in [(65, 'Odin A · x86_64', '192.168.5.21 · robot_id = 0'),
                           (1125, 'Odin B · Jetson AGX Thor / ARM64', '192.168.5.9 · robot_id = 1')]:
        text(d, (x+400, 180), title, 32, anchor='mm')
        text(d, (x+400, 222), addr, 25, MUTED, 'mm')
        box(d, (x,260,x+800,350), ['本地 USB：RGB、原始 DTOF 点云、IMU、SDK 位姿'])
        arrow(d, (x+400,350), (x+400,385))
        box(d, (x,385,x+800,490), ['ROS 1 适配：去畸变、逐点运动补偿、深度投影', '原始 SDK 位姿同步转发；使用本设备时间戳'])
        arrow(d, (x+400,490), (x+400,525))
        box(d, (x,525,x+800,630), ['SDK 前端：相机位姿图 + ORB / BoW / VLC', '整幅 RGB 用于检索；3D 特征要求有效测量深度'])
        arrow(d, (x+400,630), (x+400,665))
        box(d, (x,665,x+800,770), ['分布式回环节点 + DPGO 节点已部署运行', '跨机回环与共同坐标系初始化：尚未通过'], '#fff0da', '#bf8535')
    arrow(d, (865,717), (1125,717), both=True)
    text(d, (995,645), 'Wi-Fi', 30, anchor='mm')
    text(d, (995,680), 'BoW / 查询 / 状态', 22, anchor='mm')
    box(d, (65,825,1925,905), ['共用 ROS master（A 端）：注册与发现；两端分别处理本地数据并运行算法'], '#e8eef8', '#6784a6')
    box(d, (65,940,1925,1020), ['下一验收点：有效跨机回环 → 全局坐标初始化 → 联合图优化 → 统一地图'], '#f1f1f1', '#929aa3')
    text(d, (65,1050), '按现有实现整理；数据来源见报告。节点运行与消息互通已有记录，端到端融合效果仍待验证。', 25, MUTED)
    im.save(ROOT/'01_当前双机系统流程.png')


def retrieval():
    paths = [('运动 / 10月13日', 'motion_retrieval.json'),
             ('静态重叠 / 10月13日', 'static_retrieval.json'),
             ('椅子与背包 / 10月14日', 'reference_retrieval.json')]
    im = Image.new('RGB', (1800, 850), '#f4f7fb')
    d = ImageDraw.Draw(im)
    text(d, (65,30), '跨机回环当前卡点：检索分数未达到候选阈值', 44)
    text(d, (65,102), '实际 BoW 数据库的离线诊断上界；横轴为相对前一帧的 NSS 归一化比值', 28, MUTED)
    left, width, maxv = 460, 1050, .6
    for n in range(7):
        x = left + width*n/6
        d.line([(x,190),(x,660)], fill='#d4dce6', width=2)
        text(d, (x,690), f'{n/10:.1f}', 25, MUTED, 'mm')
    cutoff = left+width*.5/maxv
    for y in range(190,661,22):
        d.line([(cutoff,y),(cutoff,min(y+12,660))], fill='#c56625', width=4)
    text(d, (cutoff,168), 'alpha = 0.5', 28, '#ac4f18', 'mm')
    for i,(label,name) in enumerate(paths):
        j=json.loads((ROOT/'evidence'/name).read_text())
        ratio=j['max_cross_to_previous_nss_ratio']
        y=250+i*165
        text(d, (420,y), label, 29, anchor='rm')
        d.rounded_rectangle((left,y-25,left+width*ratio/maxv,y+25), radius=9, fill='#43869d')
        text(d, (left+width*ratio/maxv+16,y), f'{ratio:.4f}', 29, anchor='lm')
        text(d, (left,y+57), f"原始最高跨机 BoW 分数 {j['max_cross_score']:.5f}  ·  达到 alpha 的方向候选对数 {j['eligible_pairs_directions_above_alpha']}", 24, MUTED, 'lm')
    text(d, (65,756), '诊断含义：当前数据在检索阶段已无合格候选；原始 BoW 分数与 NSS 比值采用不同尺度。', 27)
    text(d, (65,805), '这张图不代表几何验证成功，也不构成降低阈值后即可融合的证据。', 25, MUTED)
    im.save(ROOT/'09_跨机检索诊断.png')


if __name__ == '__main__':
    architecture()
    retrieval()
