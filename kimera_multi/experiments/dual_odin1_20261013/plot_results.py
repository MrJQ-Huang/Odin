"""Render the recorded dual-Odin results; never modifies experiment inputs."""
from pathlib import Path
import json
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'artifacts/dual_odin1_20261013'
OUT = BASE / 'visualization'
OUT.mkdir(exist_ok=True)
font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font_path)
plt.rcParams.update({'font.family': font_manager.FontProperties(fname=font_path).get_name(),
                     'axes.unicode_minus': False, 'font.size': 11,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.titleweight': 'bold', 'figure.facecolor': '#f4f7fb',
                     'axes.facecolor': 'white', 'grid.color': '#dce3ed'})
validation = json.loads((BASE / 'validation.json').read_text())
series = {}
metrics = {}
for slot, color in [('a', '#167cbd'), ('b', '#b057cc')]:
    name = 'odin_' + slot
    result = json.loads((BASE / 'docker_motion_usb128_fixed_pair' / name / 'summary.json').read_text())
    assert result['success'], 'Only render successful final runs'
    pose, reference = result['odometry'], result['reference_odometry']
    position = np.array([row['position'] for row in pose])
    position -= position[0]
    ref_position = np.array([row['position'] for row in reference])
    ref_position -= ref_position[0]
    q = np.array([row['quaternion_xyzw'] for row in pose])
    q /= np.linalg.norm(q, axis=1)[:, None]
    rotation = np.rad2deg(2 * np.arccos(np.clip(np.abs(q @ q[0]), 0, 1)))
    start = reference[0]['stamp']
    t = np.array([row['stamp'] - start for row in pose])
    ref_t = np.array([row['stamp'] - start for row in reference])
    radius = np.linalg.norm(position, axis=1)
    ref_radius = np.linalg.norm(ref_position, axis=1)
    series[slot] = dict(name=name, color=color, result=result, position=position,
                        t=t, ref_t=ref_t, radius=radius, ref_radius=ref_radius,
                        rotation=rotation)
    metrics[name] = {'vio_count': len(pose), 'vio_span_s': float(t[-1]-t[0]),
                     'vio_max_radius_m': float(radius.max()),
                     'vio_max_rotation_deg': float(rotation.max()),
                     'reference_max_radius_m': float(ref_radius.max()),
                     'vio_end_radius_m': float(radius[-1]),
                     'reference_end_radius_m': float(ref_radius[-1])}

fig, axes = plt.subplots(3, 2, figsize=(13, 11.8),
                          gridspec_kw={'height_ratios': [1.15, 1, 0.8]})
fig.suptitle('双 Odin1 实测结果｜分别运动，同一主机并发回放',
             fontsize=21, fontweight='bold', x=.05, ha='left', y=.985)
fig.text(.05, .945, '2026-10-13  ·  完整运动片段约 103 秒  ·  两路 VIO / LCD 均正常退出',
         fontsize=12, color='#375266')
for col, (slot, data) in enumerate(series.items()):
    pos, color = data['position'], data['color']
    ax = axes[0, col]
    ax.plot(pos[:, 0], pos[:, 1], color=color, lw=1.6, alpha=.65)
    scatter = ax.scatter(pos[:, 0], pos[:, 1], c=data['t'], cmap='viridis', s=11, zorder=3)
    ax.scatter(*pos[0, :2], s=90, marker='o', color='#008c6a', label='起点', zorder=4)
    ax.scatter(*pos[-1, :2], s=100, marker='X', color='#c43e36', label='终点', zorder=4)
    ax.set(title=f'Odin {slot.upper()}：VIO 轨迹的 XY 投影', xlabel='本机 X (m)', ylabel='本机 Y (m)')
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-1.45, .65)
    ax.set_ylim(-1.25, .6)
    ax.grid(alpha=.75)
    ax.legend(loc='upper right', fontsize=9)
    fig.colorbar(scatter, ax=ax, fraction=.035, pad=.025, label='相对时间 (s)')

    ax = axes[1, col]
    ax.plot(data['t'], data['radius'], color=color, lw=2, label='Kimera VIO')
    ax.plot(data['ref_t'], data['ref_radius'], color='#687787', ls='--', lw=1.6,
            label='Odin SDK 参考里程计')
    ax.set(title=f'Odin {slot.upper()}：距各自起点的三维距离',
           xlabel='片段相对时间 (s)', ylabel='距离 (m)', xlim=(0, 104), ylim=(0, 1.4))
    ax.grid(alpha=.75)
    ax.legend(loc='upper left', fontsize=9)

ax = axes[2, 0]
for slot, data in series.items():
    ax.plot(data['t'], data['rotation'], color=data['color'], lw=1.8,
            label='Odin ' + slot.upper())
ax.set(title='VIO 相对初始朝向的旋转角', xlabel='片段相对时间 (s)',
       ylabel='主旋转角 (°)', xlim=(0, 104), ylim=(0, 185))
ax.grid(alpha=.75)
ax.legend(fontsize=9)

ax = axes[2, 1]
ax.axis('off')
ax.set_title('完整运动回放输出', loc='left', pad=12)
table_rows = []
for label, key in [('VIO 位姿', 'odometry_count'), ('位姿图节点', 'last_pose_graph'),
                   ('BoW 查询', 'bow_queries'), ('VLC 关键帧', 'vlc_frames')]:
    values = []
    for data in series.values():
        value = data['result'][key]
        values.append(str(value['nodes'] if isinstance(value, dict) else value))
    table_rows.append([label] + values)
table_rows += [['机器人编号', '0', '1'], ['进程退出', '全部正常', '全部正常']]
table = ax.table(cellText=table_rows, colLabels=['指标', 'Odin A', 'Odin B'],
                 cellLoc='center', loc='center', bbox=[0, -.05, 1, 1.02])
table.auto_set_font_size(False)
table.set_fontsize(11)
for (row, col), cell in table.get_celld().items():
    cell.set_edgecolor('#e2e8f0')
    cell.set_facecolor('#eaf0f8' if row == 0 else 'white')
    if row == 0: cell.set_text_props(weight='bold')
fig.text(.05, .035, '每台轨迹使用自己的坐标系，未做跨机对齐。SDK 仅作参考，距离曲线差异不是 ATE 精度误差。',
         fontsize=10, color='#435568')
fig.text(.05, .014, '本次验证数据接入与双前端运行；尚未验证跨机器人回环、分布式优化或轨迹精度。',
         fontsize=10, color='#435568')
fig.subplots_adjust(top=.895, bottom=.09, left=.075, right=.95, hspace=.43, wspace=.27)
fig.savefig(OUT / 'dual_odin1_results.png', dpi=170)
fig.savefig(OUT / 'dual_odin1_results.pdf')
plt.close(fig)

# A separate 3D figure avoids silently discarding vertical motion.
fig = plt.figure(figsize=(12, 5.5))
fig.suptitle('双 Odin1 的三维 VIO 轨迹（各自坐标系）', fontsize=18, fontweight='bold')
for col, (slot, data) in enumerate(series.items()):
    ax = fig.add_subplot(1, 2, col + 1, projection='3d')
    pos = data['position']
    ax.plot(*pos.T, color=data['color'], lw=1.8)
    ax.scatter(*pos[0], color='#008c6a', s=55, label='起点')
    ax.scatter(*pos[-1], color='#c43e36', s=65, marker='X', label='终点')
    ax.set(title='Odin '+slot.upper(), xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)')
    ax.set_xlim(-1.45, .65); ax.set_ylim(-1.25, .6); ax.set_zlim(-.8, .9)
    ax.set_box_aspect((2.1, 1.85, 1.7)); ax.legend(loc='upper right', fontsize=9)
fig.text(.08, .025, '独立局部轨迹；不是两机融合地图，也不是精度验收结果。', fontsize=10, color='#435568')
fig.subplots_adjust(top=.88, bottom=.12, left=.02, right=.98, wspace=.05)
fig.savefig(OUT / 'dual_odin1_3d.png', dpi=170)
plt.close(fig)
(OUT / 'plotted_metrics.json').write_text(json.dumps(metrics, indent=2, ensure_ascii=False)+'\n')
print(OUT)
