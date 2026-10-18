#!/usr/bin/env python3
# 離線把 named_poses 點位畫在地圖（pgm）上，輸出 PNG，不需要 ROS。
#
#   python3 plot_named_poses.py                  # field_config.yaml 所有 side 各一張
#   python3 plot_named_poses.py mirror dit -o ~/maps_png
#   python3 plot_named_poses.py normal --show    # 開視窗看（可縮放）
#   python3 plot_named_poses.py dit --full-map   # 整張地圖（預設只放大到點位附近）
#
# 座標以 world frame（場地左下角）為準，地圖用 field_config.yaml 的
# world_to_map 擺到 world 上，跟 Nav2 實際用的轉換一樣。

import argparse
import math
import os
import re
from pathlib import Path
import sys

import yaml
import matplotlib
from PIL import Image

from named_poses_common import (
    all_sides, group_coincident, load_named_poses, load_side, remap_path)


def load_map(map_yaml, ws_root):
    with open(map_yaml) as f:
        m = yaml.safe_load(f)
    image = remap_path(m['image'], ws_root)
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(map_yaml), image)
    img = Image.open(image).convert('L')
    if m.get('negate', 0):
        img = Image.eval(img, lambda v: 255 - v)
    return img, float(m['resolution']), m['origin']


def fsm_trajectories(side, poses, full_arm=False):
    """名目軌跡：車體速度旋轉到 world；讀目前 src 常數，不代表實測路徑。"""
    workspace = Path(__file__).resolve().parents[3]
    headers = workspace / 'src/robot_fsm/include/robot_fsm/stages'
    def constants(filename):
        text = (headers / filename).read_text()
        return {name: float(value) for name, value in re.findall(
            r'constexpr double\s+(\w+)\s*=\s*([-+0-9.eE]+)\s*;', text)}
    s2 = constants('stage2_clam_fsm.hpp')
    s3 = constants('stage3_hay_fsm.hpp')
    with open(os.path.join(side['ws_root'], 'field_config.yaml')) as f:
        config = yaml.safe_load(f)['sides'][side['side']]
    with open(remap_path(config['fsm_side_file'], side['ws_root'])) as f:
        sign = float(yaml.safe_load(f).get('stage2_push_clam_x_sign', 1.0))
    points = {p[0]: p[1:4] for p in poses}
    paths = []
    def timed(name, start, vx, vy, duration):
        x, y, yaw = start
        dx = duration * (vx * math.cos(yaw) - vy * math.sin(yaw))
        dy = duration * (vx * math.sin(yaw) + vy * math.cos(yaw))
        end = (x + dx, y + dy, yaw)
        distance = math.hypot(dx, dy)
        paths.append((name, start, end,
                      f'{distance:.2f} m; vx={vx:g}, vy={vy:g}; {duration:g} s', False))
        return end
    push_end = timed('S2_PUSH_CLAM', points['stage2_entry'],
                     s2['kPushClamLinearX'] * sign, s2['kPushClamLinearY'],
                     s2['kPushClamDurationSec'])
    if full_arm:
        timed('S2_ALLIGN_LOCK', points['stage2_move_forward_align'],
              s2['kAlignLockLinearX'], s2['kAlignLockLinearY'],
              s2['kAlignLockDurationSec'])
    else:
        timed('S2_SHIFT_TO_RETURN (after LIFT_ARM)', push_end, 0.0,
              s2['kShiftToReturnLinearY'], s2['kShiftToReturnDurationSec'])
    timed('S3_CROSS', points['stage3_entry'], s3['kCrossLinearX'],
          s3['kCrossLinearY'], s3['kCrossDurationSec'])
    timed('S3_AFTER_PLACE', points['stage3_place'], s3['MoveBackLinearX'],
          0.0, s3['MoveBackDuration'])
    # 掃描／退回／換排是感知閉迴路；畫參考線，不當作固定速度積分。
    for row in range(4, 0, -1):
        start, end = points[f'stage3_row{row}_start'], points[f'stage3_row{row}_end']
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        paths.append((f'S3_SWEEP row{row}', start, end,
                      f'{length:.2f} m ref.; vx={s3["kSweepFastSpeed"]:g}/{s3["kSweepSlowSpeed"]:g}', True))
    notes = ('NAV_TO_PLACE: stage3_place_approach (turn 180 deg) -> stage3_place.\n'
             'ROW_RETURN: reverse along row; ROW_SHIFT: lateral to next row.\n'
             'BACKOFF: sensor-dependent; no fixed length. Solid = timed nominal; dashed = row reference.')
    return paths, notes


def draw_trajectories(ax, paths):
    renderer = ax.figure.canvas.get_renderer()
    obstacles = [artist.get_window_extent(renderer) for artist in ax.texts
                 if artist.get_bbox_patch() is not None]
    frame = ax.get_window_extent(renderer)
    for name, start, end, detail, reference in paths:
        color = 'seagreen' if reference else 'purple'
        ax.annotate('', xy=end[:2], xytext=start[:2],
                    arrowprops=dict(arrowstyle='-|>', color=color, lw=2,
                                    linestyle='--' if reference else '-', mutation_scale=14),
                    zorder=5)
        midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
        best = None
        for ox, oy, ha, va in LABEL_CANDIDATES:
            ann = ax.annotate(f'{name}\n{detail}', midpoint,
                              xytext=(ox, oy), textcoords='offset points', ha=ha, va=va,
                              fontsize=7, color=color,
                              bbox=dict(fc='white', ec=color, alpha=0.85, boxstyle='round,pad=0.2'),
                              arrowprops=dict(arrowstyle='-', color=color, lw=0.6), zorder=6)
            ann.get_window_extent(renderer)
            box = ann.get_bbox_patch().get_window_extent(renderer)
            def intersection(other):
                return (max(0, min(box.x1, other.x1) - max(box.x0, other.x0)) *
                        max(0, min(box.y1, other.y1) - max(box.y0, other.y0)))
            cost = sum(intersection(o) for o in obstacles)
            cost += 10 * (box.width * box.height - intersection(frame))
            if best is None or cost < best[0]:
                if best is not None:
                    best[1].remove()
                best = (cost, ann, box)
            else:
                ann.remove()
            if cost == 0:
                break
        obstacles.append(best[2])


# 標籤候選位置（offset points, ha, va），依序嘗試，選第一個不跟其他標籤 / 點重疊的
LABEL_CANDIDATES = [(6, 6, 'left', 'bottom'), (6, -6, 'left', 'top'),
                    (-6, 6, 'right', 'bottom'), (-6, -6, 'right', 'top'),
                    (6, 28, 'left', 'bottom'), (6, -28, 'left', 'top'),
                    (-6, 28, 'right', 'bottom'), (-6, -28, 'right', 'top'),
                    (40, 0, 'left', 'center'), (-40, 0, 'right', 'center'),
                    (20, 55, 'left', 'bottom'), (20, -55, 'left', 'top'),
                    (-20, 55, 'right', 'bottom'), (-20, -55, 'right', 'top')]


def place_labels(ax, groups, yaw_by_name):
    renderer = ax.figure.canvas.get_renderer()
    obstacles = []
    for x, y, members in groups:
        for _name, px, py in members:
            dx, dy = ax.transData.transform((px, py))
            obstacles.append((dx - 4, dy - 4, dx + 4, dy + 4))

    frame = ax.get_window_extent(renderer)

    def area(b, o):
        return (max(0, min(b.x1, o[2]) - max(b.x0, o[0])) *
                max(0, min(b.y1, o[3]) - max(b.y0, o[1])))

    def overlap(b):
        # 跟其他標籤 / 點重疊的面積 + 超出圖框的面積（加重，避免被切掉）
        outside = b.width * b.height - area(b, (frame.x0, frame.y0, frame.x1, frame.y1))
        return sum(area(b, o) for o in obstacles) + 10 * outside

    for x, y, members in groups:
        text = '\n'.join(
            f'{name} ({px:.2f}, {py:.2f}), yaw={yaw_by_name[name]:.3f} rad'
            for name, px, py in members)
        best = None
        for ox, oy, ha, va in LABEL_CANDIDATES:
            ann = ax.annotate(text, (x, y), xytext=(ox, oy), textcoords='offset points',
                              ha=ha, va=va, fontsize=7,
                              bbox=dict(boxstyle='round,pad=0.2', fc='white',
                                        ec='darkorange', alpha=0.85),
                              arrowprops=dict(arrowstyle='-', color='darkorange', lw=0.6))
            box = ann.get_window_extent(renderer)
            cost = overlap(box)
            if best is None or cost < best[0]:
                if best is not None:
                    best[1].remove()
                best = (cost, ann, box)
            else:
                ann.remove()
            if cost == 0:
                break
        obstacles.append((best[2].x0, best[2].y0, best[2].x1, best[2].y1))


def plot_side(side_name, args):
    import matplotlib.pyplot as plt
    from matplotlib.transforms import Affine2D

    side = load_side(side_name, args.field_config)
    img, res, origin = load_map(side['map_yaml'], side['ws_root'])
    poses, errors = load_named_poses(side['named_poses_file'])
    for e in errors:
        print(f'[{side_name}] ERROR: {e}', file=sys.stderr)

    paths, trajectory_notes = fsm_trajectories(side, poses, args.stage2_arm)
    w2m = side['world_to_map']
    # 地圖像素 -> map frame（map yaml origin）-> world frame（world_to_map）
    map_to_world = (Affine2D()
                    .rotate(float(origin[2])).translate(float(origin[0]), float(origin[1]))
                    .rotate(float(w2m['yaw'])).translate(float(w2m['x']), float(w2m['y'])))

    fig, ax = plt.subplots(figsize=(args.size, args.size))
    width, height = img.size[0] * res, img.size[1] * res
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, origin='upper',
              extent=(0, width, 0, height), interpolation='nearest',
              transform=map_to_world + ax.transData)

    # 顯示範圍：預設是所有點位 + 原點外擴 margin；--full-map 則包含整張地圖
    xs = [p[1] for p in poses] + [0.0]
    ys = [p[2] for p in poses] + [0.0]
    xs += [point[0] for path in paths for point in (path[1], path[2])]
    ys += [point[1] for path in paths for point in (path[1], path[2])]
    margin = args.margin
    if args.full_map:
        corners = map_to_world.transform([(0, 0), (width, 0), (0, height), (width, height)])
        xs += [c[0] for c in corners]
        ys += [c[1] for c in corners]
        margin = 0.3
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)
    ax.set_aspect('equal')

    # world 原點 + x/y 軸
    L = args.axis_length
    arrow = dict(width=0.02, head_width=0.08, head_length=0.1, length_includes_head=True)
    ax.arrow(0, 0, L, 0, color='red', **arrow)
    ax.arrow(0, 0, 0, L, color='green', **arrow)
    ax.text(L * 1.05, 0, 'x', color='red', fontsize=12, weight='bold', va='center')
    ax.text(0, L * 1.05, 'y', color='green', fontsize=12, weight='bold', ha='center')
    ax.plot(0, 0, 'o', color='royalblue', markersize=8)
    ax.annotate('world (0,0)', (0, 0), xytext=(-6, -14), textcoords='offset points',
                color='royalblue', fontsize=9, ha='right')

    ax.set_xlabel('world x [m]')
    ax.set_ylabel('world y [m]')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.set_title(f"{side_name}: {os.path.basename(side['map_yaml'])} + "
                 f"{os.path.basename(side['named_poses_file'])}\n"
                 f"world_to_map = ({w2m['x']}, {w2m['y']}, yaw {w2m['yaw']})", fontsize=10)
    fig.text(0.02, 0.01, trajectory_notes, fontsize=7)
    fig.tight_layout(rect=(0, 0.065, 1, 1))

    # 點位：每個點畫一個圓點，靠很近的點合併成一個標籤
    for _name, x, y, yaw, _rel in poses:
        # yaw 已解析為 world frame：0 朝 +x，正角度逆時針。
        ax.annotate('', xy=(x + args.yaw_length * math.cos(yaw),
                            y + args.yaw_length * math.sin(yaw)),
                    xytext=(x, y),
                    arrowprops=dict(arrowstyle='-|>', color='dodgerblue',
                                    lw=1.5, mutation_scale=12), zorder=4)
        ax.plot(x, y, 'o', color='darkorange', markersize=6, markeredgecolor='black')
    place_labels(ax, group_coincident(poses), {p[0]: p[3] for p in poses})
    draw_trajectories(ax, paths)


    if args.show:
        plt.show()
    else:
        os.makedirs(args.out_dir, exist_ok=True)
        out = os.path.join(args.out_dir, f'named_poses_{side_name}.png')
        fig.savefig(out, dpi=args.dpi)
        print(f'[{side_name}] {len(poses)} poses -> {out}')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='Plot named_poses on the field map.')
    parser.add_argument('sides', nargs='*', help='normal / mirror / dit（預設全部）')
    parser.add_argument('-o', '--out-dir', default='.', help='PNG 輸出目錄（預設目前目錄）')
    parser.add_argument('--show', action='store_true', help='開視窗顯示，不存檔')
    parser.add_argument('--field-config', help='field_config.yaml 路徑（預設自動找）')
    parser.add_argument('--full-map', action='store_true', help='顯示整張地圖（預設只放大到點位附近）')
    parser.add_argument('--margin', type=float, default=1.5, help='點位外擴的顯示範圍（m）')
    parser.add_argument('--axis-length', type=float, default=1.0, help='原點 x/y 軸長度（m）')
    parser.add_argument('--yaw-length', type=float, default=0.3,
                        help='點位 yaw 方向箭頭長度（m，預設0.3）')
    parser.add_argument('--stage2-arm', action='store_true',
                        help='畫完整 arm 第二關（預設 skip_stage2_arm=true 縮減流程）')
    parser.add_argument('--size', type=float, default=12, help='圖片邊長（inch）')
    parser.add_argument('--dpi', type=int, default=150)
    args = parser.parse_args()

    if not args.show:
        matplotlib.use('Agg')
    for side_name in args.sides or all_sides(args.field_config):
        plot_side(side_name, args)


if __name__ == '__main__':
    main()
