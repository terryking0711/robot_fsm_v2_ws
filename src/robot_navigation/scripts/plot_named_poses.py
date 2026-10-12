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
import os
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


# 標籤候選位置（offset points, ha, va），依序嘗試，選第一個不跟其他標籤 / 點重疊的
LABEL_CANDIDATES = [(6, 6, 'left', 'bottom'), (6, -6, 'left', 'top'),
                    (-6, 6, 'right', 'bottom'), (-6, -6, 'right', 'top'),
                    (6, 28, 'left', 'bottom'), (6, -28, 'left', 'top'),
                    (-6, 28, 'right', 'bottom'), (-6, -28, 'right', 'top'),
                    (40, 0, 'left', 'center'), (-40, 0, 'right', 'center'),
                    (20, 55, 'left', 'bottom'), (20, -55, 'left', 'top'),
                    (-20, 55, 'right', 'bottom'), (-20, -55, 'right', 'top')]


def place_labels(ax, groups):
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
        text = '\n'.join(f'{name} ({px:.2f}, {py:.2f})' for name, px, py in members)
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
    fig.tight_layout()

    # 點位：每個點畫一個圓點，靠很近的點合併成一個標籤
    for _name, x, y, _yaw, _rel in poses:
        ax.plot(x, y, 'o', color='darkorange', markersize=6, markeredgecolor='black')
    place_labels(ax, group_coincident(poses))


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
    parser.add_argument('--size', type=float, default=12, help='圖片邊長（inch）')
    parser.add_argument('--dpi', type=int, default=150)
    args = parser.parse_args()

    if not args.show:
        matplotlib.use('Agg')
    for side_name in args.sides or all_sides(args.field_config):
        plot_side(side_name, args)


if __name__ == '__main__':
    main()
