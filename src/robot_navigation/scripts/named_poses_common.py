# named_poses_markers.py（live）和 plot_named_poses.py（離線出圖）共用：
#   讀 field_config.yaml -> 某個 side 的地圖 / named_poses / world_to_map，
#   並把 named_poses 解析成 world frame 的 (x, y, yaw)。
#
# 不 import rclpy / matplotlib，container 內外都能用。
# ⚠ 相對點位的算法要跟 src/navigation_server.cpp 的 resolve_named_pose 一致。

import math
import os

import yaml

CONTAINER_WS = '/workspaces'


def find_field_config(explicit=None):
    """container 內是 /workspaces/field_config.yaml；host 上往上找。"""
    if explicit:
        return explicit
    default = os.path.join(CONTAINER_WS, 'field_config.yaml')
    if os.path.isfile(default):
        return default
    d = os.path.dirname(os.path.realpath(__file__))
    while d != os.path.dirname(d):
        candidate = os.path.join(d, 'field_config.yaml')
        if os.path.isfile(candidate):
            return candidate
        d = os.path.dirname(d)
    raise FileNotFoundError('field_config.yaml not found (use --field-config)')


def remap_path(path, ws_root):
    """field_config / map yaml 裡寫的是 container 路徑 /workspaces/...，
    在 host 上跑時換成實際的 workspace 目錄。"""
    if os.path.exists(path) or not path.startswith(CONTAINER_WS + '/'):
        return path
    return os.path.join(ws_root, path[len(CONTAINER_WS) + 1:])


def load_side(side, field_config_file=None):
    """回傳 dict：side, map_yaml, named_poses_file, world_to_map{x,y,yaw}"""
    field_config_file = find_field_config(field_config_file)
    ws_root = os.path.dirname(os.path.realpath(field_config_file))
    with open(field_config_file) as f:
        cfg = yaml.safe_load(f)

    side = side or cfg['default_side']
    if side not in cfg['sides']:
        raise KeyError(f"field_side '{side}' not in {field_config_file} sides "
                       f"{list(cfg['sides'])}")
    s = cfg['sides'][side]
    maps_dir = remap_path(cfg['maps_dir'], ws_root)
    return {
        'side': side,
        'ws_root': ws_root,
        'map_yaml': os.path.join(maps_dir, s['map_name'] + '.yaml'),
        'named_poses_file': remap_path(s['named_poses_file'], ws_root),
        'world_to_map': s.get('world_to_map', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}),
    }


def all_sides(field_config_file=None):
    with open(find_field_config(field_config_file)) as f:
        return list(yaml.safe_load(f)['sides'])


def _resolve(poses, name, depth=0):
    if depth > 8:
        raise ValueError(f"relative_to chain too deep (circular?) at '{name}'")
    p = poses.get(name)
    if not isinstance(p, dict):
        raise ValueError(f"pose '{name}' not found")

    if 'relative_to' in p:
        if 'dx' not in p or 'dy' not in p:
            raise ValueError(f"relative pose '{name}' missing dx/dy")
        ax, ay, ayaw = _resolve(poses, p['relative_to'], depth + 1)
        if 'x_relative_to' in p:
            ax = _resolve(poses, p['x_relative_to'], depth + 1)[0]
        dx, dy = float(p['dx']), float(p['dy'])
        dyaw = float(p.get('dyaw', 0.0))
        yaw = math.atan2(math.sin(ayaw + dyaw), math.cos(ayaw + dyaw))
        return ax + dx, ay + dy, yaw

    if 'x' not in p or 'y' not in p or 'yaw' not in p:
        raise ValueError(f"pose '{name}' missing x/y/yaw")
    return float(p['x']), float(p['y']), float(p['yaw'])


def load_named_poses(named_poses_file):
    """回傳 (poses, errors)：poses = [(name, x, y, yaw, is_relative)]（world frame，照檔案順序）"""
    with open(named_poses_file) as f:
        raw = (yaml.safe_load(f) or {}).get('named_poses') or {}

    poses, errors = [], []
    for name, p in raw.items():
        try:
            x, y, yaw = _resolve(raw, name)
        except ValueError as e:
            errors.append(str(e))
            continue
        poses.append((name, x, y, yaw, isinstance(p, dict) and 'relative_to' in p))
    return poses, errors


def group_coincident(poses, tol=0.05):
    """距離 tol 內的點合併成一組，標籤才不會疊在一起。
    回傳 [(x, y, [(name, px, py)])]，(x, y) 是組內第一個點。"""
    groups = []
    for name, x, y, _yaw, _rel in poses:
        for g in groups:
            if math.hypot(g[0] - x, g[1] - y) < tol:
                g[2].append((name, x, y))
                break
        else:
            groups.append((x, y, [(name, x, y)]))
    return groups
