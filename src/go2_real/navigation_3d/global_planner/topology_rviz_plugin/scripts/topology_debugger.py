#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import heapq
import math
import threading
import textwrap
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons
from collections import OrderedDict, defaultdict

try:
    import rospy
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PointStamped, Pose
    ROS_AVAILABLE = True
except Exception:
    ROS_AVAILABLE = False

TOPO_FILE = "../map/connected_topology_map.txt"

# “异常边”判定：权重/欧氏距离比值过离谱则标红虚线
BAD_EDGE_RATIO_LOW = 0.5
BAD_EDGE_RATIO_HIGH = 2.0

# 临时 start/goal 连接边的权重模式
# EUCLID: 临时边权 = 欧氏距离（推荐）
# CONST:  临时边权 = 0（用于排查：如果这样路径还绕，说明最近点/拓扑边本身有问题）
TEMP_EDGE_MODE = "EUCLID"

# 显示与高亮配置
SHOW_EDGE_WEIGHTS = True          # 显示拓扑边权重（中点标注）
EDGE_WEIGHT_FONTSIZE = 7
HIGHLIGHT_PATH_EDGES = True       # 高亮当前最短路上的边（加粗）

# 贴近 global_path_planner 的关键参数（用于离线复现）
ENABLE_DIRECT_CONNECTION = True
DIRECT_CONNECTION_THRESHOLD = 8.0
INTERPOLATION_POINTS = 20

# 话题绘图模式默认参数
DEFAULT_PATH_TOPIC = "/smoothed_global_path"
DEFAULT_LOCAL_PATH_TOPIC = "/pure_pursuit_local_planner/local_trajectory"
DEFAULT_CLICKED_TOPIC = "/clicked_point"
DEFAULT_TARGET_TOPIC = "/target_points"
DEFAULT_UPDATE_HZ = 3.0
DEFAULT_INDEX_STEP = 5


# ----------------------------
# 解析拓扑文件
# ----------------------------
def parse_topology():
    nodes = OrderedDict()  # nid -> [x,y,z,label,pindex]
    edges = []             # (frm,to,w,etype(str),bidir)

    in_nodes = False
    in_edges = False

    with open(TOPO_FILE, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue

            if line.startswith("#"):
                if "Topology Nodes" in line:
                    in_nodes, in_edges = True, False
                elif "Topology Edges" in line:
                    in_nodes, in_edges = False, True
                continue

            parts = line.split()
            if in_nodes:
                # id x y z label path_index
                if len(parts) < 6:
                    continue
                nid = int(parts[0])
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                label = parts[4]
                pindex = int(parts[5])
                if nid not in nodes:
                    nodes[nid] = [x, y, z, label, pindex]

            elif in_edges:
                # edge_id from to weight type bidirectional
                if len(parts) < 6:
                    continue
                frm = int(parts[1])
                to = int(parts[2])
                w = float(parts[3])
                etype = parts[4]       # auto/manual/...
                bidir = int(parts[5])  # 0/1
                edges.append((frm, to, w, etype, bidir))

    return nodes, edges


def euclid_xy(nodes_dict, a, b):
    x1, y1 = nodes_dict[a][0], nodes_dict[a][1]
    x2, y2 = nodes_dict[b][0], nodes_dict[b][1]
    return float(math.hypot(x1 - x2, y1 - y2))


def build_graph(nodes_dict, edges_list):
    graph = {nid: [] for nid in nodes_dict}
    for frm, to, w, etype, bidir in edges_list:
        if frm in graph and to in graph:
            graph[frm].append((to, w))
            if bidir:
                graph[to].append((frm, w))
    return graph


def dijkstra(graph, start, goal):
    pq = [(0.0, start)]
    dist = {n: float("inf") for n in graph}
    prev = {}
    dist[start] = 0.0
    visited_order = []

    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        visited_order.append(u)
        if u == goal:
            break
        for v, w in graph[u]:
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    if start == goal:
        return [start], visited_order
    if goal not in prev:
        return [], visited_order

    path = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = prev[cur]
    path.append(start)
    path.reverse()
    return path, visited_order


def nearest_topo_node(nodes, x, y):
    best = None
    best_d2 = float("inf")
    for nid, (nx, ny, *_rest) in nodes.items():
        d2 = (nx - x) ** 2 + (ny - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = nid
    return best, math.sqrt(best_d2)


def classify_edges(nodes, edges):
    normal_edges = []
    bad_edges = []
    pair_count = defaultdict(int)

    for frm, to, w, etype, bidir in edges:
        u, v = (frm, to) if frm <= to else (to, frm)
        pair_count[(u, v)] += 1

    duplicates = [((u, v), c) for (u, v), c in pair_count.items() if c > 1]

    for frm, to, w, etype, bidir in edges:
        if frm not in nodes or to not in nodes:
            continue
        d = euclid_xy(nodes, frm, to)
        if d < 1e-6:
            bad_edges.append((frm, to, w, etype, bidir, float("inf"), d))
            continue
        ratio = w / d
        if ratio < BAD_EDGE_RATIO_LOW or ratio > BAD_EDGE_RATIO_HIGH:
            bad_edges.append((frm, to, w, etype, bidir, ratio, d))
        else:
            normal_edges.append((frm, to, w, etype, bidir))

    return normal_edges, bad_edges, duplicates


def suggest_topology_fixes(nodes, edges, k_nearest=4):
    normal_edges, bad_edges, duplicates = classify_edges(nodes, edges)
    suggestions = []

    for frm, to, w, etype, bidir, ratio, d in sorted(
        bad_edges, key=lambda x: (abs(x[5] - 1.0) if math.isfinite(x[5]) else 9999), reverse=True
    ):
        suggestions.append(
            f"[异常边] {frm}<->{to}  type={etype}  w={w:.3f}  euclid={d:.3f}  ratio(w/d)={ratio:.2f}"
        )

    for (u, v), c in duplicates:
        suggestions.append(f"[重复边] {u}<->{v} 出现 {c} 次（建议去重或统一权重）")

    edge_set = set()
    for frm, to, w, etype, bidir in edges:
        u, v = (frm, to) if frm <= to else (to, frm)
        edge_set.add((u, v))

    nids = list(nodes.keys())
    for a in nids:
        dists = []
        for b in nids:
            if a == b:
                continue
            dists.append((euclid_xy(nodes, a, b), b))
        dists.sort(key=lambda x: x[0])
        for d, b in dists[:k_nearest]:
            u, v = (a, b) if a <= b else (b, a)
            if (u, v) not in edge_set:
                suggestions.append(f"[建议补边] {a}<->{b}  euclid={d:.3f}（近邻未连，易导致绕路）")

    return suggestions


def path_to_edge_set(path):
    """把路径节点序列转换为无向边集合 {(u,v),...}，用于高亮"""
    s = set()
    if not path or len(path) < 2:
        return s
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        u, v = (a, b) if a <= b else (b, a)
        s.add((u, v))
    return s


def collect_second_layer_candidates(edges, anchor_id):
    """复现 global_path_planner::addTemporaryConnections 的候选节点收集逻辑"""
    candidates = set([anchor_id])
    for frm, to, _w, _etype, bidir in edges:
        if frm == anchor_id:
            candidates.add(to)
        elif bidir and to == anchor_id:
            candidates.add(frm)
    return candidates


def interpolate_points(p1, p2, num_points):
    """与 C++ interpolatePath 一致：每段输出 num_points+1 个点（含首尾）"""
    out = []
    if num_points <= 0:
        return out
    x1, y1 = p1
    x2, y2 = p2
    for i in range(num_points + 1):
        t = float(i) / float(num_points)
        out.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
    return out


def build_interpolated_path_xy(nodes, path, num_points):
    """根据节点序列构建插值后的全局路径点（xy）"""
    if not path or len(path) < 2:
        return []

    all_pts = []
    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        seg = interpolate_points(
            (nodes[a][0], nodes[a][1]),
            (nodes[b][0], nodes[b][1]),
            num_points,
        )
        if i == 0:
            all_pts.extend(seg)
        else:
            all_pts.extend(seg[1:])  # 避免段首重复
    return all_pts


def path_length_xy(path_xy):
    if not path_xy or len(path_xy) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path_xy) - 1):
        total += math.hypot(path_xy[i + 1][0] - path_xy[i][0], path_xy[i + 1][1] - path_xy[i][1])
    return total


def nearest_index_to_point(path_xy, point_xy):
    if not path_xy or point_xy is None:
        return None
    best_i = 0
    best_d2 = float("inf")
    gx, gy = point_xy
    for i, (x, y) in enumerate(path_xy):
        d2 = (x - gx) * (x - gx) + (y - gy) * (y - gy)
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    return best_i


def build_key_indices(n):
    if n <= 0:
        return []
    ids = {0, n - 1}
    if n > 2:
        ids.add(n // 2)
    if n > 4:
        ids.add(n // 4)
        ids.add((3 * n) // 4)
    return sorted(ids)


# ----------------------------
# 主程序（交互）
# ----------------------------
def offline_main():
    global TEMP_EDGE_MODE, ENABLE_DIRECT_CONNECTION

    topo_nodes, topo_edges = parse_topology()

    print(f"读取文件: {TOPO_FILE}")
    print(f"拓扑节点: {len(topo_nodes)}")
    print(f"拓扑边:   {len(topo_edges)}")
    print(f"临时边权模式 TEMP_EDGE_MODE = {TEMP_EDGE_MODE}（按 t 切换）")
    print(
        f"直接连接 ENABLE_DIRECT_CONNECTION={ENABLE_DIRECT_CONNECTION}, "
        f"threshold={DIRECT_CONNECTION_THRESHOLD}, interpolation={INTERPOLATION_POINTS}"
    )
    print(f"显示边权重: {SHOW_EDGE_WEIGHTS}, 高亮路径边: {HIGHLIGHT_PATH_EDGES}")

    normal_edges, bad_edges, duplicates = classify_edges(topo_nodes, topo_edges)
    if duplicates:
        print("⚠ 检测到重复边，按 o 可输出建议详情")

    # ======== 画图 ========
    fig, ax = plt.subplots(figsize=(11, 8))

    # 色条只创建一次
    cbar_holder = {"cbar": None}

    # overlay objects
    path_line = None                 # 拓扑节点连线
    global_path_line = None          # 插值后的全局路径（更接近 /global_path）
    visit_scat = None
    start_artist = None
    goal_artist = None
    start_conn_lines = []
    goal_conn_lines = []

    # 状态
    clicks = []  # [(x,y), (x,y)] -> start, goal
    last_visited = []
    current_path_edges = set()   # 用于高亮拓扑边（仅 topo 部分）

    # 保存最近一次的 temp_nodes 给动画用
    nonlocal_last = {"temp_nodes": None}

    def redraw_base():
        """重画拓扑底图（节点+边+边权重+高亮）"""
        ax.clear()

        xs = [topo_nodes[n][0] for n in topo_nodes]
        ys = [topo_nodes[n][1] for n in topo_nodes]
        cs = [topo_nodes[n][4] for n in topo_nodes]
        sc = ax.scatter(xs, ys, c=cs, cmap="viridis", s=90, zorder=3)

        # 正常边：按 type 区分（manual 实线灰，auto 虚线）
        for frm, to, w, etype, bidir in normal_edges:
            x1, y1 = topo_nodes[frm][0], topo_nodes[frm][1]
            x2, y2 = topo_nodes[to][0], topo_nodes[to][1]

            # 是否属于当前路径（无向边判定）
            u, v = (frm, to) if frm <= to else (to, frm)
            on_path = HIGHLIGHT_PATH_EDGES and ((u, v) in current_path_edges)

            linestyle = "--" if etype == "auto" else "-"
            linewidth = 3.0 if on_path else 1.0
            color = "black" if on_path else ("gray" if etype != "auto" else "gray")

            ax.plot([x1, x2], [y1, y2], linestyle=linestyle, linewidth=linewidth, color=color, zorder=1)

            # 显示边权重（中点）
            if SHOW_EDGE_WEIGHTS:
                mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                ax.text(mx, my, f"{w:.2f}", fontsize=EDGE_WEIGHT_FONTSIZE,
                        ha="center", va="center", zorder=6)

        # 异常边：红色虚线
        for frm, to, w, etype, bidir, ratio, d in bad_edges:
            if frm not in topo_nodes or to not in topo_nodes:
                continue
            x1, y1 = topo_nodes[frm][0], topo_nodes[frm][1]
            x2, y2 = topo_nodes[to][0], topo_nodes[to][1]
            ax.plot([x1, x2], [y1, y2], color="red", linestyle="--", linewidth=1.6, zorder=2)

        # 节点编号
        for nid, (x, y, z, label, pidx) in topo_nodes.items():
            ax.text(x, y, str(nid), fontsize=10, ha="left", va="bottom", zorder=4)

        ax.set_title(
            "Topology + Global Path Debugger | click 2 points: START then GOAL | "
            "keys: r reset, a animate, t temp mode, d direct, o suggest"
        )
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True)
        return sc

    sc = redraw_base()
    if cbar_holder["cbar"] is None:
        cbar_holder["cbar"] = plt.colorbar(sc, ax=ax, label="path_index")

    def clear_overlays():
        """清除 start/goal/path/animation overlay，但不清底图"""
        nonlocal path_line, global_path_line, visit_scat, start_artist, goal_artist
        nonlocal start_conn_lines, goal_conn_lines

        for obj in [path_line, global_path_line, visit_scat, start_artist, goal_artist]:
            if obj is not None:
                try:
                    obj.remove()
                except Exception:
                    pass
        for obj in start_conn_lines + goal_conn_lines:
            try:
                obj.remove()
            except Exception:
                pass

        path_line = None
        global_path_line = None
        visit_scat = None
        start_artist = None
        goal_artist = None
        start_conn_lines = []
        goal_conn_lines = []

    def temp_edge_weight():
        if TEMP_EDGE_MODE == "EUCLID":
            return None  # 用欧氏距离
        if TEMP_EDGE_MODE == "CONST":
            return 0.0
        return None

    def build_temp_graph_with_start_goal(s_xy, g_xy):
        """
        贴近 global_path_planner:
          - 新建 start/goal 临时节点
          - 每个临时节点连接“最近节点 + 最近节点的第二层候选”
          - 可选：起终点在阈值内时添加直接连接（奖励系数 0.9）
        """
        temp_nodes = OrderedDict()
        for nid, val in topo_nodes.items():
            temp_nodes[nid] = list(val)

        max_id = max(temp_nodes.keys()) if temp_nodes else 0
        start_id = max_id + 1
        goal_id = max_id + 2

        temp_nodes[start_id] = [float(s_xy[0]), float(s_xy[1]), 0.0, "START", -1]
        temp_nodes[goal_id] = [float(g_xy[0]), float(g_xy[1]), 0.0, "GOAL", -1]

        near_s, ds = nearest_topo_node(topo_nodes, s_xy[0], s_xy[1])
        near_g, dg = nearest_topo_node(topo_nodes, g_xy[0], g_xy[1])

        temp_edges = list(topo_edges)

        def connect_temp_node(temp_id, nearest_id):
            w_mode = temp_edge_weight()
            candidates = sorted(list(collect_second_layer_candidates(topo_edges, nearest_id)))
            created = []
            for nid in candidates:
                w = euclid_xy(temp_nodes, temp_id, nid) if w_mode is None else float(w_mode)
                temp_edges.append((temp_id, nid, w, "temp", 1))
                created.append((nid, w))
            return created

        start_connections = connect_temp_node(start_id, near_s)
        goal_connections = connect_temp_node(goal_id, near_g)

        direct_distance = euclid_xy(temp_nodes, start_id, goal_id)
        added_direct = False
        if ENABLE_DIRECT_CONNECTION and direct_distance <= DIRECT_CONNECTION_THRESHOLD:
            temp_edges.append((start_id, goal_id, direct_distance * 0.9, "temp_direct", 1))
            added_direct = True

        temp_graph = build_graph(temp_nodes, temp_edges)

        return (
            temp_nodes, temp_edges, temp_graph,
            start_id, goal_id,
            near_s, near_g, ds, dg,
            start_connections, goal_connections,
            added_direct, direct_distance,
        )

    def draw_temp_markers(temp_nodes, start_id, goal_id, near_s, near_g, start_connections, goal_connections):
        nonlocal start_artist, goal_artist, start_conn_lines, goal_conn_lines

        sx, sy = temp_nodes[start_id][0], temp_nodes[start_id][1]
        gx, gy = temp_nodes[goal_id][0], temp_nodes[goal_id][1]
        start_artist = ax.scatter([sx], [sy], s=140, marker="o", zorder=10)
        goal_artist = ax.scatter([gx], [gy], s=140, marker="X", zorder=10)

        for nid, _w in start_connections:
            tx, ty = topo_nodes[nid][0], topo_nodes[nid][1]
            is_nearest = (nid == near_s)
            line, = ax.plot(
                [sx, tx], [sy, ty],
                linewidth=2.8 if is_nearest else 1.2,
                linestyle="-" if is_nearest else "--",
                color="tab:blue",
                alpha=0.95 if is_nearest else 0.6,
                zorder=9,
            )
            start_conn_lines.append(line)

        for nid, _w in goal_connections:
            tx, ty = topo_nodes[nid][0], topo_nodes[nid][1]
            is_nearest = (nid == near_g)
            line, = ax.plot(
                [gx, tx], [gy, ty],
                linewidth=2.8 if is_nearest else 1.2,
                linestyle="-" if is_nearest else "--",
                color="tab:orange",
                alpha=0.95 if is_nearest else 0.6,
                zorder=9,
            )
            goal_conn_lines.append(line)

        ax.text(sx, sy, "START", fontsize=11, ha="left", va="top", zorder=11)
        ax.text(gx, gy, "GOAL", fontsize=11, ha="left", va="top", zorder=11)

    def draw_path(temp_nodes, path):
        nonlocal path_line, global_path_line
        for obj_name in ["path_line", "global_path_line"]:
            obj = path_line if obj_name == "path_line" else global_path_line
            if obj is not None:
                try:
                    obj.remove()
                except Exception:
                    pass
        path_line = None
        global_path_line = None

        if not path:
            return

        px = [temp_nodes[n][0] for n in path]
        py = [temp_nodes[n][1] for n in path]
        (path_line,) = ax.plot(px, py, color="red", linewidth=2.2, zorder=8, label="topology_path")

        interp = build_interpolated_path_xy(temp_nodes, path, INTERPOLATION_POINTS)
        if interp:
            ipx = [p[0] for p in interp]
            ipy = [p[1] for p in interp]
            (global_path_line,) = ax.plot(
                ipx, ipy, color="limegreen", linewidth=3.5, alpha=0.9, zorder=7, label="global_path_interp"
            )

    def animate_visited(temp_nodes):
        nonlocal visit_scat
        if not last_visited:
            print("⚠ 先点击两点生成一次路径，再按 a 动画")
            return

        if visit_scat is not None:
            try:
                visit_scat.remove()
            except Exception:
                pass
            visit_scat = None

        xs, ys = [], []
        for nid in last_visited:
            xs.append(temp_nodes[nid][0])
            ys.append(temp_nodes[nid][1])
            if visit_scat is not None:
                try:
                    visit_scat.remove()
                except Exception:
                    pass
            visit_scat = ax.scatter(xs, ys, s=70, zorder=12)
            fig.canvas.draw_idle()
            plt.pause(0.03)
        print("▶ 扩散动画完成（按 r 清除）")

    def on_click(event):
        nonlocal current_path_edges
        if event.inaxes != ax:
            return

        clicks.append((event.xdata, event.ydata))

        if len(clicks) == 1:
            print(f"设置 START = ({clicks[0][0]:.3f}, {clicks[0][1]:.3f})")
            return

        if len(clicks) == 2:
            s_xy = clicks[0]
            g_xy = clicks[1]

            # 清旧 overlay
            clear_overlays()

            # 构建临时图并规划
            (
                temp_nodes, temp_edges, temp_graph,
                start_id, goal_id,
                near_s, near_g, ds, dg,
                start_connections, goal_connections,
                added_direct, direct_distance,
            ) = build_temp_graph_with_start_goal(s_xy, g_xy)

            print(f"START 最近拓扑点: {near_s}  距离={ds:.3f}")
            print(f"GOAL  最近拓扑点: {near_g}  距离={dg:.3f}")
            print(f"临时边权模式: {TEMP_EDGE_MODE}")
            print(f"START 临时连接数: {len(start_connections)}")
            print(f"GOAL  临时连接数: {len(goal_connections)}")
            print(
                f"起终点直连: {'YES' if added_direct else 'NO'}  "
                f"(distance={direct_distance:.3f}, threshold={DIRECT_CONNECTION_THRESHOLD:.3f})"
            )

            path, visited = dijkstra(temp_graph, start_id, goal_id)
            last_visited.clear()
            last_visited.extend(visited)

            # 更新高亮拓扑边（只高亮 topo 部分：start/goal 临时边不在 topo_nodes 中）
            # 这里直接对整个 path 做边集合，然后 redraw_base 时会只匹配 topo_edges 的边
            current_path_edges = path_to_edge_set(path)

            # 重画底图（这样高亮/权重文字会刷新）
            redraw_base()

            if not path:
                print("❌ 规划失败：start -> goal 不可达（拓扑断裂/最近点错误/边方向问题）")
            else:
                topo_length = 0.0
                for i in range(len(path) - 1):
                    topo_length += euclid_xy(temp_nodes, path[i], path[i + 1])
                straight_dist = float(math.hypot(g_xy[0] - s_xy[0], g_xy[1] - s_xy[1]))
                detour_ratio = topo_length / straight_dist if straight_dist > 1e-6 else float("inf")
                interp_pts = build_interpolated_path_xy(temp_nodes, path, INTERPOLATION_POINTS)
                print(f"✅ 路径节点序列: {path}")
                print(f"   拓扑路径长度: {topo_length:.3f}")
                print(f"   直线距离: {straight_dist:.3f}  detour_ratio={detour_ratio:.3f}")
                print(f"   插值后路径点数: {len(interp_pts)}  (INTERPOLATION_POINTS={INTERPOLATION_POINTS})")

            # 画 start/goal + 连接线 + 路径
            draw_temp_markers(
                temp_nodes, start_id, goal_id, near_s, near_g, start_connections, goal_connections
            )
            draw_path(temp_nodes, path)
            fig.canvas.draw_idle()

            # reset clicks
            clicks.clear()
            nonlocal_last["temp_nodes"] = temp_nodes

    def on_key(event):
        nonlocal current_path_edges
        global TEMP_EDGE_MODE, ENABLE_DIRECT_CONNECTION

        if event.key == "r":
            clicks.clear()
            last_visited.clear()
            current_path_edges.clear()
            clear_overlays()
            redraw_base()
            fig.canvas.draw_idle()
            print("🔁 重置完成（清除 start/goal/路径/动画）")

        elif event.key == "a":
            tn = nonlocal_last.get("temp_nodes")
            if tn is None:
                print("⚠ 先点击两点生成一次路径，再按 a 动画")
                return
            animate_visited(tn)

        elif event.key == "t":
            TEMP_EDGE_MODE = "CONST" if TEMP_EDGE_MODE == "EUCLID" else "EUCLID"
            print(f"切换 TEMP_EDGE_MODE = {TEMP_EDGE_MODE}")
            print("提示：CONST=0 用于排查“最近点选错导致绕路”；EUCLID 更符合真实长度。")

        elif event.key == "d":
            ENABLE_DIRECT_CONNECTION = not ENABLE_DIRECT_CONNECTION
            print(
                f"切换 ENABLE_DIRECT_CONNECTION = {ENABLE_DIRECT_CONNECTION} "
                f"(DIRECT_CONNECTION_THRESHOLD={DIRECT_CONNECTION_THRESHOLD})"
            )

        elif event.key == "o":
            print("=== 拓扑优化建议（不改文件，只打印前 40 条） ===")
            sug = suggest_topology_fixes(topo_nodes, topo_edges, k_nearest=4)
            for s in sug[:40]:
                print(" -", s)
            if len(sug) > 40:
                print(f"... 共 {len(sug)} 条建议（只显示前 40 条）")

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)

    print("\n操作说明：")
    print("  - 左键点两次：第一次=START，第二次=GOAL（会按全局规划逻辑画临时连接和路径）")
    print("  - 红线=拓扑节点路径；绿线=插值后全局路径（接近 /global_path）")
    print("  - 显示所有拓扑边权重（中点标注），并高亮最短路上的拓扑边（加粗）")
    print("  - 按 a：播放 Dijkstra 扩散动画（需要先点两次生成路径）")
    print("  - 按 t：切换临时边权模式（EUCLID/CONST）")
    print("  - 按 d：切换起终点直连开关")
    print("  - 按 o：输出拓扑优化建议")
    print("  - 按 r：重置\n")

    plt.show()


def topic_mode_main(args):
    if not ROS_AVAILABLE:
        print("❌ 当前环境不可用 rospy（无法订阅 ROS 话题）")
        return

    rospy.init_node("topology_debugger_topic_mode", anonymous=True)

    lock = threading.Lock()
    state = {
        "global_path_xy": [],
        "local_path_xy": [],
        "goal_xy": None,
        "goal_source": "",
    }

    def global_path_cb(msg):
        xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with lock:
            state["global_path_xy"] = xy

    def local_path_cb(msg):
        xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with lock:
            state["local_path_xy"] = xy

    def clicked_cb(msg):
        with lock:
            state["goal_xy"] = (msg.point.x, msg.point.y)
            state["goal_source"] = args.clicked_topic

    def target_cb(msg):
        # clicked_point 优先，target_points 仅兜底
        with lock:
            if state["goal_xy"] is None or state["goal_source"] != args.clicked_topic:
                state["goal_xy"] = (msg.position.x, msg.position.y)
                state["goal_source"] = args.target_topic

    rospy.Subscriber(args.path_topic, Path, global_path_cb, queue_size=1)
    rospy.Subscriber(args.local_path_topic, Path, local_path_cb, queue_size=1)
    rospy.Subscriber(args.clicked_topic, PointStamped, clicked_cb, queue_size=1)
    rospy.Subscriber(args.target_topic, Pose, target_cb, queue_size=1)

    topo_nodes = OrderedDict()
    topo_edges = []
    normal_edges = []
    bad_edges = []
    try:
        topo_nodes, topo_edges = parse_topology()
        normal_edges, bad_edges, _ = classify_edges(topo_nodes, topo_edges)
        print(f"[topic-mode] 已加载拓扑底图: nodes={len(topo_nodes)} edges={len(topo_edges)}")
    except Exception as exc:
        print(f"[topic-mode] 加载拓扑底图失败，继续仅显示话题路径: {exc}")
        topo_nodes = OrderedDict()
        topo_edges = []
        normal_edges = []
        bad_edges = []

    fig, ax = plt.subplots(figsize=(13.5, 8))
    # 左侧: 控件；中间: 主图；右侧: 指标/说明，避免遮挡主图
    plt.subplots_adjust(left=0.24, right=0.77, top=0.95, bottom=0.07)
    rate = rospy.Rate(max(float(args.update_hz), 0.5))

    layer_visible = {
        "global": True,
        "local": True,
        "topology": bool(topo_nodes),
        "indices": True,
        "distance": True,
        "offline": False,
    }
    check_ax = fig.add_axes([0.02, 0.54, 0.19, 0.34])
    check_ax.set_title("Display Layers", fontsize=10)
    check = CheckButtons(
        check_ax,
        ["Global", "Local", "Topology", "Indices", "Distance", "Offline"],
        [
            layer_visible["global"],
            layer_visible["local"],
            layer_visible["topology"],
            layer_visible["indices"],
            layer_visible["distance"],
            layer_visible["offline"],
        ],
    )
    for t in check.labels:
        t.set_fontsize(9)

    def on_check(label):
        if label == "Global":
            layer_visible["global"] = not layer_visible["global"]
        elif label == "Local":
            layer_visible["local"] = not layer_visible["local"]
        elif label == "Topology":
            layer_visible["topology"] = not layer_visible["topology"]
        elif label == "Indices":
            layer_visible["indices"] = not layer_visible["indices"]
        elif label == "Distance":
            layer_visible["distance"] = not layer_visible["distance"]
        elif label == "Offline":
            layer_visible["offline"] = not layer_visible["offline"]

    check.on_clicked(on_check)

    metric_ax = fig.add_axes([0.79, 0.54, 0.19, 0.34])
    metric_ax.set_title("Metrics", fontsize=10)
    metric_ax.axis("off")

    hint_ax = fig.add_axes([0.79, 0.10, 0.19, 0.38])
    hint_ax.set_title("Tips", fontsize=10)
    hint_ax.axis("off")

    # 视图交互状态：滚轮缩放 + 右键拖拽旋转 + 中键拖拽平移
    camera = {
        "rot_deg": 0.0,
        "pivot": None,      # (x, y), 旋转中心（世界坐标）
        "xlim": None,       # 当前视窗
        "ylim": None,
        "drag_btn": None,   # 2=中键平移, 3=右键旋转
        "last_px": None,    # 上次像素坐标
    }

    def transform_point(pt):
        x, y = pt
        if camera["pivot"] is None or abs(camera["rot_deg"]) < 1e-9:
            return x, y
        px, py = camera["pivot"]
        th = math.radians(camera["rot_deg"])
        c = math.cos(th)
        s = math.sin(th)
        dx = x - px
        dy = y - py
        return (c * dx - s * dy + px, s * dx + c * dy + py)

    def transform_points(pts):
        if not pts:
            return []
        return [transform_point(p) for p in pts]

    def init_camera_if_needed(all_points_raw):
        if not all_points_raw:
            return
        xs = [p[0] for p in all_points_raw]
        ys = [p[1] for p in all_points_raw]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(0.5, max_x - min_x)
        span_y = max(0.5, max_y - min_y)
        pad_x = span_x * 0.08
        pad_y = span_y * 0.08
        if camera["pivot"] is None:
            camera["pivot"] = ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5)
        if camera["xlim"] is None or camera["ylim"] is None:
            camera["xlim"] = (min_x - pad_x, max_x + pad_x)
            camera["ylim"] = (min_y - pad_y, max_y + pad_y)

    def on_scroll(event):
        if event.inaxes != ax:
            return
        if camera["xlim"] is None or camera["ylim"] is None:
            return
        if event.xdata is None or event.ydata is None:
            return
        xmin, xmax = camera["xlim"]
        ymin, ymax = camera["ylim"]
        cx, cy = event.xdata, event.ydata
        scale = 0.90 if event.button == "up" else 1.10
        camera["xlim"] = (cx - (cx - xmin) * scale, cx + (xmax - cx) * scale)
        camera["ylim"] = (cy - (cy - ymin) * scale, cy + (ymax - cy) * scale)

    def on_press(event):
        if event.inaxes != ax:
            return
        if event.button in (2, 3):
            camera["drag_btn"] = event.button
            camera["last_px"] = (event.x, event.y)

    def on_release(_event):
        camera["drag_btn"] = None
        camera["last_px"] = None

    def on_motion(event):
        if camera["drag_btn"] is None or camera["last_px"] is None:
            return
        if event.inaxes != ax:
            return
        dx_pix = event.x - camera["last_px"][0]
        dy_pix = event.y - camera["last_px"][1]

        # 右键：旋转视图（绕 pivot）
        if camera["drag_btn"] == 3:
            camera["rot_deg"] = (camera["rot_deg"] + dx_pix * 0.25) % 360.0

        # 中键：平移视图
        elif camera["drag_btn"] == 2 and camera["xlim"] is not None and camera["ylim"] is not None:
            xmin, xmax = camera["xlim"]
            ymin, ymax = camera["ylim"]
            xr = xmax - xmin
            yr = ymax - ymin
            w = max(1.0, ax.bbox.width)
            h = max(1.0, ax.bbox.height)
            shift_x = -dx_pix * (xr / w)
            shift_y = -dy_pix * (yr / h)
            camera["xlim"] = (xmin + shift_x, xmax + shift_x)
            camera["ylim"] = (ymin + shift_y, ymax + shift_y)

        camera["last_px"] = (event.x, event.y)

    offline_state = {
        "clicks": [],
        "path_xy": [],
        "start_xy": None,
        "goal_xy": None,
        "path_ids": [],
        "detour_ratio": None,
    }

    def plan_offline_path_from_clicks(s_xy, g_xy):
        if not topo_nodes:
            return None

        temp_nodes = OrderedDict()
        for nid, val in topo_nodes.items():
            temp_nodes[nid] = list(val)

        max_id = max(temp_nodes.keys()) if temp_nodes else 0
        start_id = max_id + 1
        goal_id = max_id + 2
        temp_nodes[start_id] = [float(s_xy[0]), float(s_xy[1]), 0.0, "OFF_START", -1]
        temp_nodes[goal_id] = [float(g_xy[0]), float(g_xy[1]), 0.0, "OFF_GOAL", -1]

        near_s, _ds = nearest_topo_node(topo_nodes, s_xy[0], s_xy[1])
        near_g, _dg = nearest_topo_node(topo_nodes, g_xy[0], g_xy[1])
        temp_edges = list(topo_edges)

        def add_temp_connections(temp_id, nearest_id):
            candidates = sorted(list(collect_second_layer_candidates(topo_edges, nearest_id)))
            for nid in candidates:
                w = euclid_xy(temp_nodes, temp_id, nid)
                temp_edges.append((temp_id, nid, w, "temp", 1))

        add_temp_connections(start_id, near_s)
        add_temp_connections(goal_id, near_g)

        direct_distance = euclid_xy(temp_nodes, start_id, goal_id)
        if ENABLE_DIRECT_CONNECTION and direct_distance <= DIRECT_CONNECTION_THRESHOLD:
            temp_edges.append((start_id, goal_id, direct_distance * 0.9, "temp_direct", 1))

        temp_graph = build_graph(temp_nodes, temp_edges)
        topo_path_ids, _visited = dijkstra(temp_graph, start_id, goal_id)
        if not topo_path_ids:
            return {
                "path_xy": [],
                "path_ids": [],
                "detour_ratio": None,
            }

        path_xy = build_interpolated_path_xy(temp_nodes, topo_path_ids, INTERPOLATION_POINTS)
        straight = math.hypot(g_xy[0] - s_xy[0], g_xy[1] - s_xy[1])
        plen = path_length_xy(path_xy)
        detour_ratio = (plen / straight) if straight > 1e-6 else float("inf")
        return {
            "path_xy": path_xy,
            "path_ids": topo_path_ids,
            "detour_ratio": detour_ratio,
        }

    def on_click(event):
        if event.inaxes != ax:
            return
        if event.button != 1:
            return
        if not layer_visible["offline"]:
            return
        if not topo_nodes:
            print("⚠ 离线模式需要拓扑文件，当前不可用")
            return
        if event.xdata is None or event.ydata is None:
            return

        offline_state["clicks"].append((event.xdata, event.ydata))
        if len(offline_state["clicks"]) == 1:
            print(f"[offline] 设置起点: ({event.xdata:.3f}, {event.ydata:.3f})")
            return

        if len(offline_state["clicks"]) >= 2:
            s_xy = offline_state["clicks"][0]
            g_xy = offline_state["clicks"][1]
            result = plan_offline_path_from_clicks(s_xy, g_xy)
            offline_state["start_xy"] = s_xy
            offline_state["goal_xy"] = g_xy
            offline_state["path_xy"] = result["path_xy"] if result else []
            offline_state["path_ids"] = result["path_ids"] if result else []
            offline_state["detour_ratio"] = result["detour_ratio"] if result else None
            offline_state["clicks"] = []

            if offline_state["path_xy"]:
                print(
                    f"[offline] 路径点数={len(offline_state['path_xy'])}, "
                    f"拓扑序列={offline_state['path_ids']}, "
                    f"detour_ratio={offline_state['detour_ratio']:.3f}"
                )
            else:
                print("[offline] 规划失败：未找到可行路径")

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("scroll_event", on_scroll)
    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)

    print("\nTopic 模式说明：")
    print(f"  - 订阅全局路径: {args.path_topic}")
    print(f"  - 订阅局部路径: {args.local_path_topic}")
    print(f"  - 订阅目标: {args.clicked_topic} (优先), {args.target_topic} (兜底)")
    print(f"  - 初始显示拓扑: {'ON' if layer_visible['topology'] else 'OFF'}")
    print("  - 左侧勾选框可切换显示: Global / Local / Topology / Indices / Distance / Offline")
    print("  - Offline 打开后：在图中点击两次（起点->终点）可绘制离线路径")
    print("  - 交互: 滚轮缩放, 右键拖拽旋转, 中键拖拽平移")
    print("  - 按 Ctrl+C 退出\n")

    while not rospy.is_shutdown() and plt.fignum_exists(fig.number):
        with lock:
            global_path_xy = list(state["global_path_xy"])
            local_path_xy = list(state["local_path_xy"])
            goal_xy = state["goal_xy"]
            goal_source = state["goal_source"]

        # 初始化/保持视图中心与范围
        raw_points = []
        if topo_nodes:
            raw_points.extend((topo_nodes[n][0], topo_nodes[n][1]) for n in topo_nodes)
        raw_points.extend(global_path_xy)
        raw_points.extend(local_path_xy)
        if goal_xy is not None:
            raw_points.append(goal_xy)
        if offline_state["path_xy"]:
            raw_points.extend(offline_state["path_xy"])
        if offline_state["start_xy"] is not None:
            raw_points.append(offline_state["start_xy"])
        if offline_state["goal_xy"] is not None:
            raw_points.append(offline_state["goal_xy"])
        init_camera_if_needed(raw_points)

        ax.clear()
        ax.grid(True)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title("Topology Debugger - Topic Mode (global + local path)")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")

        if layer_visible["topology"] and topo_nodes:
            for frm, to, _w, etype, _bidir in normal_edges:
                x1, y1 = transform_point((topo_nodes[frm][0], topo_nodes[frm][1]))
                x2, y2 = transform_point((topo_nodes[to][0], topo_nodes[to][1]))
                ax.plot(
                    [x1, x2], [y1, y2],
                    linestyle="--" if etype == "auto" else "-",
                    linewidth=0.8, color="gray", alpha=0.45, zorder=1,
                )
            for frm, to, _w, _etype, _bidir, _ratio, _d in bad_edges:
                if frm in topo_nodes and to in topo_nodes:
                    x1, y1 = transform_point((topo_nodes[frm][0], topo_nodes[frm][1]))
                    x2, y2 = transform_point((topo_nodes[to][0], topo_nodes[to][1]))
                    ax.plot([x1, x2], [y1, y2], "r--", linewidth=0.9, alpha=0.5, zorder=1)

        def draw_index_labels(path_xy, prefix, color, zbase):
            if not path_xy:
                return
            step = max(1, int(args.index_step))
            adaptive_step = max(step, max(1, len(path_xy) // 80))
            for i in range(0, len(path_xy), adaptive_step):
                x, y = transform_point(path_xy[i])
                ax.text(x, y, f"{prefix}{i}", fontsize=7, color=color, zorder=zbase,
                        bbox=dict(facecolor="white", edgecolor="none", alpha=0.65, pad=0.5))

        def draw_key_points(path_xy, prefix, color, marker, zbase, extra_idx=None):
            if not path_xy:
                return []
            ids = set(build_key_indices(len(path_xy)))
            if extra_idx is not None and 0 <= extra_idx < len(path_xy):
                ids.add(extra_idx)
            ids = sorted(ids)
            for i in ids:
                x, y = transform_point(path_xy[i])
                is_extra = (extra_idx is not None and i == extra_idx)
                ax.scatter([x], [y], s=95 if is_extra else 70, marker="D" if is_extra else marker,
                           color=color, edgecolors="black", linewidths=0.6, zorder=zbase)
                tag = f"{prefix}*{i}" if is_extra else f"{prefix}{i}"
                ax.text(x, y, tag, fontsize=8.5, zorder=zbase + 0.5,
                        bbox=dict(facecolor="white", edgecolor=color, alpha=0.82, pad=0.9))
            return ids

        def draw_distance_line(p1, p2, text, color):
            p1t = transform_point(p1)
            p2t = transform_point(p2)
            ax.plot([p1t[0], p2t[0]], [p1t[1], p2t[1]], linestyle="--", color=color, linewidth=1.2, zorder=5)
            mx = 0.5 * (p1t[0] + p2t[0])
            my = 0.5 * (p1t[1] + p2t[1])
            ax.text(mx, my, text, fontsize=8.5, color=color, zorder=9,
                    bbox=dict(facecolor="white", edgecolor=color, alpha=0.78, pad=0.8))

        if layer_visible["global"] and global_path_xy:
            global_tf = transform_points(global_path_xy)
            px = [p[0] for p in global_tf]
            py = [p[1] for p in global_tf]
            ax.plot(px, py, color="limegreen", linewidth=3.0, zorder=6, label=args.path_topic)
            ax.scatter(px, py, s=9, color="limegreen", alpha=0.75, zorder=6)
            ax.scatter([px[0]], [py[0]], s=70, marker="o", color="tab:blue", zorder=7, label="global_start")
            ax.scatter([px[-1]], [py[-1]], s=90, marker="X", color="tab:red", zorder=7, label="global_end")
            near_goal_idx = nearest_index_to_point(global_path_xy, goal_xy) if goal_xy is not None else None
            draw_key_points(global_path_xy, "G", "darkgreen", "o", 10, extra_idx=near_goal_idx)
            if layer_visible["indices"]:
                draw_index_labels(global_path_xy, "G", "darkgreen", 9)
            if layer_visible["distance"] and len(global_path_xy) >= 2:
                seg_step = max(1, len(global_path_xy) // 120)
                for i in range(0, len(global_path_xy) - 1, seg_step):
                    p1 = global_path_xy[i]
                    p2 = global_path_xy[i + 1]
                    d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                    mx, my = transform_point(((p1[0] + p2[0]) * 0.5, (p1[1] + p2[1]) * 0.5))
                    ax.text(mx, my, f"{d:.2f}", fontsize=6.8, color="darkgreen", zorder=9,
                            bbox=dict(facecolor="white", edgecolor="none", alpha=0.70, pad=0.4))
        else:
            if not global_path_xy:
                ax.text(0.5, 0.5, f"waiting {args.path_topic} ...",
                        transform=ax.transAxes, ha="center", va="center")

        if layer_visible["local"] and local_path_xy:
            local_tf = transform_points(local_path_xy)
            lpx = [p[0] for p in local_tf]
            lpy = [p[1] for p in local_tf]
            ax.plot(lpx, lpy, color="tab:orange", linewidth=2.2, zorder=7, label=args.local_path_topic)
            ax.scatter(lpx, lpy, s=8, color="tab:orange", alpha=0.7, zorder=7)
            ax.scatter([lpx[0]], [lpy[0]], s=60, marker="^", color="navy", zorder=8, label="local_start")
            ax.scatter([lpx[-1]], [lpy[-1]], s=80, marker="P", color="orangered", zorder=8, label="local_end")
            draw_key_points(local_path_xy, "L", "darkorange", "^", 11, extra_idx=None)
            if layer_visible["indices"]:
                draw_index_labels(local_path_xy, "L", "saddlebrown", 9)

        if goal_xy is not None:
            gtx, gty = transform_point(goal_xy)
            ax.scatter([gtx], [gty], s=100, marker="*", color="tab:orange",
                       zorder=8, label=goal_source if goal_source else "goal")
            ax.text(gtx, gty, "GOAL", fontsize=9.5, zorder=12,
                    bbox=dict(facecolor="white", edgecolor="tab:orange", alpha=0.85, pad=0.9))
            if layer_visible["distance"] and layer_visible["global"] and global_path_xy:
                ds = math.hypot(goal_xy[0] - global_path_xy[0][0], goal_xy[1] - global_path_xy[0][1])
                draw_distance_line(global_path_xy[0], goal_xy, f"S->Goal {ds:.2f}m", "gray")

        if layer_visible["offline"] and offline_state["path_xy"]:
            off_tf = transform_points(offline_state["path_xy"])
            opx = [p[0] for p in off_tf]
            opy = [p[1] for p in off_tf]
            ax.plot(opx, opy, color="magenta", linewidth=2.8, zorder=8, label="offline_path")
            ax.scatter(opx, opy, s=7, color="magenta", alpha=0.55, zorder=8)

            os_xy = offline_state["start_xy"]
            og_xy = offline_state["goal_xy"]
            if os_xy is not None:
                osx, osy = transform_point(os_xy)
                ax.scatter([osx], [osy], s=90, marker="o", color="purple", zorder=12, label="off_start")
                ax.text(osx, osy, "OFF_S", fontsize=8.8, zorder=13,
                        bbox=dict(facecolor="white", edgecolor="purple", alpha=0.85, pad=0.8))
            if og_xy is not None:
                ogx, ogy = transform_point(og_xy)
                ax.scatter([ogx], [ogy], s=100, marker="X", color="fuchsia", zorder=12, label="off_goal")
                ax.text(ogx, ogy, "OFF_G", fontsize=8.8, zorder=13,
                        bbox=dict(facecolor="white", edgecolor="fuchsia", alpha=0.85, pad=0.8))
            if layer_visible["distance"] and os_xy is not None and og_xy is not None:
                od = math.hypot(og_xy[0] - os_xy[0], og_xy[1] - os_xy[1])
                draw_distance_line(os_xy, og_xy, f"Off S->G {od:.2f}m", "purple")

            draw_key_points(offline_state["path_xy"], "O", "purple", "s", 12, extra_idx=None)
            if layer_visible["indices"]:
                draw_index_labels(offline_state["path_xy"], "O", "purple", 11)

        # 1) 拓扑节点与编号始终最上层，避免被路径覆盖
        if layer_visible["topology"] and topo_nodes:
            topo_tf = [transform_point((topo_nodes[n][0], topo_nodes[n][1])) for n in topo_nodes]
            xs = [p[0] for p in topo_tf]
            ys = [p[1] for p in topo_tf]
            ax.scatter(xs, ys, s=22, c="white", edgecolors="black", linewidths=0.4, alpha=0.95, zorder=20)
            for nid, (x, y, _z, _label, _pidx) in topo_nodes.items():
                xt, yt = transform_point((x, y))
                ax.text(
                    xt, yt, str(nid), fontsize=8, ha="center", va="center", zorder=21,
                    bbox=dict(facecolor="white", edgecolor="none", alpha=0.75, pad=0.8)
                )

        # 固定并复用当前视窗，支持滚轮缩放与拖拽交互
        if camera["xlim"] is not None and camera["ylim"] is not None:
            ax.set_xlim(camera["xlim"])
            ax.set_ylim(camera["ylim"])

        # 2) 显示两点距离（优先 global_start -> goal，其次 global_start -> global_end）
        info_lines = []
        if global_path_xy:
            g_start = global_path_xy[0]
            g_end = global_path_xy[-1]
            straight_se = math.hypot(g_end[0] - g_start[0], g_end[1] - g_start[1])
            g_len = path_length_xy(global_path_xy)
            info_lines.append(f"global_len: {g_len:.2f} m")
            if layer_visible["distance"]:
                info_lines.append(f"global_start->end: {straight_se:.2f} m")
            if layer_visible["distance"] and goal_xy is not None:
                sg = math.hypot(goal_xy[0] - g_start[0], goal_xy[1] - g_start[1])
                info_lines.append(f"global_start->goal: {sg:.2f} m")
        if local_path_xy:
            l_len = path_length_xy(local_path_xy)
            info_lines.append(f"local_len: {l_len:.2f} m")
            if layer_visible["distance"] and goal_xy is not None:
                ls = local_path_xy[0]
                lg = math.hypot(goal_xy[0] - ls[0], goal_xy[1] - ls[1])
                info_lines.append(f"local_start->goal: {lg:.2f} m")
        if layer_visible["offline"] and offline_state["path_xy"]:
            o_len = path_length_xy(offline_state["path_xy"])
            info_lines.append(f"offline_len: {o_len:.2f} m")
            if offline_state["detour_ratio"] is not None:
                info_lines.append(f"offline_detour_ratio: {offline_state['detour_ratio']:.3f}")

        # 右侧指标面板（不覆盖主图）
        metric_ax.clear()
        metric_ax.set_title("Metrics", fontsize=10)
        metric_ax.axis("off")
        if info_lines:
            metric_ax.text(
                0.02, 0.98, "\n".join(info_lines),
                transform=metric_ax.transAxes, ha="left", va="top", fontsize=9.5,
                bbox=dict(facecolor="white", edgecolor="gray", alpha=0.90, pad=4.0),
            )
        else:
            metric_ax.text(0.02, 0.98, "waiting path data ...",
                           transform=metric_ax.transAxes, ha="left", va="top", fontsize=9.5)

        # 右侧说明面板（替代主图顶部提示框）
        hint_ax.clear()
        hint_ax.set_title("Tips", fontsize=10)
        hint_ax.axis("off")
        def wrap_line(s, width=26, indent="  "):
            return textwrap.fill(str(s), width=width, subsequent_indent=indent, break_long_words=True)
        hint_lines = [
            wrap_line(f"G: {args.path_topic}"),
            wrap_line(f"L: {args.local_path_topic}"),
            "",
            "Style:",
            " G-path: green",
            " L-path: orange",
            " Off-path: magenta",
            " GOAL: orange star",
            "",
            "Toggle:",
            " Global / Local / Topology",
            " Indices / Distance / Offline",
            "",
            "View:",
            " wheel: zoom",
            " RMB drag: rotate",
            " MMB drag: pan",
            "",
            "Offline mode:",
            " click 2 points in map",
        ]
        hint_ax.text(
            0.02, 0.98, "\n".join(hint_lines),
            transform=hint_ax.transAxes, ha="left", va="top", fontsize=8.6,
            bbox=dict(facecolor="white", edgecolor="gray", alpha=0.90, pad=4.0),
        )
        fig.canvas.draw_idle()
        plt.pause(0.001)
        rate.sleep()


def main():
    parser = argparse.ArgumentParser(description="Topology debugger + topic plotter")
    parser.add_argument("--path-topic", default=DEFAULT_PATH_TOPIC)
    parser.add_argument("--local-path-topic", default=DEFAULT_LOCAL_PATH_TOPIC)
    parser.add_argument("--clicked-topic", default=DEFAULT_CLICKED_TOPIC)
    parser.add_argument("--target-topic", default=DEFAULT_TARGET_TOPIC)
    parser.add_argument("--update-hz", type=float, default=DEFAULT_UPDATE_HZ)
    parser.add_argument("--index-step", type=int, default=DEFAULT_INDEX_STEP,
                        help="路径点序号标注步长（1=每个点都标）")

    args, _unknown = parser.parse_known_args()
    topic_mode_main(args)


if __name__ == "__main__":
    main()
