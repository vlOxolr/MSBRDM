#!/usr/bin/env python3
import os
import glob
import argparse
from typing import Optional, List, Tuple, Dict, Set

import numpy as np
import matplotlib.pyplot as plt

from skimage.io import imread, imsave
from skimage.color import rgb2gray
from skimage.filters import threshold_otsu
from skimage.morphology import (
    white_tophat,
    opening,
    closing,
    rectangle,
    disk,
    medial_axis,
)


Point = Tuple[int, int]  # (row, col)
EdgeId = int


def resolve_img_path(script_dir: str, filename: Optional[str]) -> str:
    """Resolve an image path under scripts/img."""
    img_dir = os.path.join(script_dir, "img")
    if not os.path.isdir(img_dir):
        raise FileNotFoundError("Image folder not found: {}".format(img_dir))

    if filename:
        cand = os.path.join(img_dir, filename)
        if not os.path.isfile(cand):
            raise FileNotFoundError("Image not found: {}".format(cand))
        return cand

    exts = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff")
    files = []
    for e in exts:
        files.extend(glob.glob(os.path.join(img_dir, e)))
    files = sorted(files)

    if not files:
        raise FileNotFoundError("No images found in: {}".format(img_dir))
    return files[0]


def load_gray(image_path: str) -> np.ndarray:
    """Load image and convert to grayscale in [0, 1]."""
    img = imread(image_path)
    if img.ndim == 3:
        gray = rgb2gray(img)  # [0, 1]
    else:
        gray = img.astype(np.float32)
        if gray.max() > 1.0:
            gray /= 255.0
    return gray


def neighbors8(p: Point, shape: Tuple[int, int]) -> List[Point]:
    """8-neighborhood coordinates inside bounds."""
    r, c = p
    H, W = shape
    out = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W:
                out.append((rr, cc))
    return out


def degree_map(skel: np.ndarray) -> np.ndarray:
    """Degree (# of 8-neighbors that are True) for skeleton pixels."""
    deg = np.zeros_like(skel, dtype=np.uint8)
    pts = np.argwhere(skel)
    for r, c in pts:
        p = (int(r), int(c))
        cnt = 0
        for q in neighbors8(p, skel.shape):
            if skel[q]:
                cnt += 1
        deg[p] = cnt
    return deg


def extract_nodes(skel: np.ndarray, deg: np.ndarray) -> Tuple[Set[Point], Set[Point], Set[Point]]:
    """
    Extract node sets:
    - endpoints: degree == 1
    - junctions: degree >= 3
    - isolated: degree == 0 (single pixel components)
    """
    endpoints = set(map(tuple, np.argwhere(skel & (deg == 1)).astype(int)))
    junctions = set(map(tuple, np.argwhere(skel & (deg >= 3)).astype(int)))
    isolated = set(map(tuple, np.argwhere(skel & (deg == 0)).astype(int)))
    return endpoints, junctions, isolated


def trace_segment_from_node(
    skel: np.ndarray,
    deg: np.ndarray,
    start_node: Point,
    next_pixel: Point,
    node_set: Set[Point],
    visited_dir: Set[Tuple[Point, Point]],
) -> List[Point]:
    """
    Trace a segment starting at start_node and stepping into next_pixel.
    The segment ends when reaching another node (endpoint or junction) or dead-end.
    visited_dir stores directed traversals to avoid duplicating segments.
    """
    seg = [start_node]
    prev = start_node
    cur = next_pixel

    visited_dir.add((start_node, next_pixel))

    while True:
        seg.append(cur)

        if cur in node_set and cur != start_node:
            break

        nbs = [q for q in neighbors8(cur, skel.shape) if skel[q]]
        if len(nbs) == 0:
            break

        # remove coming-from pixel
        nbs2 = [q for q in nbs if q != prev]
        if len(nbs2) == 0:
            break

        # if multiple choices (near junction/noise), pick the one not immediately visited
        nxt = None
        for cand in nbs2:
            if (cur, cand) not in visited_dir:
                nxt = cand
                break
        if nxt is None:
            nxt = nbs2[0]

        visited_dir.add((cur, nxt))
        prev, cur = cur, nxt

    return seg


def extract_segments(skel: np.ndarray, deg: np.ndarray) -> Tuple[List[List[Point]], Set[Point], Set[Point], Set[Point]]:
    """
    Split skeleton into segments between nodes (endpoints/junctions).
    Returns:
    - segments: list of polyline pixel chains, each begins/ends at a node (or dead end)
    - endpoints, junctions, isolated points
    """
    endpoints, junctions, isolated = extract_nodes(skel, deg)
    node_set = set(endpoints) | set(junctions)

    visited_dir: Set[Tuple[Point, Point]] = set()
    segments: List[List[Point]] = []

    # Start segments from each node, for each neighbor direction
    for node in sorted(node_set):
        nbs = [q for q in neighbors8(node, skel.shape) if skel[q]]
        for nb in nbs:
            if (node, nb) in visited_dir:
                continue
            seg = trace_segment_from_node(skel, deg, node, nb, node_set, visited_dir)
            if len(seg) >= 2:
                segments.append(seg)

    # Handle cycles with no endpoints/junctions (all degree==2)
    # We find remaining unvisited skeleton pixels and trace a loop as a segment.
    unvisited = skel.copy()
    for (a, b) in visited_dir:
        unvisited[a] = False
        unvisited[b] = False

    # Robust loop extraction: any pixel that is True but not in node_set and still exists may form a loop.
    loop_candidates = np.argwhere(skel & (~np.isin(np.arange(skel.size).reshape(skel.shape), [])))
    # We will do a simpler pass: find pixels with deg==2 that are not in any segment.
    covered = np.zeros_like(skel, dtype=bool)
    for seg in segments:
        for p in seg:
            covered[p] = True
    remain = np.argwhere(skel & (~covered) & (deg == 2))
    remain = [tuple(map(int, p)) for p in remain]

    visited_loop = set()
    for start in remain:
        if start in visited_loop:
            continue
        # trace loop
        loop = [start]
        visited_loop.add(start)
        prev = None
        cur = start
        for _ in range(200000):
            nbs = [q for q in neighbors8(cur, skel.shape) if skel[q]]
            if prev is not None:
                nbs = [q for q in nbs if q != prev]
            if len(nbs) == 0:
                break
            nxt = nbs[0]
            prev, cur = cur, nxt
            if cur == start:
                loop.append(cur)
                break
            if cur in visited_loop:
                break
            loop.append(cur)
            visited_loop.add(cur)
        if len(loop) >= 3:
            segments.append(loop)

    return segments, endpoints, junctions, isolated


def segment_endpoints(seg: List[Point]) -> Tuple[Point, Point]:
    return seg[0], seg[-1]


def build_graph_from_segments(segments: List[List[Point]]) -> Tuple[Dict[Point, List[EdgeId]], Dict[EdgeId, Tuple[Point, Point]]]:
    """
    Build a node-edge adjacency graph from segments.
    Nodes are segment endpoints (pixel coordinates).
    """
    node_to_edges: Dict[Point, List[EdgeId]] = {}
    edge_to_nodes: Dict[EdgeId, Tuple[Point, Point]] = {}

    for eid, seg in enumerate(segments):
        a, b = segment_endpoints(seg)
        edge_to_nodes[eid] = (a, b)
        node_to_edges.setdefault(a, []).append(eid)
        node_to_edges.setdefault(b, []).append(eid)

    return node_to_edges, edge_to_nodes


def prune_short_spurs(
    segments: List[List[Point]],
    endpoints: Set[Point],
    junctions: Set[Point],
    spur_len_thresh: int = 3,
) -> List[List[Point]]:
    """
    Remove short spur segments attached to a junction, but keep isolated short segments (likely dots).
    Rule:
    - If len(segment) < spur_len_thresh AND one end is a junction AND the other end is an endpoint -> remove it.
    - If both ends are endpoints (isolated short stroke) -> keep it.
    - If both ends are junctions (rare) -> keep it.
    """
    kept: List[List[Point]] = []
    for seg in segments:
        L = len(seg)
        a, b = segment_endpoints(seg)
        is_spur = (L < spur_len_thresh) and (
            (a in junctions and b in endpoints) or (b in junctions and a in endpoints)
        )
        if not is_spur:
            kept.append(seg)
    return kept


def tangent_at_node(seg: List[Point], node: Point, k: int = 5) -> np.ndarray:
    """
    Estimate direction vector of a segment at a given endpoint node.
    We take k pixels away from the node to compute a local tangent.
    """
    if len(seg) < 2:
        return np.array([0.0, 0.0], dtype=np.float32)

    if seg[0] == node:
        p0 = np.array(seg[0], dtype=np.float32)
        p1 = np.array(seg[min(k, len(seg) - 1)], dtype=np.float32)
    elif seg[-1] == node:
        p0 = np.array(seg[-1], dtype=np.float32)
        p1 = np.array(seg[max(0, len(seg) - 1 - k)], dtype=np.float32)
    else:
        # node not an endpoint of the segment
        p0 = np.array(node, dtype=np.float32)
        p1 = p0.copy()

    v = p1 - p0  # (dr, dc)
    n = float(np.linalg.norm(v))
    if n < 1e-6:
        return np.array([0.0, 0.0], dtype=np.float32)
    return v / n


def turn_angle(v_in: np.ndarray, v_out: np.ndarray) -> float:
    """
    Turning angle definition:
    - v_in points OUTWARD from junction along the incoming segment.
    - v_out points OUTWARD from junction along the candidate outgoing segment.
    We want small bending => v_out ~ -v_in.
    So we compute angle between (-v_in) and v_out.
    """
    a = -v_in
    b = v_out
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.arccos(dot))  # 0 is best (straight), pi is worst


def build_junction_pairing(
    segments: List[List[Point]],
    node_to_edges: Dict[Point, List[EdgeId]],
    edge_to_nodes: Dict[EdgeId, Tuple[Point, Point]],
    junctions: Set[Point],
    short_edge_len: int = 30,
) -> Dict[Tuple[Point, EdgeId], EdgeId]:
    """
    For each junction node with degree >= 3, pair incident edges to define "through connections".
    Preference:
    - Prefer pairing among short segments (len <= short_edge_len)
    - Within candidates, choose pair with smallest turning angle.

    Returns mapping:
    - (junction, incoming_edge) -> outgoing_edge
    symmetric for the paired direction.
    """
    pair_map: Dict[Tuple[Point, EdgeId], EdgeId] = {}

    # Precompute segment length and tangents at node
    seg_len = [len(s) for s in segments]

    for j in junctions:
        inc = node_to_edges.get(j, [])
        if len(inc) < 3:
            continue

        # Build direction vectors for each incident edge at this junction
        v_out: Dict[EdgeId, np.ndarray] = {}
        for eid in inc:
            v_out[eid] = tangent_at_node(segments[eid], j, k=5)

        # Greedy pairing
        remaining = set(inc)

        def best_pair(candidates: List[EdgeId]) -> Optional[Tuple[EdgeId, EdgeId, float]]:
            best = None
            for e1 in candidates:
                for e2 in candidates:
                    if e2 <= e1:
                        continue
                    ang = turn_angle(v_out[e1], v_out[e2])
                    if best is None or ang < best[2]:
                        best = (e1, e2, ang)
            return best

        while len(remaining) >= 2:
            # Prefer short edges if possible
            short_edges = [e for e in remaining if seg_len[e] <= short_edge_len]
            cand = short_edges if len(short_edges) >= 2 else list(remaining)

            bp = best_pair(cand)
            if bp is None:
                break
            e1, e2, _ = bp

            # Register pairing both ways at this junction
            pair_map[(j, e1)] = e2
            pair_map[(j, e2)] = e1

            remaining.remove(e1)
            remaining.remove(e2)

    return pair_map


def other_node(edge_nodes: Tuple[Point, Point], node: Point) -> Point:
    a, b = edge_nodes
    return b if node == a else a


def build_continuous_polylines(
    segments: List[List[Point]],
    node_to_edges: Dict[Point, List[EdgeId]],
    edge_to_nodes: Dict[EdgeId, Tuple[Point, Point]],
    pair_map: Dict[Tuple[Point, EdgeId], EdgeId],
) -> List[List[Point]]:
    """
    Merge segments into continuous polylines with no branching.
    We walk edge-by-edge, continuing through nodes using:
    - If node has degree==2: continue along the other edge.
    - If node has degree>=3 (junction): continue only if a pairing exists for (node, incoming_edge).
      Otherwise stop.
    """
    used: Set[EdgeId] = set()
    polylines: List[List[Point]] = []

    # Helper: get ordered segment points so that it starts at 'from_node'
    def oriented_segment(eid: EdgeId, from_node: Point) -> List[Point]:
        seg = segments[eid]
        a, b = edge_to_nodes[eid]
        if seg[0] == from_node:
            return seg
        if seg[-1] == from_node:
            return seg[::-1]
        # fallback: orient by closest endpoint
        if from_node == a:
            return seg
        return seg[::-1]

    for eid0 in range(len(segments)):
        if eid0 in used:
            continue

        a0, b0 = edge_to_nodes[eid0]

        # Build forward from a0 -> ...
        path_fwd: List[Point] = []
        cur_node = a0
        incoming_eid = eid0
        seg_pts = oriented_segment(incoming_eid, cur_node)
        path_fwd.extend(seg_pts)
        used.add(incoming_eid)
        cur_node = seg_pts[-1]  # next node

        while True:
            inc_edges = node_to_edges.get(cur_node, [])
            deg_node = len(inc_edges)

            if deg_node == 0:
                break

            # Determine next edge
            nxt_eid = None

            if deg_node == 2:
                # Continue through a degree-2 node (no branching)
                e_candidates = [e for e in inc_edges if e != incoming_eid and e not in used]
                if len(e_candidates) == 0:
                    break
                nxt_eid = e_candidates[0]

            elif deg_node >= 3:
                # Junction: follow pairing if exists
                key = (cur_node, incoming_eid)
                if key in pair_map:
                    cand = pair_map[key]
                    if cand not in used:
                        nxt_eid = cand
                if nxt_eid is None:
                    break

            else:
                # deg_node == 1 -> endpoint
                break

            # Append next segment (avoid duplicating the node point)
            seg2 = oriented_segment(nxt_eid, cur_node)
            path_fwd.extend(seg2[1:])
            used.add(nxt_eid)

            incoming_eid = nxt_eid
            cur_node = seg2[-1]

        polylines.append(path_fwd)

    return polylines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--file",
        type=str,
        default=None,
        help="Image filename under scripts/img (e.g., test.png). If omitted, the first image is used.",
    )
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    image_path = resolve_img_path(script_dir, args.file)
    print("[INFO] Loading: {}".format(image_path))

    gray = load_gray(image_path)

    # --- Step 1: Binarization ---
    t1 = threshold_otsu(gray)
    binary1 = gray < t1

    # --- Step 2: White top-hat on inverted grayscale ---
    inv = 1.0 - gray
    TH_H = 50#15
    TH_W = 50#10
    selem_tophat = rectangle(TH_H, TH_W)
    tophat = white_tophat(inv, selem_tophat)

    # --- Step 3: Binarization after top-hat ---
    TH_T = 0.2
    binary2 = tophat > TH_T

    # --- Step 4: Opening ---
    OPEN_R = 1
    opened = opening(binary2, disk(OPEN_R))

    # --- Step 5: Closing (+ extra opening) ---
    CLOSE_R = 2
    cleaned = closing(opened, disk(CLOSE_R))
    cleaned = opening(cleaned, disk(OPEN_R))

    # --- Step 6: Medial axis skeleton ---
    medial_skel, dist = medial_axis(cleaned, return_distance=True)

    # --- Graph extraction ---
    deg = degree_map(medial_skel)
    segments, endpoints, junctions, isolated = extract_segments(medial_skel, deg)

    # --- Spur pruning (remove attached short branches < 3 pixels) ---
    segments_pruned = prune_short_spurs(
        segments=segments,
        endpoints=endpoints,
        junctions=junctions,
        spur_len_thresh=3,
    )

    # Rebuild graph after pruning
    node_to_edges, edge_to_nodes = build_graph_from_segments(segments_pruned)

    # Recompute "junction set" for graph-level (some junctions may become degree-2 after pruning)
    graph_junctions = {n for n, es in node_to_edges.items() if len(es) >= 3}

    # --- Junction pairing rule (angle-based, prefer short edges) ---
    pair_map = build_junction_pairing(
        segments=segments_pruned,
        node_to_edges=node_to_edges,
        edge_to_nodes=edge_to_nodes,
        junctions=graph_junctions,
        short_edge_len=30,  # you can tune this: what counts as "short segment"
    )

    # --- Merge into continuous non-branching polylines ---
    polylines = build_continuous_polylines(
        segments=segments_pruned,
        node_to_edges=node_to_edges,
        edge_to_nodes=edge_to_nodes,
        pair_map=pair_map,
    )

    print("[INFO] Raw segments: {}".format(len(segments)))
    print("[INFO] Pruned segments: {}".format(len(segments_pruned)))
    print("[INFO] Continuous polylines: {}".format(len(polylines)))
    print("[INFO] Endpoints: {}, junctions(before graph): {}, isolated points: {}".format(
        len(endpoints), len(junctions), len(isolated)
    ))
    print("[INFO] Graph junctions (deg>=3): {}".format(len(graph_junctions)))

    # --- Save key outputs ---
    in_base = os.path.basename(image_path)
    name, ext = os.path.splitext(in_base)

    out_dir = os.path.dirname(image_path)

    cleaned_u8 = (cleaned.astype(np.uint8) * 255)
    imsave(os.path.join(out_dir, f"{name}_cleaned{ext}"), cleaned_u8)

    medial_u8 = (medial_skel.astype(np.uint8) * 255)
    imsave(os.path.join(out_dir, f"{name}_skeleton_medial{ext}"), medial_u8)

    if dist.max() > 0:
        dist_u8 = (dist / dist.max() * 255.0).astype(np.uint8)
        imsave(os.path.join(out_dir, f"{name}_dist{ext}"), dist_u8)

    # --- Visualization (2 rows x 3 cols) ---
    # Keep the original layout, but use the last two panels for graph/paths debugging.
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), sharex=True, sharey=True)
    ax = axes.ravel()

    ax[0].imshow(gray, cmap=plt.cm.gray)
    ax[0].axis("off")
    ax[0].set_title("gray", fontsize=12)

    ax[1].imshow(binary2, cmap=plt.cm.gray)
    ax[1].axis("off")
    ax[1].set_title("binary2 (after top-hat)", fontsize=12)

    ax[2].imshow(cleaned, cmap=plt.cm.gray)
    ax[2].axis("off")
    ax[2].set_title("cleaned", fontsize=12)

    ax[3].imshow(medial_skel, cmap=plt.cm.gray)
    ax[3].axis("off")
    ax[3].set_title("medial axis skeleton", fontsize=12)

    # Panel 4: segments overlay
    ax[4].imshow(medial_skel, cmap=plt.cm.gray)
    ax[4].axis("off")
    ax[4].set_title("segments (pruned) + nodes", fontsize=12)

    # Plot each segment with a thin line
    for seg in segments_pruned:
        rr = np.array([p[0] for p in seg], dtype=np.float32)
        cc = np.array([p[1] for p in seg], dtype=np.float32)
        ax[4].plot(cc, rr, linewidth=1.0)

    # Plot endpoints and junctions
    if len(endpoints) > 0:
        ep = np.array(list(endpoints))
        ax[4].scatter(ep[:, 1], ep[:, 0], s=10, marker="o")
    if len(graph_junctions) > 0:
        jn = np.array(list(graph_junctions))
        ax[4].scatter(jn[:, 1], jn[:, 0], s=20, marker="x")

    # Panel 5: merged polylines
    ax[5].imshow(medial_skel, cmap=plt.cm.gray)
    ax[5].axis("off")
    ax[5].set_title("merged polylines (no branching)", fontsize=12)

    for pl in polylines:
        rr = np.array([p[0] for p in pl], dtype=np.float32)
        cc = np.array([p[1] for p in pl], dtype=np.float32)
        ax[5].plot(cc, rr, linewidth=1.2)
        # mark start/end
        ax[5].scatter([cc[0]], [rr[0]], s=12)
        ax[5].scatter([cc[-1]], [rr[-1]], s=12)

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
