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

from tqdm import tqdm

# Try to use scipy for fast convolution / KDTree; fallback to numpy brute force if not available.
try:
    from scipy.ndimage import convolve as ndi_convolve
except Exception:
    ndi_convolve = None

try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None


Point = Tuple[int, int]  # (row, col)


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


# -------------------- New method (from medial_skel) --------------------

def neighbor_count_8(skel: np.ndarray) -> np.ndarray:
    """
    Count 8-neighbor skeleton pixels for each pixel.
    Implemented with convolution + padding (output shape unchanged).
    """
    sk = skel.astype(np.uint8)

    kernel = np.array(
        [[1, 1, 1],
         [1, 0, 1],
         [1, 1, 1]], dtype=np.uint8
    )

    if ndi_convolve is not None:
        # mode='constant' with cval=0 ensures padding without changing size
        cnt = ndi_convolve(sk, kernel, mode="constant", cval=0)
        return cnt.astype(np.uint8)

    # Fallback: manual convolution (still padded, but slower)
    H, W = sk.shape
    pad = np.pad(sk, ((1, 1), (1, 1)), mode="constant", constant_values=0)
    cnt = np.zeros((H, W), dtype=np.uint8)
    for r in range(H):
        for c in range(W):
            win = pad[r:r+3, c:c+3]
            cnt[r, c] = int(np.sum(win * kernel))
    return cnt


def classify_pixels(skel: np.ndarray, cnt: np.ndarray) -> Dict[str, Set[Point]]:
    """
    Classification per your definition (only for skeleton pixels):
    - cnt == 0 : isolated node
    - cnt == 1 : endpoint
    - cnt == 2 : track point
    - cnt >= 3 : junction point
    """
    sk = skel.astype(bool)
    iso = set(map(tuple, np.argwhere(sk & (cnt == 0)).astype(int)))
    endp = set(map(tuple, np.argwhere(sk & (cnt == 1)).astype(int)))
    track = set(map(tuple, np.argwhere(sk & (cnt == 2)).astype(int)))
    junc = set(map(tuple, np.argwhere(sk & (cnt >= 3)).astype(int)))
    return {"isolated": iso, "endpoints": endp, "track": track, "junctions": junc}


def cluster_junctions(junctions: List[Point], radius: float = 10.0) -> Tuple[List[List[Point]], Dict[Point, Point]]:
    """
    Group junction pixels into junction-groups if within Euclidean radius.
    Return:
    - clusters: list of clusters (each is list of junction pixels)
    - map_old_to_rep: mapping old junction pixel -> representative (median) point
    """
    if len(junctions) == 0:
        return [], {}

    pts = np.array(junctions, dtype=np.float32)

    # Find neighbors efficiently if KDTree is available; else brute force.
    if cKDTree is not None:
        tree = cKDTree(pts)
        nbrs = tree.query_ball_tree(tree, r=radius)
        # nbrs[i] gives indices within radius of i (including itself)
    else:
        # brute force adjacency list
        nbrs = []
        for i in range(len(pts)):
            d = np.linalg.norm(pts - pts[i], axis=1)
            nbrs.append(list(np.where(d <= radius)[0]))

    visited = np.zeros(len(pts), dtype=bool)
    clusters: List[List[Point]] = []
    map_old_to_rep: Dict[Point, Point] = {}

    for i in tqdm(range(len(pts)), desc="Clustering junctions", leave=False):
        if visited[i]:
            continue
        # BFS/DFS to form one cluster
        stack = [i]
        visited[i] = True
        idxs = []
        while stack:
            cur = stack.pop()
            idxs.append(cur)
            for nb in nbrs[cur]:
                if not visited[nb]:
                    visited[nb] = True
                    stack.append(nb)

        cluster = [junctions[k] for k in idxs]
        clusters.append(cluster)

        # representative = median coordinate (rounded to int)
        arr = np.array(cluster, dtype=np.int32)
        rep_r = int(np.median(arr[:, 0]))
        rep_c = int(np.median(arr[:, 1]))
        rep = (rep_r, rep_c)

        for p in cluster:
            map_old_to_rep[p] = rep

    return clusters, map_old_to_rep


class Line:
    def __init__(self, lid: int, points: List[Point], start_node: Point, end_node: Point):
        self.id = lid
        self.points = points
        self.start = start_node
        self.end = end_node

    def length(self) -> int:
        return len(self.points)


def trace_lines(
    skel: np.ndarray,
    cnt: np.ndarray,
    endpoints: Set[Point],
    junctions: Set[Point],
    jmap: Dict[Point, Point],
) -> List[Line]:
    """
    Build lines:
    start/end are endpoint or junction (mapped to representative if junction-grouped).
    Middle are track points (cnt==2), and each track point can belong to ONLY ONE line.

    Strategy:
    - Start from all endpoints + all junction pixels (but treat junction nodes using representative).
    - For each node pixel, follow each neighbor direction into skeleton:
      accumulate until reaching endpoint or junction; stop.
    - Track points are marked used so they won't be reused in another line.
    """
    H, W = skel.shape
    sk = skel.astype(bool)

    # Node pixels in raw skeleton space (endpoints + junction pixels)
    node_pixels = set(endpoints) | set(junctions)

    used_track: Set[Point] = set()
    used_directed: Set[Tuple[Point, Point]] = set()  # avoid duplicating immediate traversals

    # Helper: is junction pixel?
    def is_junction_pix(p: Point) -> bool:
        return p in junctions

    # Helper: map node pixel to canonical node coordinate
    def canon_node(p: Point) -> Point:
        if p in jmap:
            return jmap[p]
        return p

    lines: List[Line] = []
    lid = 0

    # We iterate nodes with tqdm for visibility
    for node in tqdm(sorted(node_pixels), desc="Tracing lines from nodes"):
        # if node is a track or isolated, skip (should not happen)
        if not sk[node]:
            continue

        nbs = [q for q in neighbors8(node, (H, W)) if sk[q]]
        for nb in nbs:
            if (node, nb) in used_directed:
                continue

            # We will trace one candidate line
            pts = [node]
            prev = node
            cur = nb
            used_directed.add((node, nb))

            # If first step is a track point already used, we skip to avoid duplicating lines
            if (cnt[cur] == 2) and (cur in used_track):
                continue

            while True:
                pts.append(cur)

                # stop if reach a different node pixel (endpoint or junction)
                if cur in node_pixels and cur != node:
                    break

                # stop if dead
                nbrs = [q for q in neighbors8(cur, (H, W)) if sk[q]]
                if len(nbrs) == 0:
                    break

                # remove coming-from pixel
                nbrs2 = [q for q in nbrs if q != prev]
                if len(nbrs2) == 0:
                    break

                # choose next not yet traversed if possible
                nxt = None
                for cand in nbrs2:
                    if (cur, cand) not in used_directed:
                        nxt = cand
                        break
                if nxt is None:
                    nxt = nbrs2[0]

                # mark directed step
                used_directed.add((cur, nxt))

                prev, cur = cur, nxt

                # if cur is track point, and already used, then we should stop early to avoid overlap
                if (cnt[cur] == 2) and (cur in used_track):
                    # remove the overlapping point to keep consistency
                    pts.pop()
                    break

            # Determine canonical endpoints
            raw_start = pts[0]
            raw_end = pts[-1]
            start_node = canon_node(raw_start) if (raw_start in node_pixels) else raw_start
            end_node = canon_node(raw_end) if (raw_end in node_pixels) else raw_end

            # Only accept if it's a meaningful line:
            # - must include at least 2 points
            # - must end at endpoint/junction (or dead end pixel) but your definition expects endpoint/junction;
            #   here we keep dead-end too, but it usually indicates noise.
            if len(pts) < 2:
                continue

            # Mark track points used (excluding endpoints/junction pixels)
            for p in pts[1:-1]:
                if cnt[p] == 2:
                    used_track.add(p)

            lines.append(Line(lid, pts, start_node, end_node))
            lid += 1

    # Optional: deduplicate trivial self-loops (start==end and too short)
    filtered: List[Line] = []
    for ln in lines:
        if ln.start == ln.end and ln.length() <= 3:
            continue
        filtered.append(ln)

    # IMPORTANT: re-index line ids to match list indices (avoid IndexError later)
    for new_id, ln in enumerate(filtered):
        ln.id = new_id

    return filtered


def extract_cycle_lines(
    skel: np.ndarray,
    cnt: np.ndarray,
    used_track: Set[Point],
    existing_ids_start: int,
) -> Tuple[List[Line], Set[Point]]:
    """
    Extract closed cycles (ring-like skeleton components) where all pixels are track points (cnt==2),
    hence no endpoints/junctions, and thus ignored by trace_lines().

    Inputs:
      - skel: medial skeleton (bool)
      - cnt: 8-neighbor count map
      - used_track: track pixels already covered by existing lines
      - existing_ids_start: starting id for new lines

    Returns:
      - new_lines: List[Line] for cycles, each with start=end at the cycle seed
      - used_track_updated: updated used_track including cycle pixels
    """
    H, W = skel.shape
    sk = skel.astype(bool)

    # remaining track pixels not covered by any traced line
    remain = set(map(tuple, np.argwhere(sk & (cnt == 2)).astype(int))) - set(used_track)
    visited: Set[Point] = set()

    new_lines: List[Line] = []
    lid = existing_ids_start

    for start in tqdm(list(remain), desc="Extracting cycles", leave=False):
        if start in visited:
            continue
        if start not in remain:
            continue

        # a cycle pixel should have exactly 2 skeleton neighbors (under ideal conditions)
        nbs0 = [q for q in neighbors8(start, (H, W)) if sk[q]]
        if len(nbs0) < 2:
            # not a clean cycle (or broken), skip
            visited.add(start)
            continue

        # pick one neighbor to begin walking
        prev = start
        cur = nbs0[0]
        cycle_pts = [start]

        visited_local: Set[Point] = set([start])
        closed = False

        # hard safety bound: at most number of skeleton pixels steps
        max_steps = int(np.count_nonzero(skel)) + 10

        for _ in range(max_steps):
            cycle_pts.append(cur)

            if cur == start:
                closed = True
                break

            if cur in visited_local:
                # looped somewhere else without returning to start
                break

            visited_local.add(cur)

            # choose next neighbor != prev
            nbs = [q for q in neighbors8(cur, (H, W)) if sk[q]]
            if len(nbs) == 0:
                break
            nbs2 = [q for q in nbs if q != prev]
            if len(nbs2) == 0:
                break

            # for clean degree-2 cycle, nbs2 should be 1
            nxt = nbs2[0]
            prev, cur = cur, nxt

        # validate: must be closed and reasonably long
        if closed and len(cycle_pts) >= 4:
            # mark all points (except the last repeated start) as used
            for p in cycle_pts[:-1]:
                visited.add(p)
                used_track.add(p)

            # define a cycle line: start=end at start pixel
            new_lines.append(Line(lid, cycle_pts, start, start))
            lid += 1
        else:
            # even if not closed, mark visited_local to avoid repeated attempts
            visited |= visited_local

    return new_lines, used_track


def filter_short_junction_lines(
    lines: List[Line],
    junction_reps: Set[Point],
    min_len: int = 20,
) -> List[Line]:
    """
    Remove short lines whose start OR end is a junction representative.
    Rule:
      - if line.length() < min_len and (start in junction_reps or end in junction_reps) -> drop
    After filtering, re-index line.id to keep consistent with list indices.
    """
    kept: List[Line] = []
    for ln in lines:
        is_short = ln.length() < min_len
        touches_junction = (ln.start in junction_reps) or (ln.end in junction_reps)
        if is_short and touches_junction:
            continue
        kept.append(ln)

    # re-index ids
    for new_id, ln in enumerate(kept):
        ln.id = new_id

    return kept


def direction_at_node(line: Line, node: Point, k: int = 5) -> np.ndarray:
    """
    Compute outward direction vector at a line end (node).
    Use k pixels along the polyline.
    Returns a normalized vector in (dr, dc).
    """
    pts = line.points
    if len(pts) < 2:
        return np.array([0.0, 0.0], dtype=np.float32)

    # Need to detect whether 'node' corresponds to start or end of the line in canonical space.
    # Since junction pixels were mapped to representative, the point list may contain raw junction pixels,
    # but the line.start/line.end are canonical. We'll treat:
    # - if node == line.start -> use early segment direction
    # - if node == line.end   -> use late segment direction
    if node == line.start:
        p0 = np.array(pts[0], dtype=np.float32)
        p1 = np.array(pts[min(k, len(pts) - 1)], dtype=np.float32)
        v = p1 - p0
    elif node == line.end:
        p0 = np.array(pts[-1], dtype=np.float32)
        p1 = np.array(pts[max(0, len(pts) - 1 - k)], dtype=np.float32)
        v = p1 - p0  # from node to inner -> outward along the line away from the node is (p1 - p0)
    else:
        return np.array([0.0, 0.0], dtype=np.float32)

    n = float(np.linalg.norm(v))
    if n < 1e-6:
        return np.array([0.0, 0.0], dtype=np.float32)
    return v / n


def angle_for_pair(v1: np.ndarray, v2: np.ndarray) -> float:
    """
    We want to merge two lines through a junction to make it as straight as possible.
    If v1 and v2 are both outward from the junction, straight-through means v2 ≈ -v1.
    So angle = arccos( clamp( dot(v1, -v2) ) ).
    """
    a = v1
    b = -v2
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.arccos(dot))


class DSU:
    def __init__(self, n: int):
        self.p = list(range(n))
        self.r = [0] * n

    def find(self, x: int) -> int:
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x

    def union(self, a: int, b: int):
        ra, rb = self.find(a), self.find(b)
        if ra == rb:
            return
        if self.r[ra] < self.r[rb]:
            ra, rb = rb, ra
        self.p[rb] = ra
        if self.r[ra] == self.r[rb]:
            self.r[ra] += 1


def merge_lines_into_strokes(
    lines: List[Line],
    junction_reps: Set[Point],
    k_dir: int = 5,
) -> Tuple[DSU, Dict[Point, List[int]]]:
    """
    For each junction representative node:
    - gather incident lines (line.start==j or line.end==j)
    - compute direction vectors at junction
    - greedily pair the two most "straight" lines (min angle) and union them
    """
    n = len(lines)
    dsu = DSU(n)

    node_to_lines: Dict[Point, List[int]] = {}
    for ln in lines:
        node_to_lines.setdefault(ln.start, []).append(ln.id)
        node_to_lines.setdefault(ln.end, []).append(ln.id)

    for j in tqdm(sorted(junction_reps), desc="Merging lines at junctions"):
        inc = node_to_lines.get(j, [])
        if len(inc) < 2:
            continue

        # compute outward directions for each incident line at this junction
        dirs: Dict[int, np.ndarray] = {}
        for lid in inc:
            dirs[lid] = direction_at_node(lines[lid], j, k=k_dir)

        remaining = set(inc)
        while len(remaining) >= 2:
            best = None  # (angle, l1, l2)
            rem_list = list(remaining)
            for a in range(len(rem_list)):
                for b in range(a + 1, len(rem_list)):
                    l1, l2 = rem_list[a], rem_list[b]
                    ang = angle_for_pair(dirs[l1], dirs[l2])
                    if best is None or ang < best[0]:
                        best = (ang, l1, l2)

            if best is None:
                break

            _, l1, l2 = best
            dsu.union(l1, l2)
            remaining.remove(l1)
            remaining.remove(l2)

    return dsu, node_to_lines


def build_stroke_polylines(lines: List[Line], dsu: DSU) -> List[List[Point]]:
    """
    Convert DSU groups to ordered stroke polylines.
    We build a small graph of line endpoints inside each group, then traverse.
    """
    groups: Dict[int, List[int]] = {}
    for ln in lines:
        g = dsu.find(ln.id)
        groups.setdefault(g, []).append(ln.id)

    stroke_polys: List[List[Point]] = []

    for g, lids in groups.items():
        if len(lids) == 1:
            stroke_polys.append(lines[lids[0]].points)
            continue

        # Build node -> incident line ids (within this group)
        node_adj: Dict[Point, List[int]] = {}
        for lid in lids:
            ln = lines[lid]
            node_adj.setdefault(ln.start, []).append(lid)
            node_adj.setdefault(ln.end, []).append(lid)

        # Find a start node with degree 1 (an endpoint of the stroke)
        start_node = None
        for n, inc in node_adj.items():
            if len(inc) == 1:
                start_node = n
                break
        if start_node is None:
            # cycle: pick arbitrary
            start_node = lines[lids[0]].start

        used_line: Set[int] = set()

        # walk
        cur_node = start_node
        stroke_pts: List[Point] = []
        prev_node = None

        while True:
            inc = [lid for lid in node_adj.get(cur_node, []) if lid not in used_line]
            if len(inc) == 0:
                break

            # if multiple choices (should be rare after pairing), pick one deterministically
            lid = inc[0]
            ln = lines[lid]

            # orient line to start from cur_node
            if ln.start == cur_node:
                seg = ln.points
                nxt_node = ln.end
            else:
                seg = ln.points[::-1]
                nxt_node = ln.start

            if len(stroke_pts) == 0:
                stroke_pts.extend(seg)
            else:
                stroke_pts.extend(seg[1:])  # avoid duplicate node point

            used_line.add(lid)

            prev_node, cur_node = cur_node, nxt_node

        stroke_polys.append(stroke_pts)

    return stroke_polys


# -------------------- Smoothing (already added before) --------------------

def smooth_polyline_moving_average(poly: List[Point], window: int = 9, iters: int = 2) -> List[Point]:
    """
    Smooth a stroke polyline using moving average on (row, col) coordinates.
    Endpoints are preserved.

    Tuning notes:
    - window: odd number recommended (e.g., 5, 7, 9, 11). Larger => smoother but may oversmooth corners.
    - iters: apply smoothing multiple times for stronger effect.
    """
    if len(poly) < 3:
        return list(poly)

    w = int(window)
    if w < 3:
        return list(poly)
    if w % 2 == 0:
        w += 1

    arr = np.array(poly, dtype=np.float32)
    out = arr.copy()

    kernel = np.ones(w, dtype=np.float32) / float(w)
    pad = w // 2

    for _ in range(int(iters)):
        r = out[:, 0]
        c = out[:, 1]

        r_pad = np.pad(r, (pad, pad), mode="edge")
        c_pad = np.pad(c, (pad, pad), mode="edge")

        r_s = np.convolve(r_pad, kernel, mode="valid")
        c_s = np.convolve(c_pad, kernel, mode="valid")

        out[:, 0] = r_s
        out[:, 1] = c_s

        # Preserve endpoints exactly
        out[0] = arr[0]
        out[-1] = arr[-1]

    out_int = np.rint(out).astype(np.int32)
    return [tuple(map(int, p)) for p in out_int]


# -------------------- Path planning + polynomial fitting (NEW) --------------------

def pick_key_lefttop(p: Point) -> Tuple[int, int]:
    """
    Sorting key for 'top-left to bottom-right'.
    Prioritize left (col) first, then up (row).
    """
    return (p[1], p[0])


def order_strokes_by_lefttop_endpoints(strokes: List[List[Point]]) -> List[List[Point]]:
    """
    Ordering rule:
    - Each stroke has two endpoints: first and last point.
    - Pick the globally most 'top-left' endpoint among all remaining strokes.
    - Append its stroke to ordered list.
    - Remove that stroke from future consideration.
    - Orient the stroke so that the chosen endpoint becomes the first point.
    """
    remaining = list(range(len(strokes)))
    ordered: List[List[Point]] = []

    while len(remaining) > 0:
        best = None  # (key, stroke_index_in_remaining_list, endpoint_is_start_bool)
        for ridx, sid in enumerate(remaining):
            st = strokes[sid]
            if len(st) == 0:
                continue
            p0 = st[0]
            p1 = st[-1]

            k0 = pick_key_lefttop(p0)
            k1 = pick_key_lefttop(p1)

            if best is None or k0 < best[0]:
                best = (k0, ridx, True)
            if best is None or k1 < best[0]:
                best = (k1, ridx, False)

        if best is None:
            break

        _, ridx, is_start = best
        sid = remaining[ridx]
        st = strokes[sid]

        if not is_start:
            st = st[::-1]

        ordered.append(st)
        remaining.pop(ridx)

    return ordered


def fit_stroke_polynomial(stroke: List[Point], deg: int = 5) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Fit x(t) and y(t) with polynomials.
    - x := col
    - y := row
    Returns:
    - coef_x, coef_y in np.polyfit format (highest power first)
    - t used for fitting
    """
    n = len(stroke)
    if n < max(3, deg + 1):
        deg2 = max(1, min(2, n - 1))
    else:
        deg2 = deg

    t = np.linspace(0.0, 1.0, n, endpoint=True)
    y = np.array([p[0] for p in stroke], dtype=np.float32)  # row
    x = np.array([p[1] for p in stroke], dtype=np.float32)  # col

    coef_x = np.polyfit(t, x, deg2)
    coef_y = np.polyfit(t, y, deg2)
    return coef_x, coef_y, t


def eval_poly(coef: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Evaluate polynomial given np.polyfit coefficients."""
    return np.polyval(coef, t)


# -------------------- Main --------------------

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
    TH_H = 50  # 15
    TH_W = 50  # 10
    selem_tophat = rectangle(TH_H, TH_W)
    tophat = white_tophat(inv, selem_tophat)

    # --- Step 3: Binarization after top-hat ---
    TH_T = 0.2
    binary2 = tophat > TH_T

    # --- Step 4: Opening ---
    OPEN_R = 2
    opened = opening(binary2, disk(OPEN_R))

    # --- Step 5: Closing (+ extra opening) ---
    CLOSE_R = 2
    cleaned = closing(opened, disk(CLOSE_R))
    cleaned = opening(cleaned, disk(OPEN_R))

    # --- Step 6: Medial axis skeleton ---
    medial_skel, dist = medial_axis(cleaned, return_distance=True)

    # ==========================================================
    # =============== New path planning method =================
    # ==========================================================

    # 1) fast neighbor counting (8-neighbors) with conv + padding
    cnt = neighbor_count_8(medial_skel)

    # 2) classify skeleton pixels
    cls = classify_pixels(medial_skel, cnt)
    isolated = cls["isolated"]
    endpoints = cls["endpoints"]
    track = cls["track"]
    junctions = cls["junctions"]

    print("[INFO] isolated: {}, endpoints: {}, track: {}, junctions(raw): {}".format(
        len(isolated), len(endpoints), len(track), len(junctions)
    ))

    # 3) cluster junctions (noise removal) and map to representative
    junction_list = sorted(list(junctions))
    J_RADIUS = 20.0  # you said this will be tuned later
    clusters, jmap = cluster_junctions(junction_list, radius=J_RADIUS)

    # representatives set
    junction_reps = set(jmap.values())

    print("[INFO] junction clusters: {}, junction reps: {}".format(len(clusters), len(junction_reps)))

    # 4) trace "lines" (endpoint/junction -> track points -> endpoint/junction)
    lines = trace_lines(
        skel=medial_skel,
        cnt=cnt,
        endpoints=endpoints,
        junctions=junctions,
        jmap=jmap,
    )

    print("[INFO] lines (before cycles): {}".format(len(lines)))

    # -------- NEW: extract ring-like cycles (all track points) --------
    # recompute used_track from current lines (track points only)
    used_track_from_lines: Set[Point] = set()
    for ln in lines:
        for p in ln.points[1:-1]:
            if cnt[p] == 2:
                used_track_from_lines.add(p)

    cycle_lines, used_track_from_lines = extract_cycle_lines(
        skel=medial_skel,
        cnt=cnt,
        used_track=used_track_from_lines,
        existing_ids_start=len(lines),
    )
    if len(cycle_lines) > 0:
        lines.extend(cycle_lines)

    # IMPORTANT: re-index line ids to match list indices (avoid any id/index mismatch)
    for new_id, ln in enumerate(lines):
        ln.id = new_id

    print("[INFO] lines (after cycles): {}".format(len(lines)))
    assert all(0 <= ln.id < len(lines) for ln in lines), "Line.id is not consistent with lines list indices"


    print("[INFO] lines (before filter): {}".format(len(lines)))

    # --- NEW: filter short lines attached to junctions ---
    MIN_JUNC_LINE_LEN = 35
    lines = filter_short_junction_lines(
        lines=lines,
        junction_reps=junction_reps,
        min_len=MIN_JUNC_LINE_LEN,
    )
    print("[INFO] lines (after filter): {}".format(len(lines)))

    # (optional but recommended) sanity check to prevent index bugs
    assert all(0 <= ln.id < len(lines) for ln in lines), "Line.id is not consistent with lines list indices"

    dsu, node_to_lines = merge_lines_into_strokes(
        lines=lines,
        junction_reps=junction_reps,
        k_dir=5,
    )

    strokes = build_stroke_polylines(lines, dsu)
    print("[INFO] strokes: {}".format(len(strokes)))

    # --- Smoothing parameters (already added before) ---
    SMOOTH_WINDOW = 9  # TODO: tune later (odd number recommended)
    SMOOTH_ITERS = 2   # TODO: tune later
    strokes_smooth = [smooth_polyline_moving_average(st, window=SMOOTH_WINDOW, iters=SMOOTH_ITERS) for st in strokes]
    print("[INFO] smoothing: window={}, iters={}".format(SMOOTH_WINDOW, SMOOTH_ITERS))

    # ==========================================================
    # ===================== Path planning ======================
    # ==========================================================

    ordered_strokes = order_strokes_by_lefttop_endpoints(strokes_smooth)
    print("[INFO] ordered_strokes: {}".format(len(ordered_strokes)))

    POLY_DEG = 50  # TODO: tune later
    poly_list = []  # list of (coef_x, coef_y) or None if failed

    for i, st in enumerate(ordered_strokes):
        try:
            coef_x, coef_y, t_fit = fit_stroke_polynomial(st, deg=POLY_DEG)
            poly_list.append((coef_x, coef_y))

            print("\n[POLY] Stroke {}: n_points={}, deg_x={}, deg_y={}".format(
                i, len(st), len(coef_x) - 1, len(coef_y) - 1
            ))
            print("[POLY] x(t) coefficients (highest->lowest): {}".format(coef_x))
            print("[POLY] y(t) coefficients (highest->lowest): {}".format(coef_y))
        except Exception as e:
            poly_list.append(None)
            print("\n[POLY] Stroke {}: FIT FAILED, n_points={}, err={}".format(i, len(st), str(e)))

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

    # ==========================================================
    # ===================== Visualization ======================
    # ==========================================================
    fig, axes = plt.subplots(2, 3, figsize=(15, 9), sharex=True, sharey=True)
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

    # Panel 4: node classification + junction reps
    ax[4].imshow(medial_skel, cmap=plt.cm.gray)
    ax[4].axis("off")
    ax[4].set_title("classified nodes + junction reps", fontsize=12)

    if len(isolated) > 0:
        iso = np.array(list(isolated))
        ax[4].scatter(iso[:, 1], iso[:, 0], s=18, marker="s")
    if len(endpoints) > 0:
        ep = np.array(list(endpoints))
        ax[4].scatter(ep[:, 1], ep[:, 0], s=14, marker="o")
    if len(junctions) > 0:
        jn = np.array(list(junctions))
        ax[4].scatter(jn[:, 1], jn[:, 0], s=10, marker="x")
    if len(junction_reps) > 0:
        jr = np.array(list(junction_reps))
        ax[4].scatter(jr[:, 1], jr[:, 0], s=60, marker="+")  # representative

    # Panel 5: lines + strokes (overlay)
    ax[5].imshow(medial_skel, cmap=plt.cm.gray)
    ax[5].axis("off")
    ax[5].set_title("lines and smoothed strokes (overlay)", fontsize=12)

    # draw lines thin
    for ln in lines:
        rr = np.array([p[0] for p in ln.points], dtype=np.float32)
        cc = np.array([p[1] for p in ln.points], dtype=np.float32)
        ax[5].plot(cc, rr, linewidth=0.8)

    # draw smoothed strokes thicker
    for st in strokes_smooth:
        rr = np.array([p[0] for p in st], dtype=np.float32)
        cc = np.array([p[1] for p in st], dtype=np.float32)
        ax[5].plot(cc, rr, linewidth=2.0)

        # mark stroke ends
        ax[5].scatter([cc[0]], [rr[0]], s=25)
        ax[5].scatter([cc[-1]], [rr[-1]], s=25)

    fig.tight_layout()
    plt.show()

    # fitting visualization
    fig2, axes2 = plt.subplots(1, 2, figsize=(14, 6), sharex=True, sharey=True)
    ax2 = axes2.ravel()

    ax2[0].imshow(medial_skel, cmap=plt.cm.gray)
    ax2[0].axis("off")
    ax2[0].set_title("ordered strokes (direction)", fontsize=12)

    for i, st in enumerate(ordered_strokes):
        rr = np.array([p[0] for p in st], dtype=np.float32)
        cc = np.array([p[1] for p in st], dtype=np.float32)
        ax2[0].plot(cc, rr, linewidth=2.0)
        ax2[0].scatter([cc[0]], [rr[0]], s=30)
        ax2[0].text(cc[0], rr[0], str(i), fontsize=10)

    ax2[1].imshow(medial_skel, cmap=plt.cm.gray)
    ax2[1].axis("off")
    ax2[1].set_title("polynomial fits (x(t), y(t))", fontsize=12)

    for i, st in enumerate(ordered_strokes):
        if i >= len(poly_list):
            continue
        if poly_list[i] is None:
            continue

        coef_x, coef_y = poly_list[i]

        t_dense = np.linspace(0.0, 1.0, 300, endpoint=True)
        x_dense = eval_poly(coef_x, t_dense)  # col
        y_dense = eval_poly(coef_y, t_dense)  # row
        ax2[1].plot(x_dense, y_dense, linewidth=2.0)
        ax2[1].scatter([x_dense[0]], [y_dense[0]], s=30)
        ax2[1].text(x_dense[0], y_dense[0], str(i), fontsize=10)

    fig2.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
