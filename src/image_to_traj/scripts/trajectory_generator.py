#!/usr/bin/env python3
import os
from typing import Optional, List, Tuple, Dict, Set

import numpy as np

from skimage.io import imread
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

import xml.etree.ElementTree as ET

# Optional speedups
try:
    from scipy.ndimage import convolve as ndi_convolve
except Exception:
    ndi_convolve = None

try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None


Point = Tuple[int, int]  # (row, col)


class TrajectoryGenerator:
    """
    Input: image path
    Output: XML string containing ordered strokes, polynomial fit params, and fitted curve length.
    """

    class Line:
        def __init__(self, lid: int, points: List[Point], start_node: Point, end_node: Point):
            self.id = lid
            self.points = points
            self.start = start_node
            self.end = end_node

        def length(self) -> int:
            return len(self.points)

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

    def __init__(
        self,
        # preprocessing params
        th_h: int = 50,
        th_w: int = 50,
        th_t: float = 0.2,
        open_r: int = 2,
        close_r: int = 2,
        # junction clustering + filtering
        j_radius: float = 20.0,
        min_junc_line_len: int = 35,
        # merging/smoothing
        k_dir: int = 5,
        smooth_window: int = 9,
        smooth_iters: int = 2,
        # polynomial fit + curve length
        poly_deg: int = 50,
        length_samples: int = 1200,
    ):
        self.th_h = th_h
        self.th_w = th_w
        self.th_t = th_t
        self.open_r = open_r
        self.close_r = close_r

        self.j_radius = float(j_radius)
        self.min_junc_line_len = int(min_junc_line_len)

        self.k_dir = int(k_dir)
        self.smooth_window = int(smooth_window)
        self.smooth_iters = int(smooth_iters)

        self.poly_deg = int(poly_deg)
        self.length_samples = int(length_samples)

    # -------------------- Public API --------------------

    def generate_xml(self, image_path: str) -> str:
        """
        Main entry:
          - reads image
          - extracts ordered strokes
          - polynomial fit each stroke
          - compute fitted-curve length
          - returns XML string
        """
        gray = self._load_gray(image_path)
        cleaned = self._preprocess(gray)
        skel, cnt = self._skeleton_and_cnt(cleaned)

        cls = self._classify_pixels(skel, cnt)
        endpoints = cls["endpoints"]
        junctions = cls["junctions"]

        junction_list = sorted(list(junctions))
        _, jmap = self._cluster_junctions(junction_list, radius=self.j_radius)
        junction_reps = set(jmap.values())

        lines = self._trace_lines(
            skel=skel,
            cnt=cnt,
            endpoints=endpoints,
            junctions=junctions,
            jmap=jmap,
        )

        # add ring-like cycles
        used_track = self._used_track_from_lines(lines, cnt)
        cycle_lines, used_track = self._extract_cycle_lines(skel, cnt, used_track, existing_ids_start=len(lines))
        if cycle_lines:
            lines.extend(cycle_lines)
        for new_id, ln in enumerate(lines):
            ln.id = new_id

        # filter short junction-attached lines
        lines = self._filter_short_junction_lines(lines, junction_reps, min_len=self.min_junc_line_len)

        # merge lines into strokes
        dsu, _ = self._merge_lines_into_strokes(lines, junction_reps, k_dir=self.k_dir)
        strokes = self._build_stroke_polylines(lines, dsu)

        # smoothing
        strokes_smooth = [self._smooth_polyline_moving_average(st, self.smooth_window, self.smooth_iters) for st in strokes]

        # ordering
        ordered_strokes = self._order_strokes_by_lefttop_endpoints(strokes_smooth)

        # fit + length
        stroke_infos = []
        for i, st in enumerate(ordered_strokes):
            coef_x, coef_y, deg_used = self._fit_stroke_polynomial(st, deg=self.poly_deg)
            L = self._polyfit_curve_length(coef_x, coef_y, n=self.length_samples)
            stroke_infos.append({
                "index": i,
                "deg": deg_used,
                "coef_x": coef_x,
                "coef_y": coef_y,
                "length_px": L,
                "n_points": len(st),
            })

        # build XML
        return self._to_xml_string(
            image_path=image_path,
            stroke_infos=stroke_infos,
        )

    # -------------------- Core steps --------------------

    def _load_gray(self, image_path: str) -> np.ndarray:
        img = imread(image_path)
        if img.ndim == 3:
            gray = rgb2gray(img)  # [0,1]
        else:
            gray = img.astype(np.float32)
            if gray.max() > 1.0:
                gray /= 255.0
        return gray.astype(np.float32)

    def _preprocess(self, gray: np.ndarray) -> np.ndarray:
        # Step 1: Otsu on gray
        t1 = threshold_otsu(gray)
        _binary1 = gray < t1  # kept for compatibility, but not used further

        # Step 2: white top-hat on inverted
        inv = 1.0 - gray
        selem_tophat = rectangle(self.th_h, self.th_w)
        tophat = white_tophat(inv, selem_tophat)

        # Step 3: threshold after top-hat
        binary2 = tophat > float(self.th_t)

        # Step 4: opening
        opened = opening(binary2, disk(self.open_r))

        # Step 5: closing + extra opening
        cleaned = closing(opened, disk(self.close_r))
        cleaned = opening(cleaned, disk(self.open_r))
        return cleaned.astype(bool)

    def _skeleton_and_cnt(self, cleaned: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        skel, _dist = medial_axis(cleaned, return_distance=True)
        cnt = self._neighbor_count_8(skel)
        return skel.astype(bool), cnt.astype(np.uint8)

    def _neighbor_count_8(self, skel: np.ndarray) -> np.ndarray:
        sk = skel.astype(np.uint8)
        kernel = np.array([[1,1,1],[1,0,1],[1,1,1]], dtype=np.uint8)

        if ndi_convolve is not None:
            cnt = ndi_convolve(sk, kernel, mode="constant", cval=0)
            return cnt.astype(np.uint8)

        H, W = sk.shape
        pad = np.pad(sk, ((1,1),(1,1)), mode="constant", constant_values=0)
        cnt = np.zeros((H, W), dtype=np.uint8)
        for r in range(H):
            for c in range(W):
                win = pad[r:r+3, c:c+3]
                cnt[r, c] = int(np.sum(win * kernel))
        return cnt

    def _classify_pixels(self, skel: np.ndarray, cnt: np.ndarray) -> Dict[str, Set[Point]]:
        sk = skel.astype(bool)
        iso = set(map(tuple, np.argwhere(sk & (cnt == 0)).astype(int)))
        endp = set(map(tuple, np.argwhere(sk & (cnt == 1)).astype(int)))
        track = set(map(tuple, np.argwhere(sk & (cnt == 2)).astype(int)))
        junc = set(map(tuple, np.argwhere(sk & (cnt >= 3)).astype(int)))
        return {"isolated": iso, "endpoints": endp, "track": track, "junctions": junc}

    def _neighbors8(self, p: Point, shape: Tuple[int,int]) -> List[Point]:
        r, c = p
        H, W = shape
        out = []
        for dr in (-1,0,1):
            for dc in (-1,0,1):
                if dr == 0 and dc == 0:
                    continue
                rr, cc = r+dr, c+dc
                if 0 <= rr < H and 0 <= cc < W:
                    out.append((rr, cc))
        return out

    def _cluster_junctions(self, junctions: List[Point], radius: float) -> Tuple[List[List[Point]], Dict[Point, Point]]:
        if len(junctions) == 0:
            return [], {}

        pts = np.array(junctions, dtype=np.float32)

        if cKDTree is not None:
            tree = cKDTree(pts)
            nbrs = tree.query_ball_tree(tree, r=float(radius))
        else:
            nbrs = []
            for i in range(len(pts)):
                d = np.linalg.norm(pts - pts[i], axis=1)
                nbrs.append(list(np.where(d <= float(radius))[0]))

        visited = np.zeros(len(pts), dtype=bool)
        clusters: List[List[Point]] = []
        map_old_to_rep: Dict[Point, Point] = {}

        for i in range(len(pts)):
            if visited[i]:
                continue
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

            arr = np.array(cluster, dtype=np.int32)
            rep_r = int(np.median(arr[:, 0]))
            rep_c = int(np.median(arr[:, 1]))
            rep = (rep_r, rep_c)

            for p in cluster:
                map_old_to_rep[p] = rep

        return clusters, map_old_to_rep

    def _trace_lines(
        self,
        skel: np.ndarray,
        cnt: np.ndarray,
        endpoints: Set[Point],
        junctions: Set[Point],
        jmap: Dict[Point, Point],
    ) -> List["TrajectoryGenerator.Line"]:
        H, W = skel.shape
        sk = skel.astype(bool)

        node_pixels = set(endpoints) | set(junctions)

        used_track: Set[Point] = set()
        used_directed: Set[Tuple[Point, Point]] = set()

        def canon_node(p: Point) -> Point:
            return jmap.get(p, p)

        lines: List[TrajectoryGenerator.Line] = []
        lid = 0

        for node in sorted(node_pixels):
            if not sk[node]:
                continue

            nbs = [q for q in self._neighbors8(node, (H, W)) if sk[q]]
            for nb in nbs:
                if (node, nb) in used_directed:
                    continue

                pts = [node]
                prev = node
                cur = nb
                used_directed.add((node, nb))

                if (cnt[cur] == 2) and (cur in used_track):
                    continue

                while True:
                    pts.append(cur)

                    if cur in node_pixels and cur != node:
                        break

                    nbrs = [q for q in self._neighbors8(cur, (H, W)) if sk[q]]
                    if len(nbrs) == 0:
                        break

                    nbrs2 = [q for q in nbrs if q != prev]
                    if len(nbrs2) == 0:
                        break

                    nxt = None
                    for cand in nbrs2:
                        if (cur, cand) not in used_directed:
                            nxt = cand
                            break
                    if nxt is None:
                        nxt = nbrs2[0]

                    used_directed.add((cur, nxt))
                    prev, cur = cur, nxt

                    if (cnt[cur] == 2) and (cur in used_track):
                        pts.pop()
                        break

                if len(pts) < 2:
                    continue

                raw_start = pts[0]
                raw_end = pts[-1]
                start_node = canon_node(raw_start) if raw_start in node_pixels else raw_start
                end_node = canon_node(raw_end) if raw_end in node_pixels else raw_end

                for p in pts[1:-1]:
                    if cnt[p] == 2:
                        used_track.add(p)

                lines.append(self.Line(lid, pts, start_node, end_node))
                lid += 1

        # drop tiny self-loops
        filtered = []
        for ln in lines:
            if ln.start == ln.end and ln.length() <= 3:
                continue
            filtered.append(ln)

        for new_id, ln in enumerate(filtered):
            ln.id = new_id

        return filtered

    def _used_track_from_lines(self, lines: List["TrajectoryGenerator.Line"], cnt: np.ndarray) -> Set[Point]:
        used = set()
        for ln in lines:
            for p in ln.points[1:-1]:
                if cnt[p] == 2:
                    used.add(p)
        return used

    def _extract_cycle_lines(
        self,
        skel: np.ndarray,
        cnt: np.ndarray,
        used_track: Set[Point],
        existing_ids_start: int,
    ) -> Tuple[List["TrajectoryGenerator.Line"], Set[Point]]:
        H, W = skel.shape
        sk = skel.astype(bool)

        remain = set(map(tuple, np.argwhere(sk & (cnt == 2)).astype(int))) - set(used_track)
        visited: Set[Point] = set()

        new_lines: List[TrajectoryGenerator.Line] = []
        lid = existing_ids_start

        max_steps = int(np.count_nonzero(skel)) + 10

        for start in list(remain):
            if start in visited or start not in remain:
                continue

            nbs0 = [q for q in self._neighbors8(start, (H, W)) if sk[q]]
            if len(nbs0) < 2:
                visited.add(start)
                continue

            prev = start
            cur = nbs0[0]
            cycle_pts = [start]

            visited_local: Set[Point] = set([start])
            closed = False

            for _ in range(max_steps):
                cycle_pts.append(cur)

                if cur == start:
                    closed = True
                    break

                if cur in visited_local:
                    break

                visited_local.add(cur)

                nbs = [q for q in self._neighbors8(cur, (H, W)) if sk[q]]
                if len(nbs) == 0:
                    break
                nbs2 = [q for q in nbs if q != prev]
                if len(nbs2) == 0:
                    break

                nxt = nbs2[0]
                prev, cur = cur, nxt

            if closed and len(cycle_pts) >= 4:
                for p in cycle_pts[:-1]:
                    visited.add(p)
                    used_track.add(p)
                new_lines.append(self.Line(lid, cycle_pts, start, start))
                lid += 1
            else:
                visited |= visited_local

        return new_lines, used_track

    def _filter_short_junction_lines(self, lines: List["TrajectoryGenerator.Line"], junction_reps: Set[Point], min_len: int) -> List["TrajectoryGenerator.Line"]:
        kept = []
        for ln in lines:
            is_short = ln.length() < int(min_len)
            touches_j = (ln.start in junction_reps) or (ln.end in junction_reps)
            if is_short and touches_j:
                continue
            kept.append(ln)
        for new_id, ln in enumerate(kept):
            ln.id = new_id
        return kept

    def _direction_at_node(self, line: "TrajectoryGenerator.Line", node: Point, k: int) -> np.ndarray:
        pts = line.points
        if len(pts) < 2:
            return np.array([0.0, 0.0], dtype=np.float32)

        if node == line.start:
            p0 = np.array(pts[0], dtype=np.float32)
            p1 = np.array(pts[min(k, len(pts) - 1)], dtype=np.float32)
            v = p1 - p0
        elif node == line.end:
            p0 = np.array(pts[-1], dtype=np.float32)
            p1 = np.array(pts[max(0, len(pts) - 1 - k)], dtype=np.float32)
            v = p1 - p0
        else:
            return np.array([0.0, 0.0], dtype=np.float32)

        n = float(np.linalg.norm(v))
        if n < 1e-6:
            return np.array([0.0, 0.0], dtype=np.float32)
        return v / n

    def _angle_for_pair(self, v1: np.ndarray, v2: np.ndarray) -> float:
        a = v1
        b = -v2
        dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
        return float(np.arccos(dot))

    def _merge_lines_into_strokes(
        self,
        lines: List["TrajectoryGenerator.Line"],
        junction_reps: Set[Point],
        k_dir: int,
    ) -> Tuple["TrajectoryGenerator.DSU", Dict[Point, List[int]]]:
        n = len(lines)
        dsu = self.DSU(n)

        node_to_lines: Dict[Point, List[int]] = {}
        for ln in lines:
            node_to_lines.setdefault(ln.start, []).append(ln.id)
            node_to_lines.setdefault(ln.end, []).append(ln.id)

        for j in sorted(junction_reps):
            inc = node_to_lines.get(j, [])
            if len(inc) < 2:
                continue

            dirs: Dict[int, np.ndarray] = {}
            for lid in inc:
                dirs[lid] = self._direction_at_node(lines[lid], j, k=int(k_dir))

            remaining = set(inc)
            while len(remaining) >= 2:
                best = None
                rem_list = list(remaining)
                for a in range(len(rem_list)):
                    for b in range(a + 1, len(rem_list)):
                        l1, l2 = rem_list[a], rem_list[b]
                        ang = self._angle_for_pair(dirs[l1], dirs[l2])
                        if best is None or ang < best[0]:
                            best = (ang, l1, l2)

                if best is None:
                    break

                _, l1, l2 = best
                dsu.union(l1, l2)
                remaining.remove(l1)
                remaining.remove(l2)

        return dsu, node_to_lines

    def _build_stroke_polylines(self, lines: List["TrajectoryGenerator.Line"], dsu: "TrajectoryGenerator.DSU") -> List[List[Point]]:
        groups: Dict[int, List[int]] = {}
        for ln in lines:
            g = dsu.find(ln.id)
            groups.setdefault(g, []).append(ln.id)

        stroke_polys: List[List[Point]] = []

        for _g, lids in groups.items():
            if len(lids) == 1:
                stroke_polys.append(lines[lids[0]].points)
                continue

            node_adj: Dict[Point, List[int]] = {}
            for lid in lids:
                ln = lines[lid]
                node_adj.setdefault(ln.start, []).append(lid)
                node_adj.setdefault(ln.end, []).append(lid)

            start_node = None
            for n, inc in node_adj.items():
                if len(inc) == 1:
                    start_node = n
                    break
            if start_node is None:
                start_node = lines[lids[0]].start

            used_line: Set[int] = set()
            cur_node = start_node
            stroke_pts: List[Point] = []

            while True:
                inc = [lid for lid in node_adj.get(cur_node, []) if lid not in used_line]
                if len(inc) == 0:
                    break

                lid = inc[0]
                ln = lines[lid]

                if ln.start == cur_node:
                    seg = ln.points
                    nxt_node = ln.end
                else:
                    seg = ln.points[::-1]
                    nxt_node = ln.start

                if len(stroke_pts) == 0:
                    stroke_pts.extend(seg)
                else:
                    stroke_pts.extend(seg[1:])

                used_line.add(lid)
                cur_node = nxt_node

            stroke_polys.append(stroke_pts)

        return stroke_polys

    # -------------------- Smoothing + ordering --------------------

    def _smooth_polyline_moving_average(self, poly: List[Point], window: int, iters: int) -> List[Point]:
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

            out[0] = arr[0]
            out[-1] = arr[-1]

        out_int = np.rint(out).astype(np.int32)
        return [tuple(map(int, p)) for p in out_int]

    def _pick_key_lefttop(self, p: Point) -> Tuple[int, int]:
        return (p[1], p[0])  # (col, row)

    def _order_strokes_by_lefttop_endpoints(self, strokes: List[List[Point]]) -> List[List[Point]]:
        remaining = list(range(len(strokes)))
        ordered: List[List[Point]] = []

        while remaining:
            best = None  # (key, ridx_in_remaining, endpoint_is_start)
            for ridx, sid in enumerate(remaining):
                st = strokes[sid]
                if not st:
                    continue
                p0 = st[0]
                p1 = st[-1]
                k0 = self._pick_key_lefttop(p0)
                k1 = self._pick_key_lefttop(p1)

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

    # -------------------- Polynomial fit + fitted curve length --------------------

    def _fit_stroke_polynomial(self, stroke: List[Point], deg: int) -> Tuple[np.ndarray, np.ndarray, int]:
        n = len(stroke)
        if n < 2:
            # degenerate
            return np.array([0.0], dtype=np.float64), np.array([0.0], dtype=np.float64), 0

        if n < max(3, deg + 1):
            deg2 = max(1, min(2, n - 1))
        else:
            deg2 = int(deg)

        t = np.linspace(0.0, 1.0, n, endpoint=True)
        y = np.array([p[0] for p in stroke], dtype=np.float64)  # row
        x = np.array([p[1] for p in stroke], dtype=np.float64)  # col

        coef_x = np.polyfit(t, x, deg2)
        coef_y = np.polyfit(t, y, deg2)
        return coef_x.astype(np.float64), coef_y.astype(np.float64), deg2

    def _polyfit_curve_length(self, coef_x: np.ndarray, coef_y: np.ndarray, n: int) -> float:
        """
        Approximate fitted curve length by dense sampling on t in [0,1]:
          L ~= sum_i sqrt((dx)^2 + (dy)^2)
        Here x=col, y=row (pixel units).
        """
        nn = max(50, int(n))
        t = np.linspace(0.0, 1.0, nn, endpoint=True)
        x = np.polyval(coef_x, t)
        y = np.polyval(coef_y, t)
        dx = np.diff(x)
        dy = np.diff(y)
        return float(np.sum(np.sqrt(dx * dx + dy * dy)))

    # -------------------- XML --------------------

    def _to_xml_string(self, image_path: str, stroke_infos: List[Dict]) -> str:
        root = ET.Element("trajectory")
        meta = ET.SubElement(root, "meta")
        ET.SubElement(meta, "image_path").text = str(image_path)
        ET.SubElement(meta, "image_name").text = os.path.basename(image_path)
        ET.SubElement(meta, "poly_degree_requested").text = str(self.poly_deg)
        ET.SubElement(meta, "length_samples").text = str(self.length_samples)
        ET.SubElement(meta, "length_unit").text = "px"

        strokes_el = ET.SubElement(root, "strokes")
        strokes_el.set("count", str(len(stroke_infos)))

        for info in stroke_infos:
            st_el = ET.SubElement(strokes_el, "stroke")
            st_el.set("index", str(info["index"]))
            st_el.set("deg_used", str(info["deg"]))
            st_el.set("n_points", str(info["n_points"]))

            length_el = ET.SubElement(st_el, "length_px")
            length_el.text = f"{info['length_px']:.6f}"

            fit_el = ET.SubElement(st_el, "polyfit")
            x_el = ET.SubElement(fit_el, "x")
            y_el = ET.SubElement(fit_el, "y")

            # coefficients: highest power -> lowest power
            x_el.text = ",".join([self._fmt_float(v) for v in info["coef_x"].tolist()])
            y_el.text = ",".join([self._fmt_float(v) for v in info["coef_y"].tolist()])

        # pretty-ish string (without external libs)
        xml_bytes = ET.tostring(root, encoding="utf-8")
        return xml_bytes.decode("utf-8")

    def _fmt_float(self, v: float) -> str:
        # compact but stable
        return f"{float(v):.12g}"


# -------------------- Example usage --------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=str, required=True, help="Absolute or relative image path")
    args = parser.parse_args()

    gen = TrajectoryGenerator()
    xml_str = gen.generate_xml(args.image)
    print(xml_str)
