import math
import sys
import xml.etree.ElementTree as ET

MERGE_DIST = 30   # m: gộp các trạm gần nhau hơn mức này
STOP_MATCH = 50   # m: trạm trong tuyến bus phải nằm gần một trạm của đồ thị
SKIP_HIGHWAY = {"motorway", "motorway_link", "trunk", "trunk_link"}


def haversine(a, b):   #tính trọng số khoảng cách bằng công thức toán học
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * 6371000 * math.asin(math.sqrt(h))


def flat2(a, b):
    # bình phương khoảng cách xấp xỉ, chỉ dùng để so sánh nhanh
    dx = (a[1] - b[1]) * math.cos(math.radians(a[0]))
    return dx * dx + (a[0] - b[0]) ** 2


def read_osm(path):
    root = ET.parse(path).getroot()
    nodes, tags, ways, rels = {}, {}, {}, []
    for e in root:
        t = {x.get("k"): x.get("v") for x in e.findall("tag")}
        if e.tag == "node":
            i = int(e.get("id"))
            nodes[i] = (float(e.get("lat")), float(e.get("lon")))
            if t:
                tags[i] = t
        elif e.tag == "way":
            ways[int(e.get("id"))] = ([int(n.get("ref")) for n in e.findall("nd")], t)
        elif e.tag == "relation":
            members = [(m.get("type"), int(m.get("ref")), m.get("role")) for m in e.findall("member")]
            rels.append((t, members))
    return nodes, tags, ways, rels


def walk_edges(nodes, ways):
    # cạnh giữa các node liên tiếp của way, chỉ giữ thành phần liên thông lớn nhất
    edges = []
    for nds, t in ways.values():
        if "highway" not in t or t["highway"] in SKIP_HIGHWAY:
            continue
        nds = [n for n in nds if n in nodes]
        edges += [(u, v) for u, v in zip(nds, nds[1:]) if u != v]

    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for u, v in edges:
        parent[find(u)] = find(v)
    size = {}
    for u in parent:
        r = find(u)
        size[r] = size.get(r, 0) + 1
    big = max(size, key=size.get)
    return [(u, v) for u, v in edges if find(u) == big]


def merge_stops(stops, nodes):
    # gộp trạm cách nhau < MERGE_DIST (chưa kiểm tra "cùng đường")
    reps = []
    for s in stops:
        if not any(haversine(nodes[s], nodes[r]) < MERGE_DIST for r in reps):
            reps.append(s)
    return reps


def chain_ways(way_ids, bways, bnodes):
    # nối các way của tuyến thành một dãy node liên tục theo chiều xe chạy
    seq, placed = [], 0
    for w in way_ids:
        nds = [n for n in bways[w][0] if n in bnodes]
        if not nds:
            continue
        placed += 1
        if not seq:
            seq = nds
            continue
        if placed == 2 and seq[-1] not in (nds[0], nds[-1]) and seq[0] in (nds[0], nds[-1]):
            seq.reverse()  # way đầu bị ngược chiều
        if seq[-1] == nds[0]:
            seq += nds[1:]
        elif seq[-1] == nds[-1]:
            seq += nds[-2::-1]
        else:
            seq += nds
    return seq


def main(walk_file="walk.osm", bus_file="bus.osm"):
    # ---------- file 1: đồ thị ----------
    nodes, tags, ways, _ = read_osm(walk_file)
    wedges = walk_edges(nodes, ways)
    road = sorted({n for e in wedges for n in e})
    road_set = set(road)
    stops = [i for i, t in tags.items() if t.get("highway") == "bus_stop"]
    reps = merge_stops(stops, nodes)
    rep_set = set(reps)

    gid = {n: i for i, n in enumerate(road)}
    for s in reps:
        gid.setdefault(s, len(gid))

    edges = []  # (u, v, length, kind, route_id); kind 0 = đi bộ, 1 = bus
    for u, v in wedges:
        d = haversine(nodes[u], nodes[v])
        edges += [(gid[u], gid[v], d, 0, -1), (gid[v], gid[u], d, 0, -1)]
    for s in reps:
        if s in road_set:
            continue  # trạm nằm sẵn trên đường
        near = min(road, key=lambda n: flat2(nodes[s], nodes[n]))
        d = haversine(nodes[s], nodes[near])
        edges += [(gid[s], gid[near], d, 0, -1), (gid[near], gid[s], d, 0, -1)]

    # ---------- file 2: tuyến bus ----------
    bnodes, _, bways, brels = read_osm(bus_file)

    def match_stop(p):
        r = min(reps, key=lambda r: flat2(p, nodes[r]))
        return r if haversine(p, nodes[r]) <= STOP_MATCH else None

    blocks, routes = [], []
    for t, members in brels:
        if t.get("route") != "bus":
            continue
        rid = len(routes)
        routes.append((rid, t.get("ref", ""), t.get("name", "")))
        way_ids = [r for typ, r, role in members if typ == "way" and r in bways and role in ("", "forward", "backward")]
        stop_refs = [r for typ, r, role in members if typ == "node" and r in bnodes and role.startswith(("stop", "platform"))]
        seq = chain_ways(way_ids, bways, bnodes)

        pos, prev = 0, None
        for sref in stop_refs:
            g = match_stop(bnodes[sref])
            if g is None or pos >= len(seq):
                continue
            i = min(range(pos, len(seq)), key=lambda j: flat2(bnodes[sref], bnodes[seq[j]]))
            if prev and prev[0] != g and i > prev[1]:
                pts = [bnodes[n] for n in seq[prev[1]:i + 1]]
                length = sum(haversine(a, b) for a, b in zip(pts, pts[1:]))
                edges.append((gid[prev[0]], gid[g], length, 1, rid))
                blocks.append((rid, gid[prev[0]], gid[g], pts))
            prev, pos = (g, i), i

    # ---------- ghi file ----------
    with open("A..MapNode.txt", "w", encoding="utf-8") as f:
        f.write(f"{len(gid)} {len(edges)}\n")
        for n in sorted(gid, key=gid.get):
            f.write(f"{gid[n]} {nodes[n][0]:.7f} {nodes[n][1]:.7f} {1 if n in rep_set else 0}\n")
        for u, v, d, kind, rid in edges:
            f.write(f"{u} {v} {d:.2f} {kind} {rid}\n")

    with open("A..DườngĐiXeBus.txt", "w", encoding="utf-8") as f:
        for rid, a, b, pts in blocks:
            f.write(f"{rid} {a} {b} {len(pts)}\n")
            for lat, lon in pts:
                f.write(f"{lat:.7f} {lon:.7f}\n")

    with open("A..CácTuyếnXeBus.txt", "w", encoding="utf-8") as f:
        for rid, ref, name in routes:
            f.write(f"{rid}\t{ref}\t{name}\n")

    print(f"node: {len(gid)} ({len(road)} đường, {len(reps)} trạm), cạnh: {len(edges)}")
    print(f"tuyến bus: {len(routes)}, đoạn trạm->trạm: {len(blocks)}")


if __name__ == "__main__":
    main(*sys.argv[1:3])