"""Plánovač orientací kostek pro barevně navazující řetěz.

Pojmy:
- pairs: barvy kostek v pořadí NABÍRÁNÍ, každá jako [barva1, barva2].
- physical_order: 1-based indexy nabraných kostek tak, jak po vyložení leží
  v řadě od PŘEDNÍHO konce (konec směrem k přídi robota) dozadu.
- orientace kostky = (front, rear): barva mířící k přídi robota / k zádi.
- sousední kostky v řadě navazují, když rear(pozice k) == front(pozice k+1).

Skóre odpovídá pravidlům: segment n>=2 navazujících kostek = n*n, osamocená = 1.
Spuštění `python3 planner.py` ověří všechny řady v mission.json.
"""


def line_segments(line):
    """Délky navazujících segmentů v řadě [(front, rear), ...]."""
    segs = []
    n = 1
    for k in range(1, len(line)):
        if line[k - 1][1] == line[k][0]:
            n += 1
        else:
            segs.append(n)
            n = 1
    segs.append(n)
    return segs


def score_segments(segs):
    return sum(n * n if n >= 2 else n for n in segs)


def plan_line(pairs, physical_order=None, join=None):
    """Najde nejlepší orientace (hrubou silou, max 2^4 = 16 kombinací).

    join: (end, colour, existing_len) – konec nové řady ('front'/'rear'), který
    se přiloží k existujícímu řetězu s volnou barvou `colour` a délkou existing_len.
    """
    n = len(pairs)
    if physical_order:
        order = [i - 1 for i in physical_order if i <= n]
    else:
        order = list(range(n))
    if sorted(order) != list(range(n)):
        raise ValueError("physical_order neodpovídá počtu kostek: {}".format(physical_order))

    best = None
    for mask in range(1 << n):
        orient = []
        for i, p in enumerate(pairs):
            orient.append((p[1], p[0]) if (mask >> i) & 1 else (p[0], p[1]))
        line = [orient[i] for i in order]
        segs = line_segments(line)
        sc = score_segments(segs)
        ends = {"front": line[0][0], "rear": line[-1][1]}
        join_ok = False
        total = sc
        if join is not None:
            end, colour, existing = join
            join_ok = ends[end] == colour
            if join_ok:
                seg = segs[0] if end == "front" else segs[-1]
                total += 2 * existing * seg  # (E+L)^2 - E^2 - L^2
        key = (total, join_ok, -mask)
        if best is None or key > best[0]:
            best = (key, {
                "orient": orient,          # podle pořadí nabírání
                "line": [i + 1 for i in order],
                "segments": segs,
                "score": sc,
                "total": total,
                "ends": ends,
                "join_ok": join_ok,
            })
    return best[1]


def format_plan(pairs, plan):
    rows = []
    for idx, (f, r) in enumerate(plan["orient"]):
        rows.append("  kostka {} {}/{} -> front={} rear={}".format(
            idx + 1, pairs[idx][0], pairs[idx][1], f, r))
    rows.append("  řada (od přídě): {}  segmenty={} skóre={} konce={} join_ok={}".format(
        plan["line"], plan["segments"], plan["score"], plan["ends"], plan["join_ok"]))
    return "\n".join(rows)


def _selftest(path="mission.json"):
    import json
    with open(path) as f:
        cfg = json.load(f)
    order = cfg["robot"]["pick"]["physical_order"]
    ok = True
    for side, rows in cfg["field"]["layouts"].items():
        for row, pairs in rows.items():
            if row.startswith("_"):
                continue
            for rev in (False, True):
                ps = list(reversed(pairs)) if rev else list(pairs)
                plan = plan_line(ps, order)
                full = plan["segments"] == [len(ps)]
                ok = ok and full
                print("{:5} {:6} {:9} {} skóre={:2} konce={}".format(
                    side, row, "reverse" if rev else "forward",
                    "OK  " if full else "FAIL", plan["score"], plan["ends"]))
    print("VŠE OK" if ok else "NĚKTERÉ ŘADY NETVOŘÍ ÚPLNÝ ŘETĚZ")
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if _selftest(*(sys.argv[1:2])) else 1)
