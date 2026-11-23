"""Vstupní bod. Na robotu: spustit main.py (mise podle `active_mission`).
Na PC:  python3 main.py --mock [název_mise] [--flip] [--side left|right]
"""
import json
import sys

import actions
from common import COLOURS, TimeUp
from mechanism import Mechanism

MISSION_FILE = "mission.json"


def load(path=MISSION_FILE):
    with open(path) as f:
        return json.load(f)


def validate(cfg):
    errs = []
    layouts = cfg["field"]["layouts"]
    for side, rows in layouts.items():
        for row, pairs in rows.items():
            if row.startswith("_"):
                continue
            for p in pairs:
                if len(p) != 2 or any(c not in COLOURS for c in p):
                    errs.append("layout {}.{}: špatná kostka {}".format(side, row, p))
    if cfg["side"] not in layouts:
        errs.append("side {} nemá layout".format(cfg["side"]))
    po = cfg["robot"]["pick"]["physical_order"]
    if sorted(po) != list(range(1, len(po) + 1)):
        errs.append("pick.physical_order není permutace: {}".format(po))
    for c in cfg["robot"]["colour"]["background"] + [cfg["robot"]["colour"]["line"]]:
        if c not in COLOURS:
            errs.append("neznámá barva {}".format(c))
    errs += Mechanism(None, cfg["robot"]["arm"]).check()
    stf = cfg["robot"]["pick"]["seen_to_front"]
    for v in (stf.values() if isinstance(stf, dict) else [stf]):
        if v not in ("CW", "CCW"):
            errs.append("pick.seen_to_front: neplatná hodnota {}".format(v))
    if isinstance(stf, dict) and set(stf) != {"LEFT", "RIGHT"}:
        errs.append("pick.seen_to_front: musí mít klíče LEFT a RIGHT")

    groups = [("missions." + k, v) for k, v in cfg["missions"].items() if not k.startswith("_")]
    groups += [("macros." + k, v) for k, v in cfg.get("macros", {}).items() if not k.startswith("_")]
    for where, steps in groups:
        for i, st in enumerate(steps):
            if not actions.active(st):
                continue
            a = actions.ACTIONS.get(st["cmd"])
            if a is None:
                errs.append("{}[{}]: neznámý cmd {}".format(where, i, st["cmd"]))
                continue
            for p in a[1]:
                if p not in st:
                    errs.append("{}[{}] {}: chybí parametr {}".format(where, i, st["cmd"], p))
            if st["cmd"] in ("pick_row", "pick_prepare") and st["row"] not in layouts.get(cfg["side"], {}):
                errs.append("{}[{}]: řada {} neexistuje".format(where, i, st["row"]))
            if st["cmd"] == "macro" and st["name"] not in cfg.get("macros", {}):
                errs.append("{}[{}]: makro {} neexistuje".format(where, i, st["name"]))
    return errs


def main(argv):
    flags = [a for a in argv if a.startswith("--")]
    args = [a for a in argv if not a.startswith("--")]
    mock = "--mock" in flags
    cfg = load()
    if "--side" in argv:
        cfg["side"] = argv[argv.index("--side") + 1]
        args = [a for a in args if a != cfg["side"]]
    name = args[0] if args else cfg["active_mission"]

    errs = validate(cfg)
    if name not in cfg["missions"]:
        errs.append("mise {} neexistuje".format(name))
    if errs:
        for e in errs:
            print("CHYBA:", e)
        return 1

    if mock:
        from hw_mock import MockHardware
        hw = MockHardware(cfg, flip="--flip" in flags)
    else:
        from hw import Hardware
        hw = Hardware(cfg)

    ctx = actions.Context(hw, cfg)
    limit = cfg.get("time_limit_s") or 0
    print("Mise '{}', strana {}, {}".format(name, cfg["side"],
          "limit {} s".format(limit) if limit else "BEZ časového limitu (soutěž: time_limit_s 88)"))
    if cfg.get("start_delay_s") and not mock:
        hw.sleep(cfg["start_delay_s"])
    ctx.t0 = hw.now()
    if limit:
        hw.set_deadline(ctx.t0 + limit)
    try:
        actions.run_steps(ctx, cfg["missions"][name])
        print("{} HOTOVO".format(ctx.stamp()))
    except TimeUp:
        print("{} ČAS VYPRŠEL – stop".format(ctx.stamp()))
    finally:
        hw.stop_all()
    print("Řetězy: {}".format(ctx.chains))
    if mock:
        print("Celkový čas: {:.1f} s, motor C celkem {:.0f} deg".format(hw.now() - ctx.t0, hw.arm_total))
    return 0


if __name__ == "__main__":
    sys.exit(main(getattr(sys, "argv", [""])[1:]) or 0)
