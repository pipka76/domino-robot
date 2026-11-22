"""Interpret příkazů z mission.json.

Každý krok mise je objekt {"cmd": "...", ...parametry}. Klíče začínající "_"
jsou komentáře, krok s "enabled": false se přeskočí.
Konvence: vzdálenosti v mm (záporné = couvání), úhly ve stupních, kladné = vlevo (CCW).
"""
import planner
from common import clamp, sgn, other_rot
from mechanism import Mechanism


class Context:
    def __init__(self, hw, cfg, log=print):
        self.hw = hw
        self.cfg = cfg
        self.log = log
        self.side = cfg["side"]
        self.drive = cfg["robot"]["drive"]
        self.ccfg = cfg["robot"]["colour"]
        self.pick = cfg["robot"]["pick"]
        self.mech = Mechanism(hw, cfg["robot"]["arm"], log)
        self.chains = {}      # id -> {"length", "ends"}
        self.loaded = None    # co je právě v robotu (výsledek pick_row)
        self.t0 = hw.now()

    def stamp(self):
        return "{:6.2f}s".format(self.hw.now() - self.t0)


# ---------------------------------------------------------------- primitiva

def _colour_set(ctx, spec):
    if spec == "background":
        return set(ctx.ccfg["background"])
    if spec == "line":
        return {ctx.ccfg["line"]}
    if isinstance(spec, str):
        return {spec}
    return set(spec)


def drive_mm(ctx, mm, speed=None, until=None, hold=True):
    """Jízda rovně o mm (enkodéry) s rampou, dobrzděním a držením směru gyrem.
    until: funkce bez parametrů -> True = zastavit. Vrací True, když zastavil `until`."""
    hw, d = ctx.hw, ctx.drive
    speed = speed or d["speed"]
    vmin = d.get("min_speed", 100)
    sign = 1 if mm >= 0 else -1
    target = abs(mm) * d["deg_per_mm"]
    hw.reset_wheels()
    h0 = hw.gyro_angle() if hold else None
    t0 = hw.now()
    timeout = target / float(max(speed, 1)) * 2 + 2
    hit = False
    while True:
        l, r = hw.wheel_angles()
        done = (abs(l) + abs(r)) / 2
        if done >= target:
            break
        if until is not None and until():
            hit = True
            break
        if hw.now() - t0 > timeout:
            ctx.log("    drive: timeout")
            break
        t = hw.now() - t0
        v = vmin + (speed - vmin) * min(1.0, t / max(d.get("ramp_s", 0.3), 0.01))
        if until is None:  # dobrzdění jen u jízdy na vzdálenost
            v = min(v, max(vmin, (target - done) * d.get("decel_k", 3.0)))
        corr = 0.0
        if h0 is not None:
            corr = clamp((hw.gyro_angle() - h0) * d.get("heading_kp", 4.0), -v / 2, v / 2)
        hw.drive_speed(sign * v + corr, sign * v - corr)
        hw.sleep(d.get("dt", 0.01))
    hw.stop()
    return hit


def turn_deg(ctx, deg, speed=None):
    """Otočení na místě, kladně = vlevo (CCW). Gyro, jinak enkodéry."""
    hw, d = ctx.hw, ctx.drive
    speed = speed or d["turn_speed"]
    vmin = d.get("turn_min_speed", 60)
    tol = d.get("turn_tol_deg", 1.5)
    h0 = hw.gyro_angle()
    t0 = hw.now()
    timeout = abs(deg) * d["wheel_deg_per_robot_deg"] / float(vmin) + 2
    if h0 is not None:
        target = h0 + deg
        while True:
            err = target - hw.gyro_angle()
            if abs(err) <= tol or hw.now() - t0 > timeout:
                break
            v = clamp(abs(err) * d.get("turn_kp", 6.0), vmin, speed)
            hw.drive_speed(-sgn(err) * v, sgn(err) * v)
            hw.sleep(d.get("dt", 0.01))
    else:
        target = abs(deg) * d["wheel_deg_per_robot_deg"]
        hw.reset_wheels()
        s = sgn(deg)
        while True:
            l, r = hw.wheel_angles()
            done = (abs(l) + abs(r)) / 2
            if done >= target or hw.now() - t0 > timeout:
                break
            v = clamp((target - done) * d.get("decel_k", 3.0), vmin, speed)
            hw.drive_speed(-s * v, s * v)
            hw.sleep(d.get("dt", 0.01))
    hw.stop()


def until_colour(ctx, want=None, avoid=None, debounce=None):
    """Vytvoří predikát: barva je ve `want` / není v `avoid` (n× za sebou)."""
    n = debounce or ctx.ccfg.get("debounce_reads", 3)
    state = {"hits": 0, "last": None}

    def pred():
        c = ctx.hw.colour()
        state["last"] = c
        ok = (want is not None and c in want) or (avoid is not None and c not in avoid)
        state["hits"] = state["hits"] + 1 if ok else 0
        return state["hits"] >= n
    pred.state = state
    return pred


def read_colour(ctx, reads):
    """Několik čtení, vrátí nejčastější barvu mimo pozadí (nebo pozadí)."""
    bg = _colour_set(ctx, "background")
    counts = {}
    for _ in range(reads):
        c = ctx.hw.colour()
        counts[c] = counts.get(c, 0) + 1
        ctx.hw.sleep(0.01)
    fg = [(v, k) for k, v in counts.items() if k not in bg]
    if fg:
        return max(fg)[1]
    return max((v, k) for k, v in counts.items())[1]


# ---------------------------------------------------------------- příkazy

def cmd_drive(ctx, st):
    drive_mm(ctx, st["mm"], st.get("speed"), hold=st.get("hold", True))


def cmd_drive_time(ctx, st):
    v = st.get("speed", ctx.drive["speed"])
    ctx.hw.drive_speed(v, v)
    ctx.hw.sleep(st["s"])
    ctx.hw.stop()


def cmd_turn(ctx, st):
    turn_deg(ctx, st["deg"], st.get("speed"))


def cmd_drive_until_colour(ctx, st):
    want = _colour_set(ctx, st["colour"]) if "colour" in st else None
    avoid = _colour_set(ctx, st["not"]) if "not" in st else None
    pred = until_colour(ctx, want, avoid, st.get("debounce"))
    max_mm = st.get("max_mm", 1000) * (-1 if st.get("reverse") else 1)
    hit = drive_mm(ctx, max_mm, st.get("speed", 200), until=pred)
    ctx.log("    {} (barva {})".format("nalezeno" if hit else "NENALEZENO, max_mm", pred.state["last"]))
    return hit


def cmd_follow_line(ctx, st):
    """Sledování hrany čáry P-regulátorem na odraženém světle."""
    hw, d = ctx.hw, ctx.drive
    speed = st.get("speed", d["speed"])
    kp = st.get("kp", ctx.ccfg.get("line_kp", 2.0))
    target = st.get("target", ctx.ccfg.get("line_target", 50))
    e = 1 if st.get("edge", "left") == "left" else -1
    goal = abs(st["mm"]) * d["deg_per_mm"]
    hw.reset_wheels()
    t0 = hw.now()
    while True:
        l, r = hw.wheel_angles()
        if (abs(l) + abs(r)) / 2 >= goal or hw.now() - t0 > goal / speed * 2 + 2:
            break
        refl = hw.reflected()
        corr = 0 if refl is None else clamp((refl - target) * kp * e, -speed / 2, speed / 2)
        hw.drive_speed(speed - corr, speed + corr)
        hw.sleep(d.get("dt", 0.01))
    hw.stop()


def choose_rot(ctx, seen, front, rear):
    pk, mech = ctx.pick, ctx.mech
    to_front = pk["seen_to_front"]
    if front == rear:
        return mech.rot if mech.rot != "NONE" else to_front
    if seen == front:
        return to_front
    if seen == rear:
        return other_rot(to_front)
    ctx.log("    ! neočekávaná barva {} (čekal {}/{}), fallback {}".format(seen, front, rear, pk["fallback_rot"]))
    return pk["fallback_rot"]


def cmd_pick_row(ctx, st):
    pk = ctx.pick
    layout = ctx.cfg["field"]["layouts"][ctx.side][st["row"]]
    pairs = list(reversed(layout)) if st.get("reverse") else list(layout)
    pairs = pairs[:st.get("count", len(pairs))]

    join = None
    if st.get("append_to"):
        cid, end = st["append_to"].split(".")
        ch = ctx.chains.get(cid)
        if ch is None:
            ctx.log("    ! řetěz {} neexistuje, plánuji bez navázání".format(cid))
        else:
            join = (st.get("join_end", "front"), ch["ends"][end], ch["length"])
    plan = planner.plan_line(pairs, pk["physical_order"], join)
    ctx.log(planner.format_plan(pairs, plan))

    spacing = st.get("spacing_mm") or ctx.cfg["field"]["geometry_mm"].get(st["row"] + "_spacing", 200)
    ctx.hw.mock_blocks(pairs, spacing)

    bg = _colour_set(ctx, "background")
    picked = 0
    for i, (front, rear) in enumerate(plan["orient"]):
        if i > 0 and pk.get("wait_clear", True):
            drive_mm(ctx, pk.get("clear_max_mm", 80), pk["approach_speed"], until=until_colour(ctx, want=bg))
        max_mm = pk["first_max_mm"] if i == 0 else spacing + pk.get("gap_margin_mm", 100)
        if not drive_mm(ctx, max_mm, pk["approach_speed"], until=until_colour(ctx, avoid=bg)):
            ctx.log("    ! kostka {} nenalezena".format(i + 1))
            break
        if pk.get("settle_mm"):
            drive_mm(ctx, pk["settle_mm"], pk["approach_speed"])
        seen = read_colour(ctx, pk.get("reads", 5))
        rot = choose_rot(ctx, seen, front, rear)
        ctx.log("  {} kostka {}: vidím {} -> {} (front={})".format(ctx.stamp(), i + 1, seen, rot, front))
        ctx.mech.goto(rot=rot)
        drive_mm(ctx, pk["advance_mm"], pk.get("advance_speed"))
        picked += 1
        if picked in pk["shift_after"]:
            ctx.mech.goto(slot="toggle")
    ctx.loaded = {"chain_id": st.get("chain_id"), "plan": plan, "count": picked,
                  "append_to": st.get("append_to"), "join_end": st.get("join_end", "front")}


def run_macro(ctx, name):
    run_steps(ctx, ctx.cfg["macros"][name])


def cmd_unload(ctx, st):
    run_macro(ctx, st.get("macro", "unload"))
    ld, ctx.loaded = ctx.loaded, None
    if not ld or not ld["count"]:
        return
    plan = ld["plan"]
    cid = st.get("chain_id") or ld["chain_id"] or "chain{}".format(len(ctx.chains) + 1)
    full = plan["segments"] == [ld["count"]]
    if ld["append_to"] and plan["join_ok"] and full:
        acid, aend = ld["append_to"].split(".")
        ch = ctx.chains[acid]
        ch["length"] += ld["count"]
        far = "rear" if ld["join_end"] == "front" else "front"
        ch["ends"][aend] = plan["ends"][far]
        ctx.log("    řetěz {} prodloužen na {}".format(acid, ch["length"]))
    else:
        ctx.chains[cid] = {"length": plan["segments"][0], "ends": dict(plan["ends"])}
        ctx.log("    řetěz {} položen: {}".format(cid, ctx.chains[cid]))


def cmd_arm(ctx, st):
    ctx.mech.goto(rot=st.get("rot"), slot=st.get("slot"))


def cmd_arm_home(ctx, st):
    ctx.mech.home()


def cmd_macro(ctx, st):
    run_macro(ctx, st["name"])


def cmd_wait(ctx, st):
    ctx.hw.sleep(st["s"])


def cmd_beep(ctx, st):
    ms = st.get("ms", 150)
    ctx.hw.beep(st.get("hz", 880), ms)
    if st.get("wait", False):
        ctx.hw.sleep(ms / 1000.0)


def cmd_log(ctx, st):
    ctx.log("  " + st["msg"])


def cmd_print_colour(ctx, st):
    t0 = ctx.hw.now()
    while ctx.hw.now() - t0 < st.get("seconds", 10):
        ctx.log("    colour={} reflected={}".format(ctx.hw.colour(), ctx.hw.reflected() if st.get("reflected") else "-"))
        ctx.hw.sleep(st.get("period", 0.3))


# cmd -> (funkce, povinné parametry)
ACTIONS = {
    "drive": (cmd_drive, ("mm",)),
    "drive_time": (cmd_drive_time, ("s",)),
    "turn": (cmd_turn, ("deg",)),
    "drive_until_colour": (cmd_drive_until_colour, ()),
    "follow_line": (cmd_follow_line, ("mm",)),
    "pick_row": (cmd_pick_row, ("row",)),
    "unload": (cmd_unload, ()),
    "arm": (cmd_arm, ()),
    "arm_home": (cmd_arm_home, ()),
    "macro": (cmd_macro, ("name",)),
    "wait": (cmd_wait, ("s",)),
    "beep": (cmd_beep, ()),
    "log": (cmd_log, ("msg",)),
    "print_colour": (cmd_print_colour, ()),
}


def active(st):
    return "cmd" in st and st.get("enabled", True)


def run_steps(ctx, steps):
    for st in steps:
        if not active(st):
            continue
        ctx.log("{} {}".format(ctx.stamp(), {k: v for k, v in st.items() if not k.startswith("_")}))
        ACTIONS[st["cmd"]][0](ctx, st)
