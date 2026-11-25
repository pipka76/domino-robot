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
        self.prepared = None  # plán z pick_prepare (1. kostku nabere zatáčka)
        # drive.absolute_heading: cílový směr (hodnota gyra) se počítá od startu mise, ne od
        # toho, kam robot zrovna míří -> chyby otáček se nesčítají. None = ještě nezměřeno.
        self.heading = None
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


def heading_ref(ctx):
    """Směr, od kterého se počítá další jízda/otočka. Při drive.absolute_heading je to cílový
    směr z předchozích otáček (od startu mise), jinak aktuální hodnota gyra."""
    g = ctx.hw.gyro_angle()
    if g is None or not ctx.drive.get("absolute_heading", False):
        return g
    if ctx.heading is None:
        ctx.heading = g
    return ctx.heading


def set_heading(ctx, target):
    if ctx.drive.get("absolute_heading", False):
        ctx.heading = target


def drive_mm(ctx, mm, speed=None, until=None, hold=True, overrun_mm=0, tail_s=0, tail_speed=None):
    """Jízda rovně o mm (enkodéry) s rampou, dobrzděním a držením směru gyrem.
    until: funkce bez parametrů -> True = zastavit. Vrací True, když zastavil `until`.
    overrun_mm: po splnění `until` jet ještě tolik mm v jednom kuse (bez zastavení navíc).
    tail_s, tail_speed: po ujetí mm nezastavit, zpomalit na tail_speed a jet ještě tail_s s
    (bez držení směru) – např. dotlačení ke stěně v jedné jízdě."""
    hw, d = ctx.hw, ctx.drive
    speed = speed or d["speed"]
    vmin = d.get("min_speed", 100)
    sign = 1 if mm >= 0 else -1
    target = abs(mm) * d["deg_per_mm"]
    hw.reset_wheels()
    h0 = heading_ref(ctx) if hold else None
    t0 = hw.now()
    timeout = target / float(max(speed, 1)) * 2 + 2
    hit = False
    integ = 0.0
    while True:
        l, r = hw.wheel_angles()
        done = (abs(l) + abs(r)) / 2
        if done >= target:
            break
        if until is not None and until():
            hit = True
            if overrun_mm > 0:
                target = done + overrun_mm * d["deg_per_mm"]
                until = None  # dál už na vzdálenost, s dobrzděním
                continue
            break
        if hw.now() - t0 > timeout:
            ctx.log("    drive: timeout")
            break
        t = hw.now() - t0
        v = vmin + (speed - vmin) * min(1.0, t / max(d.get("ramp_s", 0.3), 0.01))
        if until is None:  # dobrzdění jen u jízdy na vzdálenost (s tail jen na tail_speed)
            v_end = abs(tail_speed) if tail_s and tail_speed else vmin
            v = min(v, max(v_end, (target - done) * d.get("decel_k", 3.0)))
        corr = 0.0
        if h0 is not None:
            herr = hw.gyro_angle() - h0
            integ = clamp(integ + herr * d.get("dt", 0.01), -50, 50)
            corr = clamp(herr * d.get("heading_kp", 4.0) + integ * d.get("heading_ki", 0.0), -v / 2, v / 2)
        hw.drive_speed(sign * v + corr, sign * v - corr)
        hw.sleep(d.get("dt", 0.01))
    if tail_s and not hit and done >= target:
        v = abs(tail_speed or vmin)
        hw.drive_speed(sign * v, sign * v)
        hw.sleep(tail_s)
    hw.stop()
    l, r = hw.wheel_angles()
    wheels = "kola L={:+.0f} P={:+.0f} deg".format(l, r)
    if h0 is not None or hw.gyro_angle() is not None:
        h1 = hw.gyro_angle()
        if h0 is None:
            ctx.log("    drive {:+.0f} mm bez gyra, gyro teď {:+.1f}, {}".format(mm, h1, wheels))
        else:
            ctx.log("    drive {:+.0f} mm, odchylka podle gyra {:+.1f} deg (kladně = doleva), {}".format(mm, h1 - h0, wheels))
    else:
        ctx.log("    drive {:+.0f} mm, {}".format(mm, wheels))
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
        target = heading_ref(ctx) + deg
        set_heading(ctx, target)
        prev = None
        timed_out = False
        while True:
            err = target - hw.gyro_angle()
            if abs(err) <= tol:
                break
            # přejel cíl jen o kousek -> nevracet se (jinak kmitá kolem cíle)
            if prev is not None and sgn(err) != sgn(prev) and abs(err) <= max(tol, 3):
                break
            if hw.now() - t0 > timeout:
                timed_out = True
                break
            prev = err
            v = clamp(abs(err) * d.get("turn_kp", 6.0), vmin, speed)
            hw.drive_speed(-sgn(err) * v, sgn(err) * v)
            hw.sleep(d.get("dt", 0.01))
        hw.stop()
        ctx.log("    turn {:+.0f}: gyro {:+.0f} -> {:+.0f} (chyba {:+.0f}), {:.1f} s{}".format(
            deg, h0, hw.gyro_angle(), target - hw.gyro_angle(), hw.now() - t0,
            "  ! TIMEOUT" if timed_out else ""))
        return
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


def curve_deg(ctx, deg, inner=None, outer=None, s_per_90=None, use_gyro=None):
    """Zatáčka po oblouku (ne na místě): vnitřní kolo pomalu, vnější rychle.
    Kladně = doleva (CCW). S gyrem zastaví na úhlu, jinak podle času jako turn() v sample.py."""
    hw, d = ctx.hw, ctx.drive
    inner = d.get("curve_inner_speed", 100) if inner is None else inner
    outer = d.get("curve_outer_speed", 500) if outer is None else outer
    s_per_90 = d.get("curve_s_per_90", 1.34) if s_per_90 is None else s_per_90
    use_gyro = d.get("curve_gyro", True) if use_gyro is None else use_gyro
    s = sgn(deg)
    l, r = (inner, outer) if s > 0 else (outer, inner)
    t_est = s_per_90 * abs(deg) / 90.0
    h0 = hw.gyro_angle() if use_gyro else None
    t0 = hw.now()
    hw.drive_speed(l, r)
    if h0 is None:
        hw.sleep(t_est)
        hw.stop()
        if ctx.heading is not None:
            ctx.heading += deg
        ctx.log("    curve {:+.0f}: podle času {:.2f} s".format(deg, t_est))
        return
    target = heading_ref(ctx) + deg
    set_heading(ctx, target)
    timed_out = False
    while sgn(target - hw.gyro_angle()) == s:
        if hw.now() - t0 > t_est * 1.5 + 1:
            timed_out = True
            break
        hw.sleep(d.get("dt", 0.01))
    hw.stop()
    ctx.log("    curve {:+.0f}: gyro {:+.0f} -> {:+.0f}, {:.2f} s{}".format(
        deg, h0, hw.gyro_angle(), hw.now() - t0, "  ! TIMEOUT" if timed_out else ""))


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


def until_block(ctx, present=True, debounce=None):
    """Predikát „pod senzorem je kostka“ (present=True) / „už není“ (False).
    colour.floor_reflected = [min, max]: rozhoduje odražené světlo – podlaha dává hodnoty
    v tomhle rozsahu, kostka mimo něj (zelená/modrá tmavší, červená/žlutá světlejší).
    Barvu podlahy senzor plete (bílou čte jako GREEN). Bez nastavení = podle barvy."""
    floor = ctx.ccfg.get("floor_reflected")
    if not floor:
        bg = _colour_set(ctx, "background")
        return until_colour(ctx, avoid=bg, debounce=debounce) if present else until_colour(ctx, want=bg, debounce=debounce)
    n = debounce or ctx.ccfg.get("debounce_reads", 3)
    state = {"hits": 0, "last": None}

    def pred():
        r = ctx.hw.reflected()
        state["last"] = r
        is_block = r < floor[0] or r > floor[1]
        ok = is_block == present
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
    drive_mm(ctx, st["mm"], st.get("speed"), hold=st.get("hold", True),
             tail_s=st.get("tail_s", 0), tail_speed=st.get("tail_speed"))


def cmd_drive_time(ctx, st):
    """Jízda daný čas bez držení směru. Se "square": true slouží k dorovnání o stěnu:
    robot se dotlačí ke stěně, počká settle_s a aktuální směr gyra vezme jako přesný
    (při drive.absolute_heading se od něj počítají další otáčky)."""
    v = st.get("speed", ctx.drive["speed"])
    ctx.hw.drive_speed(v, v)
    ctx.hw.sleep(st["s"])
    ctx.hw.stop()
    if st.get("square"):
        ctx.hw.sleep(st.get("settle_s", 0.2))
        g = ctx.hw.gyro_angle()
        if g is not None:
            old = ctx.heading
            set_heading(ctx, g)
            ctx.log("    dorovnáno o stěnu: gyro {:+.1f}{}".format(
                g, "" if old is None else ", cílový směr byl {:+.1f} (oprava {:+.1f})".format(old, g - old)))


def cmd_turn(ctx, st):
    turn_deg(ctx, st["deg"], st.get("speed"))


def cmd_curve(ctx, st):
    curve_deg(ctx, st["deg"], st.get("inner"), st.get("outer"), st.get("s_per_90"), st.get("gyro"))


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
    if isinstance(to_front, dict):  # zvlášť pro každý slot zásobníku
        to_front = to_front[mech.slot]
    if front == rear:
        return mech.rot if mech.rot != "NONE" else to_front
    if seen == front:
        return to_front
    if seen == rear:
        return other_rot(to_front)
    ctx.log("    ! neočekávaná barva {} (čekal {}/{}), fallback {}".format(seen, front, rear, pk["fallback_rot"]))
    return pk["fallback_rot"]


def _pick_plan(ctx, st):
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
    plan = planner.plan_line(pairs, ctx.pick["physical_order"], join)
    ctx.log(planner.format_plan(pairs, plan))
    return pairs, plan


def advance_len(ctx, picked_after, base):
    """Délka zasunutí kostky; před posunem zásobníku prodloužená o pick.shift_advance_mm,
    pro konkrétní kostku (pořadí od 1) navíc o pick.advance_extra_mm["N"]."""
    mm = base + ctx.pick.get("advance_extra_mm", {}).get(str(picked_after), 0)
    if picked_after in ctx.pick["shift_after"]:
        mm += ctx.pick.get("shift_advance_mm", 0)
    return mm


def shift_magazine(ctx, extra=0, wait=None):
    """Posun zásobníku během nabírání (kostku před tím dotlačí delší advance, viz advance_len).
    extra = stupně navíc k posunu (pick_row "shift_extra", např. jen pro řadu B)."""
    pk = ctx.pick
    wait = pk.get("shift_wait_s", 0) if wait is None else wait
    if wait:
        ctx.hw.sleep(wait)
    ctx.mech.goto(slot="toggle", extra=extra)


def cmd_pick_prepare(ctx, st):
    """Naplánuje řadu a PŘEDEM natočí rameno pro 1. kostku, kterou nabere zatáčka
    (barvu nejde přečíst předem, bere se z layouts). Pak následuje pick_row s first_taken."""
    pairs, plan = _pick_plan(ctx, st)
    front, rear = plan["orient"][0]
    rot = st.get("rot")
    if rot:
        ctx.log("    příprava: kostka 1 -> {} (zadáno ručně)".format(rot))
    else:
        half = st.get("seen", ctx.pick.get("seen_half", 0))
        seen = pairs[0][half]
        rot = choose_rot(ctx, seen, front, rear)
        ctx.log("    příprava: kostka 1 {} – senzor by viděl {} (seen={}) -> {} (front={})".format(
            pairs[0], seen, half, rot, front))
    ctx.mech.goto(rot=rot)
    ctx.prepared = {"row": st["row"], "pairs": pairs, "plan": plan, "st": dict(st)}


def cmd_pick_row(ctx, st):
    pk = ctx.pick
    prep = None
    if st.get("first_taken"):
        prep = getattr(ctx, "prepared", None)
        ctx.prepared = None
        if prep is None or prep["row"] != st["row"]:
            ctx.log("    ! first_taken bez pick_prepare pro řadu {} – nabírám normálně".format(st["row"]))
            prep = None
    if prep is not None:
        args = dict(prep["st"])
        args.update(st)
        st = args
        pairs, plan = prep["pairs"], prep["plan"]
    else:
        pairs, plan = _pick_plan(ctx, st)

    spacing = st.get("spacing_mm") or ctx.cfg["field"]["geometry_mm"].get(st["row"] + "_spacing", 200)
    ctx.hw.mock_blocks(pairs[1:] if prep is not None else pairs, spacing)

    bg = _colour_set(ctx, "background")
    # rychlosti lze přepsat v kroku pick_row (např. jen pro jednu řadu)
    approach_v = st.get("approach_speed", pk["approach_speed"])
    advance_v = st.get("advance_speed", pk.get("advance_speed"))
    # pick.shift_at_next: zásobník se posune až u další kostky (robot tam stejně zastaví),
    # takže kostka před posunem má víc času dojet a nepřibude další brzdění
    shift_next = st.get("shift_at_next", pk.get("shift_at_next", False))
    pending = False
    shift_extra = st.get("shift_extra", 0)
    shift_wait = st.get("shift_wait_s")  # None = pick.shift_wait_s
    picked = 0
    for i, (front, rear) in enumerate(plan["orient"]):
        if i == 0 and prep is not None:
            ctx.log("  {} kostka 1: nabraná zatáčkou, rameno připraveno {} (front={}), senzor teď vidí {}".format(
                ctx.stamp(), ctx.mech.rot, front, ctx.hw.colour()))
            drive_mm(ctx, advance_len(ctx, picked + 1, st.get("first_advance_mm", pk["advance_mm"])), advance_v)
            picked += 1
            if picked in pk["shift_after"]:
                if shift_next:
                    pending = True
                else:
                    shift_magazine(ctx, shift_extra, shift_wait)
            continue
        if i > 0 and pk.get("wait_clear", True):
            drive_mm(ctx, pk.get("clear_max_mm", 80), approach_v, until=until_block(ctx, present=False))
        max_mm = pk["first_max_mm"] if i == 0 else spacing + pk.get("gap_margin_mm", 100)
        # před posunem zásobníku (shift_at_next) dojet kousek za kostku, ať předchozí kostka
        # dojede na konec zásobníku – v jednom kuse s hledáním, bez zastavení navíc
        over = st.get("shift_overrun_mm", 0) if pending else 0
        if not drive_mm(ctx, max_mm, approach_v, until=until_block(ctx, present=True), overrun_mm=over):
            ctx.log("    ! kostka {} nenalezena".format(i + 1))
            break
        if pk.get("settle_mm"):
            drive_mm(ctx, pk["settle_mm"], approach_v)
        if pending:
            ctx.log("    posun zásobníku až u kostky {}".format(i + 1))
            shift_magazine(ctx, shift_extra, shift_wait)
            pending = False
        if st.get("ignore_colours"):
            # barvy se nečtou, ruce zůstávají, jak jsou (méně pohybů motoru C)
            ctx.log("  {} kostka {}: barvy ignoruji, ruce zůstávají {}".format(ctx.stamp(), i + 1, ctx.mech.rot))
        else:
            seen = read_colour(ctx, pk.get("reads", 5))
            rot = choose_rot(ctx, seen, front, rear)
            ctx.log("  {} kostka {}: vidím {} -> {} (front={})".format(ctx.stamp(), i + 1, seen, rot, front))
            ctx.mech.goto(rot=rot)
        drive_mm(ctx, advance_len(ctx, picked + 1, pk["advance_mm"]), advance_v)
        picked += 1
        if picked in pk["shift_after"]:
            if shift_next:
                pending = True
            else:
                shift_magazine(ctx, shift_extra, shift_wait)
    if pending:
        shift_magazine(ctx, shift_extra, shift_wait)  # další kostka nebyla, posunout aspoň na konci
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
    if st.get("rot") or st.get("slot"):
        ctx.mech.goto(rot=st.get("rot"), slot=st.get("slot"))
    nudge = st.get("nudge")
    if nudge:
        ctx.mech.nudge(None if nudge is True else nudge)


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


def cmd_print_gyro(ctx, st):
    """Vypisuje úhel gyra a kol – na zjištění driftu (robot stojí) a kontrolu proti skutečnosti."""
    hw = ctx.hw
    t0 = hw.now()
    g0 = hw.gyro_angle()
    while True:
        g = hw.gyro_angle()
        l, r = hw.wheel_angles()
        ctx.log("    gyro={} (od začátku výpisu {:+}) kola L={:+.0f} P={:+.0f}".format(
            g, None if g is None or g0 is None else g - g0, l, r))
        if hw.now() - t0 >= st.get("seconds", 5):
            break
        hw.sleep(st.get("period", 1.0))


# cmd -> (funkce, povinné parametry)
ACTIONS = {
    "drive": (cmd_drive, ("mm",)),
    "drive_time": (cmd_drive_time, ("s",)),
    "turn": (cmd_turn, ("deg",)),
    "drive_until_colour": (cmd_drive_until_colour, ()),
    "follow_line": (cmd_follow_line, ("mm",)),
    "pick_row": (cmd_pick_row, ("row",)),
    "pick_prepare": (cmd_pick_prepare, ("row",)),
    "curve": (cmd_curve, ("deg",)),
    "unload": (cmd_unload, ()),
    "arm": (cmd_arm, ()),
    "arm_home": (cmd_arm_home, ()),
    "macro": (cmd_macro, ("name",)),
    "wait": (cmd_wait, ("s",)),
    "beep": (cmd_beep, ()),
    "log": (cmd_log, ("msg",)),
    "print_colour": (cmd_print_colour, ()),
    "print_gyro": (cmd_print_gyro, ()),
}


def active(st):
    return "cmd" in st and st.get("enabled", True)


def run_steps(ctx, steps):
    for st in steps:
        if not active(st):
            continue
        ctx.log("{} {}".format(ctx.stamp(), {k: v for k, v in st.items() if not k.startswith("_")}))
        ACTIONS[st["cmd"]][0](ctx, st)
