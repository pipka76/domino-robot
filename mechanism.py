"""Stavový automat motoru C: rotace kostky (NONE/CW/CCW) × slot zásobníku (LEFT/RIGHT).

Nahrazuje příznaky IsCW/IsCCW/IsLeft/IsRight ze sample.py. Pohyby jsou relativní
(motor se před každým pohybem nuluje, stejně jako v sample.py) a berou se z JSON:
- robot.arm.transitions: "ROT,SLOT>ROT,SLOT": stupně – explicitní přechody
  (hlavně posuny zásobníku, protože mechanismus není lineární),
- robot.arm.rot_positions: poloha rotace v rámci jednoho slotu; přechod mezi
  rotacemi bez explicitního záznamu = rozdíl poloh.
"""
from common import ROTS, SLOTS


def _parse_key(key):
    a, b = key.split(">")
    r1, s1 = [x.strip().upper() for x in a.split(",")]
    r2, s2 = [x.strip().upper() for x in b.split(",")]
    return (r1, s1, r2, s2)


class Mechanism:
    def __init__(self, hw, arm_cfg, log=print):
        self.hw = hw
        self.cfg = arm_cfg
        self.log = log
        init = arm_cfg["initial"]
        self.rot = init["rot"].upper()
        self.slot = init["slot"].upper()
        self.trans = {}
        for k, v in arm_cfg["transitions"].items():
            if k.startswith("_"):
                continue
            self.trans[_parse_key(k)] = v
        self.rot_pos = arm_cfg["rot_positions"]

    def state(self):
        return (self.rot, self.slot)

    def plan(self, rot=None, slot=None):
        """Vrátí seznam pohybů [(stupně, rychlost, (rot, slot))] bez provedení."""
        r, s = self.rot, self.slot
        if slot is not None:
            slot = slot.upper()
            if slot == "TOGGLE":
                slot = "LEFT" if s == "RIGHT" else "RIGHT"
        target_slot = slot or s
        target_rot = rot.upper() if rot else None
        moves = []

        if target_slot != s:
            exact = (r, s, target_rot, target_slot)
            if target_rot and exact in self.trans:
                return [(self._shift_deg(self.trans[exact], target_slot), self.cfg["shift_speed"], (target_rot, target_slot))]
            cands = [k for k in self.trans if k[0] == r and k[1] == s and k[3] == target_slot]
            if not cands:
                raise ValueError("chybí přechod zásobníku z {},{} do slotu {}".format(r, s, target_slot))
            k = cands[0]
            moves.append((self._shift_deg(self.trans[k], target_slot), self.cfg["shift_speed"], (k[2], k[3])))
            r, s = k[2], k[3]

        if target_rot and target_rot != r:
            k = (r, s, target_rot, s)
            deg = self.trans[k] if k in self.trans else self.rot_pos[target_rot] - self.rot_pos[r]
            moves.append((deg, self.cfg["speed"], (target_rot, s)))
        return moves

    def _shift_deg(self, deg, target_slot):
        """Přičte k posunu zásobníku korekci arm.shift_extra[slot] (vůle mechaniky), ve směru pohybu."""
        extra = self.cfg.get("shift_extra", {}).get(target_slot, 0)
        return deg + (extra if deg >= 0 else -extra)

    def goto(self, rot=None, slot=None, extra=0):
        """extra: stupně navíc k posunu zásobníku (ve směru pohybu), např. jen pro jednu řadu."""
        for deg, speed, (r, s) in self.plan(rot, slot):
            if extra and s != self.slot:
                deg += extra if deg >= 0 else -extra
            self.log("    arm {:+} deg  {},{} -> {},{}".format(deg, self.rot, self.slot, r, s))
            if hasattr(self.hw, "arm_target"):  # jen 3D simulace (hw_sim.py)
                self.hw.arm_target(r, s)
            self.hw.arm_move(deg, speed)
            self.rot, self.slot = r, s

    def nudge(self, deg=None):
        """Dorovná zásobník, kdyby se vlivem vůle posunul. Motor C uvnitř slotu jen otáčí kostkou
        (rot_positions), zásobník tlačí až za krajní polohou rotace: ve slotu RIGHT za CCW (+),
        v LEFT za CW (-). Proto dojede do krajní rotace a pak ještě o nudge_deg dál.
        Pak motor vrátí o to, co skutečně ujel za krajní polohu (nudge_return), aby rotace
        zase odpovídala krajní poloze – jinak se další pohyby (otočení kostky, posun zásobníku)
        posunou o ty stupně a zásobník pak nedojede."""
        extra = abs(deg if deg is not None else self.cfg.get("nudge_deg", 800))
        end_rot, sign = ("CCW", 1) if self.slot == "RIGHT" else ("CW", -1)
        move = self.rot_pos[end_rot] - self.rot_pos[self.rot] + sign * extra
        if not move:
            return
        self.log("    arm dorovnání zásobníku {:+} deg  {},{} -> {},{} (+{} za krajní polohu)".format(
            move, self.rot, self.slot, end_rot, self.slot, extra))
        # Pevný počet stupňů, bez hlídání zablokování (na dorazu nefungovalo); krátký timeout,
        # aby motor zbytečně netlačil do dorazu.
        speed = self.cfg.get("nudge_speed", self.cfg["shift_speed"])
        if hasattr(self.hw, "arm_target"):  # jen 3D simulace (hw_sim.py)
            self.hw.arm_target(end_rot, self.slot)
        moved = self.hw.arm_move(move, speed, stall_s=0, extra_wait_s=0.3)
        self.rot = end_rot
        if moved is None or not self.cfg.get("nudge_return", True):
            return
        over = moved - (move - sign * extra)  # kolik ujel za krajní polohu (se znaménkem)
        if over * sign > 0:
            self.log("    arm návrat po dorovnání {:+} deg".format(-over))
            self.hw.arm_move(-over, speed)

    def home(self):
        init = self.cfg["initial"]
        self.goto(rot=init["rot"], slot=init["slot"])

    def check(self):
        """Ověří, že posun zásobníku je definovaný z každého stavu."""
        errs = []
        for k in self.trans:
            for x, allowed in ((k[0], ROTS), (k[1], SLOTS), (k[2], ROTS), (k[3], SLOTS)):
                if x not in allowed:
                    errs.append("arm.transitions: neznámá hodnota {} v {}".format(x, k))
        for r in ROTS:
            if r not in self.rot_pos:
                errs.append("arm.rot_positions: chybí {}".format(r))
            for s in SLOTS:
                other = "LEFT" if s == "RIGHT" else "RIGHT"
                if not any(k[0] == r and k[1] == s and k[3] == other for k in self.trans):
                    errs.append("arm.transitions: chybí posun zásobníku z {},{}".format(r, s))
        return errs
