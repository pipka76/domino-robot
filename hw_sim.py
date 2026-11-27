"""Most do 3D simulace v Godotu (složka simulation/): náš kód (actions, mechanism, mission.json)
řídí robota v simulátoru, simulátor dělá fyziku a zobrazení.

Spuštění (z kořene repa):
  "/Applications/Godot_mono.app/Contents/MacOS/Godot" --path simulation -- --ours main
Godot si Python (`python3 main.py --sim main`) spustí sám; ručně jde i takto:
  1) Godot s `-- --ours main --no-python`,  2) `python3 main.py --sim main`.

Protokol: TCP 127.0.0.1:5577, řádky „příkaz args“ -> „ok hodnoty“. Čas běží jen během `sleep`
(lock-step), takže výpočty v Pythonu simulační čas nespotřebují.
Na robot se nenahrává.
"""
import socket
import sys

from common import TimeUp

PORT = 5577
ARM_MAX_DEG_S = 1050  # motor C reálně ~1000–1100 deg/s


class _Tee:
    """Kopíruje výpis i do simulátoru (log v konzoli Godotu a v HUD)."""

    def __init__(self, hw, out):
        self.hw, self.out, self.buf = hw, out, ""

    def write(self, s):
        self.out.write(s)
        self.buf += s
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            if line.strip():
                self.hw.send_log(line)

    def flush(self):
        self.out.flush()


class SimHardware:
    def __init__(self, cfg, log=print, host="127.0.0.1", port=PORT):
        self.log = log
        self.deadline = None
        self.arm_total = 0.0
        self.sock = socket.create_connection((host, port), timeout=30)
        self.sock.settimeout(60)
        self.f = self.sock.makefile("rw", encoding="utf-8", newline="\n")
        self._w0 = (0.0, 0.0)
        sys.stdout = _Tee(self, sys.stdout)

    # --- spojení ---
    def _req(self, line):
        self.f.write(line + "\n")
        self.f.flush()
        r = self.f.readline()
        if not r:
            raise RuntimeError("simulátor ukončil spojení")
        parts = r.split()
        if not parts or parts[0] != "ok":
            raise RuntimeError("simulátor: " + r.strip())
        return parts[1:]

    def send_log(self, text):
        try:
            self._req("log " + text.replace("\n", " "))
        except Exception:
            pass

    def close(self):
        try:
            self._req("done")
        except Exception:
            pass

    # --- čas ---
    def now(self):
        return float(self._req("time")[0])

    def set_deadline(self, t):
        self.deadline = t

    def check_time(self):
        if self.deadline is not None and self.now() > self.deadline:
            raise TimeUp()

    def sleep(self, s):
        self.check_time()
        if s > 0:
            self._req("sleep {:.4f}".format(s))

    # --- pohon ---
    def drive_speed(self, l, r):
        self._req("drive {:.1f} {:.1f}".format(l, r))

    def stop(self):
        self.drive_speed(0, 0)

    def stop_all(self):
        self.stop()

    def _enc(self):
        a = self._req("enc")
        return float(a[0]), float(a[1])

    def reset_wheels(self):
        self._w0 = self._enc()

    def wheel_angles(self):
        l, r = self._enc()
        return l - self._w0[0], r - self._w0[1]

    def gyro_angle(self):
        return int(round(float(self._req("gyro")[0])))  # jako reálné gyro: celé stupně, CCW+

    # --- senzor ---
    def colour(self):
        return self._req("colour")[0]

    def reflected(self):
        return int(self._req("reflected")[0])

    # --- motor C ---
    def arm_target(self, rot, slot):
        """Volá mechanism.py před každým pohybem motoru C – simulátor podle toho ovládá ruce/zásobník."""
        self._req("arm {} {}".format(rot, slot))

    def arm_move(self, deg, speed, tol=15, stall_s=None, extra_wait_s=None):
        self.arm_total += abs(deg)
        t = abs(deg) / float(min(abs(speed), ARM_MAX_DEG_S)) + 0.05
        self._req("busy {:.3f}".format(t))  # simulátor ukáže odpočet u robota
        self.sleep(t)
        return deg

    # --- ostatní ---
    def beep(self, hz=880, ms=150):
        self.log("    [sim] beep {} Hz {} ms".format(hz, ms))

    def mock_blocks(self, pairs, spacing_mm):
        pass
