"""Hardware vrstva pro Brian – JEDINÉ místo, které importuje brian.*.

API podle dokumentace:
  https://brian.fel.cvut.cz/cs/motors.html, https://brian.fel.cvut.cz/cs/brian.motors.html
  https://brian.fel.cvut.cz/cs/EV3_sensors.html, https://brian.fel.cvut.cz/cs/brian.sensors.EV3.html

Rozhraní (stejné má hw_mock.MockHardware):
    now(), sleep(s), set_deadline(t)
    drive_speed(l, r), stop(), stop_all()
    reset_wheels(), wheel_angles() -> (l, r)
    gyro_angle() -> stupně, kladně = proti směru hodin (CCW) | None
    colour() -> "RED"/"WHITE"/"NONE"/..., reflected() -> 0..100 | None
    arm_move(deg, speed)  (relativní, blokující)
    beep(hz, ms), mock_blocks(...) (no-op)
"""
import time

import brian.audio as audio
import brian.motors as motors
import brian.sensors as sensors

from common import COLOURS, TimeUp, now as _now


class Hardware:
    def __init__(self, cfg, log=print):
        self.log = log
        rc = cfg["robot"]
        ports = rc["ports"]
        self.drive_cfg = rc["drive"]
        self.ldir = self.drive_cfg.get("left_dir", 1)
        self.rdir = self.drive_cfg.get("right_dir", 1)
        self.gyro_sign = self.drive_cfg.get("gyro_sign", -1)
        self.arm_cfg = rc["arm"]
        self.deadline = None

        self.left = motors.EV3LargeMotor(getattr(motors.MotorPort, ports["left"]))
        self.right = motors.EV3LargeMotor(getattr(motors.MotorPort, ports["right"]))
        self.arm = motors.EV3MediumMotor(getattr(motors.MotorPort, ports["arm"]))
        self.col = sensors.EV3.ColorSensorEV3(getattr(sensors.SensorPort, ports["colour"]))
        self.gyro = None
        if ports.get("gyro") and self.drive_cfg.get("use_gyro", True):
            self.gyro = sensors.EV3.GyroSensorEV3(getattr(sensors.SensorPort, ports["gyro"]))

        for m in (self.left, self.right, self.arm):
            if not m.wait_until_ready(5000):
                self.log("MOTOR na portu nepřipraven: {}".format(m))

        # Color enum -> naše jména (NO_COLOR -> NONE)
        C = sensors.EV3.ColorSensorEV3.Color
        self._colour_names = {getattr(C, "NO_COLOR"): "NONE"}
        for name in COLOURS:
            v = getattr(C, name, None)
            if v is not None:
                self._colour_names[v] = name
        self.col.set_mode(self.col.Mode.COLOR_DETECT)
        self.col.wait_until_ready(3000)

        if self.gyro is not None:
            if self.drive_cfg.get("gyro_reboot_on_start", True):
                # reboot = rekalibrace gyra, robot musí stát v klidu.
                # Pauza, aby se robot po stisku START přestal hýbat.
                time.sleep(self.drive_cfg.get("gyro_calib_delay_s", 1.0))
                self.gyro.reboot()
                time.sleep(0.2)
            self.gyro.set_mode(self.gyro.Mode.ANGLE)
            if self.gyro.wait_until_ready(5000):
                self.gyro.set_zero_point()
                self._check_gyro_drift()
            else:
                self.log("GYRO nepřipraveno, točím podle enkodérů")
                self.gyro = None

    def _check_gyro_drift(self):
        """Krátce změří, jestli gyro v klidu neujíždí (špatná kalibrace). Jen varuje."""
        s = self.drive_cfg.get("gyro_check_s", 1.0)
        if not s:
            return
        a0 = self.gyro.angle()
        time.sleep(s)
        d = self.gyro.angle() - a0
        if abs(d) >= 1:
            self.log("! GYRO UJÍŽDÍ: {:+} deg za {} s v klidu – spusť kalibrace_gyra.py".format(d, s))
            for _ in range(3):
                audio.play_tone(300, 150)
                time.sleep(0.25)
        else:
            self.log("gyro OK (drift {:+} deg za {} s)".format(d, s))
        self.gyro.set_zero_point()

    # --- čas ---
    def now(self):
        return _now()

    def set_deadline(self, t):
        self.deadline = t

    def check_time(self):
        if self.deadline is not None and _now() > self.deadline:
            raise TimeUp()

    def sleep(self, s):
        self.check_time()
        time.sleep(s)

    # --- pojezd ---
    def drive_speed(self, l, r):
        self.left.run_at_speed(int(l * self.ldir))
        self.right.run_at_speed(int(r * self.rdir))

    def stop(self):
        self.left.brake()
        self.right.brake()

    def stop_all(self):
        self.stop()
        self.arm.brake()

    def reset_wheels(self):
        self.left.reset_angle()
        self.right.reset_angle()

    def wheel_angles(self):
        return (self.left.current_angle() * self.ldir, self.right.current_angle() * self.rdir)

    def gyro_angle(self):
        if self.gyro is None:
            return None
        # Brian: kladně = po směru hodin (při pohledu na šipky) -> gyro_sign=-1 dá CCW+
        return self.gyro.angle() * self.gyro_sign

    # --- senzory (metody samy přepínají mód senzoru) ---
    def colour(self):
        c = self.col.detected_color()
        return self._colour_names.get(c, "NONE")

    def reflected(self):
        return self.col.reflected_value()

    # --- motor C ---
    def _arm_start(self, deg, speed):
        """Spustí relativní pohyb motoru C. Různé verze firmwaru Brian mají různá jména:
        novější rotate_by_angle / rotate_to_angle, starší rotate_to (jako v sample.py).
        Na dokončení se čeká v arm_move podle enkodéru, takže nezáleží, jestli volání blokuje."""
        a = self.arm
        if hasattr(a, "rotate_by_angle"):
            a.rotate_by_angle(int(deg), int(speed), 0)
            return
        target = int(a.current_angle() + deg)
        if hasattr(a, "rotate_to_angle"):
            a.rotate_to_angle(target, int(speed), 0)
        else:
            a.rotate_to(target, int(speed), 0)

    def arm_move(self, deg, speed, tol=15):
        """Relativní pohyb motoru C, čeká na dokončení (s timeoutem).
        Zablokování = enkodér se za arm.stall_s neposunul aspoň o 10 deg.
        (is_stalled() z Brian API hlásí zablokování i při normálním těžkém posunu zásobníku.)"""
        self.check_time()
        est_s = abs(deg) / max(abs(speed), 1)
        t0 = _now()
        t_end = t0 + est_s * 1.5 + self.arm_cfg.get("extra_wait_s", 0.3)
        stall_s = self.arm_cfg.get("stall_s", 0.5)
        start = self.arm.current_angle()
        target = start + deg
        self._arm_start(deg, speed)
        last_a, last_t = start, t0
        while abs(target - self.arm.current_angle()) > tol:
            now = _now()
            a = self.arm.current_angle()
            if abs(a - last_a) >= 10:
                last_a, last_t = a, now
            stalled = now - last_t > stall_s
            if stalled or now > t_end:
                self.arm.brake()  # při timeoutu se motor sám nezastaví
                self.log("    arm: {} (cíl {:+} deg, ujel {:+} deg za {:.1f} s)".format(
                    "ZABLOKOVÁNO" if stalled else "TIMEOUT", deg, int(a - start), now - t0))
                return
            self.check_time()
            time.sleep(0.01)
        dt = _now() - t0
        self.log("    arm ok: {:+} deg za {:.2f} s (průměr {:.0f} deg/s, nastaveno {})".format(
            deg, dt, abs(deg) / max(dt, 0.001), speed))

    # --- ostatní ---
    def beep(self, hz=880, ms=150):
        audio.play_tone(int(hz), int(ms))  # neblokuje

    def mock_blocks(self, pairs, spacing_mm):
        pass
