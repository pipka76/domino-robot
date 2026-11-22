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
                # reboot = rekalibrace gyra, robot musí stát v klidu
                self.gyro.reboot()
                time.sleep(0.2)
            self.gyro.set_mode(self.gyro.Mode.ANGLE)
            if self.gyro.wait_until_ready(5000):
                self.gyro.set_zero_point()
            else:
                self.log("GYRO nepřipraveno, točím podle enkodérů")
                self.gyro = None

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
    def arm_move(self, deg, speed):
        """Relativní pohyb motoru C, čeká na dokončení (s timeoutem)."""
        self.check_time()
        est_ms = abs(deg) * 1000.0 / max(abs(speed), 1)
        timeout = int(est_ms * 1.5 + self.arm_cfg.get("extra_wait_s", 0.3) * 1000)
        res = self.arm.rotate_by_angle(int(deg), int(speed), timeout)
        if res == motors.MovementEnd.TIMED_OUT or self.arm.is_stalled():
            self.arm.brake()  # při timeoutu se motor sám nezastaví
            self.log("    arm: timeout/zablokováno (cíl {:+} deg)".format(deg))

    # --- ostatní ---
    def beep(self, hz=880, ms=150):
        audio.play_tone(int(hz), int(ms))  # neblokuje

    def mock_blocks(self, pairs, spacing_mm):
        pass
