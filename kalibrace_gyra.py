"""Ruční kalibrace gyra – spustit před každou jízdou.
Robot musí stát úplně v klidu, dokud nepípne. Port a pauza se berou z mission.json."""
import json
import time

import brian.audio as audio
import brian.sensors as sensors

with open("mission.json") as f:
    cfg = json.load(f)
drive = cfg["robot"]["drive"]
port = cfg["robot"]["ports"]["gyro"]
delay = drive.get("gyro_calib_delay_s", 1.0)

gyro = sensors.EV3.GyroSensorEV3(getattr(sensors.SensorPort, port))
print("Kalibrace gyra na {} za {} s – NEHÝBEJ s robotem".format(port, delay))
time.sleep(delay)
gyro.reboot()
time.sleep(0.2)
gyro.set_mode(gyro.Mode.ANGLE)
if not gyro.wait_until_ready(5000):
    print("CHYBA: gyro nepřipraveno – zkontroluj kabel a port", port)
    audio.play_tone(200, 800)
else:
    gyro.set_zero_point()
    time.sleep(0.5)
    a0 = gyro.angle()
    time.sleep(3)
    d = gyro.angle() - a0
    if abs(d) >= 1:
        print("! Gyro ujíždí {:+} deg za 3 s – spusť kalibraci znovu".format(d))
        for _ in range(3):
            audio.play_tone(300, 150)
            time.sleep(0.25)
    else:
        print("Gyro OK (drift {:+} deg za 3 s)".format(d))
        audio.play_tone(1200, 300)
