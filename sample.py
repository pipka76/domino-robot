import brian.motors as motors
import brian.sensors as sensors
import time

# --- inicializace ---
left  = motors.EV3LargeMotor(motors.MotorPort.A)
right = motors.EV3LargeMotor(motors.MotorPort.B)
arm = motors.EV3MediumMotor(motors.MotorPort.C)

colour = sensors.EV3.ColorSensorEV3(sensors.SensorPort.S4)
gyro = sensors.EV3.GyroSensorEV3(sensors.SensorPort.S3)

IsCW = 0
IsCCW = 0

IsLeft = 0
IsRight = 0

baseColor = colour.detected_color()

# počkat, než je všechno připravené
for m in (left, right, arm):
    m.wait_until_ready()

colour.set_mode(colour.Mode.COLOR_DETECT)
colour.wait_until_ready()

def stop():
    left.brake()
    right.brake()

def rotate(angle):
    left.reset_angle()
    right.reset_angle()
    left.rotate_to(angle * 2.9, 150, 0)
    right.rotate_to(-angle * 2.9, 150, 0)

def run_until_obstacle(distance):
    left.run_at_speed(150)
    right.run_at_speed(150)
    while ultra.distance_mm() > distance:
        time.sleep(0.1)
    stop()

def run_forward(vel = 500): # default speed 500
    v = 150
    t = 0
    while t < 0.35 and v < vel:
        left.run_at_speed(v)
        right.run_at_speed(v)
        v += 1
        t += 0.001
        time.sleep(0.001)


def arm_rotate_block_cw():
    global IsCW, IsCCW
    arm.reset_angle()

    if IsCCW == 1:
        arm.rotate_to(-2000, 1000, 0)   
    if IsCCW == 0 and IsCW == 0:
        arm.rotate_to(-1000, 1000, 0)
    
    IsCW = 1
    IsCCW = 0

def arm_rotate_block_ccw():
    global IsCW, IsCCW
    arm.reset_angle()

    if IsCW == 1:
        arm.rotate_to(2000, 1000, 0)
    if IsCW == 0 and IsCCW == 0:
        arm.rotate_to(1000, 1000, 0)

    IsCCW = 1
    IsCW = 0

def mag_set_left():
    global IsCW, IsCCW, IsRight, IsLeft
    arm.reset_angle()

    if IsCCW == 1 and IsRight == 1:
        arm.rotate_to(-4200, 1400, 0)
    if IsCW == 1 and IsRight == 1:
        arm.rotate_to(-6800, 1400, 0)
    if IsCW == 0 and IsCCW == 0 and IsRight == 1:
        arm.rotate_to(-5100, 1400, 0)

    IsCW = 1
    IsCCW = 0

    IsLeft = 1
    IsRight = 0

def mag_set_right():
    global IsCW, IsCCW, IsRight, IsLeft
    arm.reset_angle()

    if IsCW == 1 and IsLeft == 1:
        arm.rotate_to(4200, 1400, 0)
    if IsCCW == 1 and IsLeft == 1:
        arm.rotate_to(6800, 1400, 0)
    if IsCW == 0 and IsCCW == 0 and IsLeft == 1:
        arm.rotate_to(5100, 1400, 0)

    IsCW = 0
    IsCCW = 1

    IsLeft = 0
    IsRight = 1

def mag_correct():
    global IsCW, IsCCW, IsRight, IsLeft
    arm.reset_angle()

    if IsRight == 1:
        if IsCCW == 1:
            arm.rotate_to(4200, 1400, 0)
        if IsCW == 1:
            arm.rotate_to(-2000, 1400, 0)
            
    if IsRight == 1:
            if IsCCW == 1:
                arm.rotate_to(-2000, 1400, 0)
            if IsCW == 1:
                arm.rotate_to(4200, 1400, 0)


def unload_blocks():
    mag_correct()
    time.sleep(0.5)
    left.run_at_speed(-300)
    right.run_at_speed(-300)
    time.sleep(1)
    stop()
    mag_change_slot()
    time.sleep(3)
    left.run_at_speed(300)
    right.run_at_speed(300)
    time.sleep(1)
    left.run_at_speed(-300)
    right.run_at_speed(-300)
    time.sleep(2)
    stop()

def run_until_colour_change(vel=200, match = baseColor):
    run_forward(vel)
    while colour.detected_color() == match:
        time.sleep(0.01)
    stop()

def rotate_by_colour(wanted = colour.Color.RED):
    if colour.detected_color() == wanted:
        arm_rotate_block_cw()
    else:
        arm_rotate_block_ccw()

def mag_change_slot():
    global IsLeft, IsRight
    if IsLeft == 1:
        mag_set_right()
    elif IsRight == 1:
        mag_set_left()

def gather_start_line():
    run_until_colour_change()
    rotate_by_colour(colour.Color.GREEN)
    run_forward()
    time.sleep(0.5)
    stop()

    run_until_colour_change()
    run_forward()
    time.sleep(0.5)
    stop()
    mag_change_slot()
    time.sleep(3)

    run_until_colour_change()
    rotate_by_colour(colour.Color.BLUE)
    run_forward()
    time.sleep(0.5)
    stop()

    run_until_colour_change()
    run_forward(300)
    time.sleep(0.5)
    stop()

def turn(angle):
    k = angle / 90
    left.run_at_speed(100)
    right.run_at_speed(500)
    time.sleep(1.34 * k)
    stop()

def run_until_time(duration = 1, spd = 500):
    run_forward(spd)
    while(duration > 0):
        time.sleep(0.01)
        duration -= 0.01
    stop()

IsLeft = 0
IsRight = 1
IsCW = 0
IsCCW = 1

gather_start_line()
turn(135)

run_until_time(1.2)
turn(45)

run_until_time(0.5)
unload_blocks()

time.sleep(1)