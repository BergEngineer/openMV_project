# ------------------------------------------------------------------------------------------------
# PROGRAM: RED BALL TRACKING WITH FILTERED PD CONTROL
# ------------------------------------------------------------------------------------------------

import sensor
import time
from pyb import Pin, Timer, ADC


# ------------------------------------------------------------------------------------------------
# CAMERA SETUP
# ------------------------------------------------------------------------------------------------

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.skip_frames(time=2000)

sensor.set_hmirror(True)
sensor.set_vflip(True)

sensor.set_auto_exposure(False, exposure_us=50000)

clock = time.clock()


# ------------------------------------------------------------------------------------------------
# OPTIONAL INPUTS / OUTPUTS
# ------------------------------------------------------------------------------------------------

proximity = ADC(Pin("P6", Pin.IN))
ball_sensor = Pin("P8", Pin.IN)
buzzer = Pin("P9", Pin.OUT_PP)

buzzer.low()


# ------------------------------------------------------------------------------------------------
# COLOR THRESHOLD
#
# Format:
# (L_min, L_max, A_min, A_max, B_min, B_max)
#
# Recalibrate with:
# Tools -> Machine Vision -> Threshold Editor
# ------------------------------------------------------------------------------------------------

RED_THRESHOLD = (0, 70, 23, 127, -19, 53)


# ------------------------------------------------------------------------------------------------
# BALL DETECTION PARAMETERS
# ------------------------------------------------------------------------------------------------

BALL_PIXELS_MIN = 80
BALL_PIXELS_MAX = 8000


# ------------------------------------------------------------------------------------------------
# FRAME CENTER
# ------------------------------------------------------------------------------------------------

CX = sensor.width() // 2
CY = sensor.height() // 2


# ------------------------------------------------------------------------------------------------
# MOTOR SETUP
#
# IMPORTANT:
#
# PWM = 0   -> faster
# PWM = 100 -> slower / almost stopped
#
# Therefore the PWM logic is inverted.
# ------------------------------------------------------------------------------------------------

EN = Pin("P7", Pin.OUT_PP)
EN.low()

timer = Timer(2, freq=1000)

LEFT_MOTOR_PWM = timer.channel(
    4,
    Timer.PWM,
    pin=Pin("P4")
)

RIGHT_MOTOR_PWM = timer.channel(
    3,
    Timer.PWM,
    pin=Pin("P5")
)


# ------------------------------------------------------------------------------------------------
# MOTOR SPEED PARAMETERS
# ------------------------------------------------------------------------------------------------

# Higher PWM = slower robot.
#
# Previous value was around 35.
# 48 makes the robot noticeably slower and easier to control.

BASE_PWM = 38

# Search should also be relatively slow.
SEARCH_FAST_PWM = 38
SEARCH_SLOW_PWM = 100

PWM_MIN = 0
PWM_MAX = 100


# ------------------------------------------------------------------------------------------------
# PD PARAMETERS
# ------------------------------------------------------------------------------------------------

# Start conservative.
KP = 0.40

# Integral intentionally disabled.
KI = 0.0

KD = 0.07

# Maximum steering influence.
MAX_CORRECTION = 18

# Ignore very small horizontal errors.
DEADBAND = 12

# Derivative low-pass filtering.
#
# Higher value = smoother derivative.
DERIVATIVE_FILTER = 0.75


# ------------------------------------------------------------------------------------------------
# TARGET LOSS PARAMETERS
# ------------------------------------------------------------------------------------------------

# Do not immediately enter SEARCH when the ball disappears.
#
# CamBot will tolerate temporary target loss for 350 ms.
TARGET_LOST_TIMEOUT_MS = 350

last_target_seen_time = time.ticks_ms()

# Last motor commands are stored so CamBot can temporarily
# continue its previous trajectory when the target disappears.

last_left_pwm = BASE_PWM
last_right_pwm = BASE_PWM


# ------------------------------------------------------------------------------------------------
# PD INTERNAL STATE
# ------------------------------------------------------------------------------------------------

previous_error = 0
previous_derivative = 0

last_pid_time = time.ticks_ms()


# ------------------------------------------------------------------------------------------------
# HELPER FUNCTIONS
# ------------------------------------------------------------------------------------------------

def clamp(value, minimum, maximum):

    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


# ------------------------------------------------------------------------------------------------
# MOTOR FUNCTIONS
# ------------------------------------------------------------------------------------------------

def set_motor_pwm(left_pwm, right_pwm):

    global last_left_pwm
    global last_right_pwm

    left_pwm = int(
        clamp(
            left_pwm,
            PWM_MIN,
            PWM_MAX
        )
    )

    right_pwm = int(
        clamp(
            right_pwm,
            PWM_MIN,
            PWM_MAX
        )
    )

    LEFT_MOTOR_PWM.pulse_width_percent(
        left_pwm
    )

    RIGHT_MOTOR_PWM.pulse_width_percent(
        right_pwm
    )

    last_left_pwm = left_pwm
    last_right_pwm = right_pwm


def move_forward():

    set_motor_pwm(
        BASE_PWM,
        BASE_PWM
    )


def stop_motors():

    set_motor_pwm(
        100,
        100
    )


def search_for_target():

    # Slow turning motion.
    #
    # Lower PWM = faster motor.
    # Left motor is slightly faster than right motor.

    set_motor_pwm(
        SEARCH_FAST_PWM,
        SEARCH_SLOW_PWM
    )


# ------------------------------------------------------------------------------------------------
# BALL DETECTION
# ------------------------------------------------------------------------------------------------

def find_biggest_red_ball(img):

    img.draw_cross(
        CX,
        CY,
        color=(0, 0, 255),
        size=12,
        thickness=2
    )

    blobs = img.find_blobs(
        [RED_THRESHOLD],
        pixels_threshold=200,
        area_threshold=200,
        merge=True
    )

    if not blobs:
        return None

    valid_blobs = []

    for blob in blobs:

        pixels = blob.pixels()

        if pixels < BALL_PIXELS_MIN:
            continue

        if pixels > BALL_PIXELS_MAX:
            continue

        valid_blobs.append(blob)

        img.draw_rectangle(
            blob.rect(),
            color=(255, 0, 0)
        )

        img.draw_cross(
            blob.cx(),
            blob.cy(),
            color=(255, 0, 0)
        )

    if not valid_blobs:
        return None

    biggest = max(
        valid_blobs,
        key=lambda b: b.pixels()
    )

    error_x = CX - biggest.cx()

    img.draw_string(
        biggest.x(),
        max(0, biggest.y() - 20),
        "X error: %d" % error_x,
        color=(255, 220, 0),
        scale=2
    )

    return biggest


# ------------------------------------------------------------------------------------------------
# FILTERED PD CONTROLLER
# ------------------------------------------------------------------------------------------------

def pd_steering(target):

    global previous_error
    global previous_derivative
    global last_pid_time

    now = time.ticks_ms()

    dt_ms = time.ticks_diff(
        now,
        last_pid_time
    )

    last_pid_time = now

    if dt_ms <= 0:
        dt_ms = 1

    dt = dt_ms / 1000.0


    # --------------------------------------------------------------------------------------------
    # ERROR
    #
    # Positive error:
    # target is on the LEFT.
    #
    # Negative error:
    # target is on the RIGHT.
    # --------------------------------------------------------------------------------------------

    error = CX - target.cx()


    # --------------------------------------------------------------------------------------------
    # DEADBAND
    # --------------------------------------------------------------------------------------------

    if abs(error) < DEADBAND:
        error = 0


    # --------------------------------------------------------------------------------------------
    # DERIVATIVE
    # --------------------------------------------------------------------------------------------

    raw_derivative = (
        error - previous_error
    ) / dt


    # Low-pass filter the derivative.
    #
    # This prevents small frame-to-frame changes from producing
    # aggressive steering corrections.

    derivative = (
        DERIVATIVE_FILTER * previous_derivative
        +
        (1.0 - DERIVATIVE_FILTER) * raw_derivative
    )


    previous_derivative = derivative
    previous_error = error


    # --------------------------------------------------------------------------------------------
    # PD OUTPUT
    # --------------------------------------------------------------------------------------------

    correction = (
        KP * error
        +
        KD * derivative
    )

    correction = clamp(
        correction,
        -MAX_CORRECTION,
        MAX_CORRECTION
    )


    # --------------------------------------------------------------------------------------------
    # INVERTED PWM STEERING
    #
    # Target LEFT:
    #
    # left wheel should slow down
    # right wheel should speed up
    #
    # Since higher PWM = slower:
    #
    # left PWM  increases
    # right PWM decreases
    # --------------------------------------------------------------------------------------------

    left_pwm = (
        BASE_PWM + correction
    )

    right_pwm = (
        BASE_PWM - correction
    )


    left_pwm = int(
        clamp(
            left_pwm,
            PWM_MIN,
            PWM_MAX
        )
    )

    right_pwm = int(
        clamp(
            right_pwm,
            PWM_MIN,
            PWM_MAX
        )
    )


    set_motor_pwm(
        left_pwm,
        right_pwm
    )


    return (
        error,
        correction,
        left_pwm,
        right_pwm
    )


# ------------------------------------------------------------------------------------------------
# RESET PD
# ------------------------------------------------------------------------------------------------

def reset_pd():

    global previous_error
    global previous_derivative
    global last_pid_time

    previous_error = 0
    previous_derivative = 0

    last_pid_time = time.ticks_ms()


# ------------------------------------------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------------------------------------------

while True:

    clock.tick()

    img = sensor.snapshot()

    target = find_biggest_red_ball(
        img
    )

    now = time.ticks_ms()


    # --------------------------------------------------------------------------------------------
    # TARGET FOUND
    # --------------------------------------------------------------------------------------------

    if target is not None:

        last_target_seen_time = now

        error, correction, left_pwm, right_pwm = pd_steering(
            target
        )

        print(
            "TRACK | error:",
            error,
            "| correction:",
            correction,
            "| left PWM:",
            left_pwm,
            "| right PWM:",
            right_pwm
        )


    # --------------------------------------------------------------------------------------------
    # TARGET TEMPORARILY LOST
    # --------------------------------------------------------------------------------------------

    else:

        time_since_target = time.ticks_diff(
            now,
            last_target_seen_time
        )

        if time_since_target < TARGET_LOST_TIMEOUT_MS:

            # Keep the previous motor command for a short time.
            #
            # This prevents CamBot from immediately entering SEARCH
            # because of a single bad frame.

            set_motor_pwm(
                last_left_pwm,
                last_right_pwm
            )

            print(
                "Target temporarily lost..."
            )


        # ----------------------------------------------------------------------------------------
        # TARGET REALLY LOST
        # ----------------------------------------------------------------------------------------

        else:

            reset_pd()

            search_for_target()

            print(
                "Searching for red target..."
            )
