# ============================================================
# CamBot - Autonomous Visual Explorer (STANDALONE)
# OpenMV H7 Plus + L9110S + VL53L0X
#
# No external Python libraries are required.
#
# Wiring used by this file
# ------------------------------------------------------------
# L9110S:
#   P4 -> LEFT motor forward input
#   P5 -> RIGHT motor forward input
#   P6 -> LEFT motor backward input
#   P9 -> RIGHT motor backward input
#
# VL53L0X:
#   P7 -> SCL
#   P8 -> SDA
#   GND -> GND
#   VIN/VCC -> compatible supply for your breakout
#
# OpenMV H7 Plus:
#   VL53L0X uses hardware I2C bus 4
#
# IMPORTANT:
# - Test the robot with the wheels lifted first.
# - Calibrate TARGET_THRESHOLD with OpenMV IDE Threshold Editor.
# - This build deliberately uses forward-only steering/pivoting
#   during normal autonomous motion to avoid abrupt reverse surges.
# ============================================================

import sensor
import time
import random

from machine import I2C
from pyb import Pin, Timer


# ============================================================
# USER CONFIGURATION
# ============================================================

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
CENTER_X = FRAME_WIDTH // 2

# Example only. Replace using OpenMV Threshold Editor.
TARGET_THRESHOLD = (100, 0, 127, 16, 3, 127)

PIXELS_THRESHOLD = 250
AREA_THRESHOLD = 250
MIN_TARGET_AREA = 350

CENTER_TOLERANCE = 18

# Motor commands are logical speeds.
MIN_MOVING_SPEED = 30

BASE_SPEED = 45
TRACK_SPEED = 37
APPROACH_SPEED = 34
TURN_SPEED = 27
MAX_SPEED = 60

# You observed that reducing forward PWM makes the motors run faster.
PWM_INVERTED = True

# Change if CamBot steers away from the object instead of toward it.
INVERT_STEERING = False

# P-controller for target centering.
KP_STEERING = 0.18
MAX_STEERING = 18

# ToF thresholds.
OBSTACLE_DISTANCE_MM = 260
INSPECTION_DISTANCE_MM = 320

# Reading validation/filtering.
TOF_MIN_VALID_MM = 35
TOF_MAX_VALID_MM = 1800
TOF_FILTER_SAMPLES = 5
TOF_TIMEOUT_MS = 200

# State timing.
TURN_TIME_MS = 650
INSPECTION_TIME_MS = 2200
TARGET_LOST_MS = 650

# Keep this False until basic obstacle avoidance works reliably.
ENABLE_RANDOM_TURNS = False
RANDOM_TURN_MIN_MS = 5000
RANDOM_TURN_MAX_MS = 9000

DEBUG = True


# ============================================================
# HELPERS
# ============================================================

def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def median(values):
    values = sorted(values)
    return values[len(values) // 2]


# ============================================================
# CAMERA
# ============================================================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.skip_frames(time=1500)

# Freeze automatic color-changing controls so LAB thresholds
# stay much more stable.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)

sensor.set_hmirror(True)
sensor.set_auto_exposure(False, exposure_us=50000)

clock = time.clock()


# ============================================================
# L9110S MOTOR CONTROL
# ============================================================
#
# Normal autonomous motion uses:
#
# LEFT:
#   forward P4 = PWM
#   backward P6 = LOW
#
# RIGHT:
#   forward P5 = PWM
#   backward P9 = LOW
#
# Turning is accomplished by slowing/stopping one wheel.
# This avoids the abrupt full-power reverse behaviour we saw
# in the first build.

timer2 = Timer(2, freq=1000)

right_forward_pwm = timer2.channel(
    3,
    Timer.PWM,
    pin=Pin("P4")
)

left_forward_pwm = timer2.channel(
    4,
    Timer.PWM,
    pin=Pin("P5")
)

left_backward = Pin("P6", Pin.OUT_PP)
right_backward = Pin("P9", Pin.OUT_PP)

left_backward.low()
right_backward.low()


def pwm_from_speed(speed):
    speed = clamp(speed, 0, 100)

    if speed == 0:
        return 0

    if PWM_INVERTED:
        return 100 - speed

    return speed


def set_left_forward(speed):
    left_backward.low()
    left_forward_pwm.pulse_width_percent(
        pwm_from_speed(speed)
    )


def set_right_forward(speed):
    right_backward.low()
    right_forward_pwm.pulse_width_percent(
        pwm_from_speed(speed)
    )


def set_motors(left_speed, right_speed):
    left_speed = clamp(left_speed, 0, MAX_SPEED)
    right_speed = clamp(right_speed, 0, MAX_SPEED)

    # Dead-zone compensation:
    # if a wheel is commanded to move, never let its logical speed
    # fall below the minimum value needed to overcome static friction.
    if 0 < left_speed < MIN_MOVING_SPEED:
        left_speed = MIN_MOVING_SPEED

    if 0 < right_speed < MIN_MOVING_SPEED:
        right_speed = MIN_MOVING_SPEED

    set_left_forward(left_speed)
    set_right_forward(right_speed)


def stop_motors():
    left_backward.low()
    right_backward.low()

    left_forward_pwm.pulse_width_percent(0)
    right_forward_pwm.pulse_width_percent(0)


def pivot_left(speed=TURN_SPEED):
    # left wheel stopped, right wheel forward
    set_motors(0, speed)


def pivot_right(speed=TURN_SPEED):
    # right wheel stopped, left wheel forward
    set_motors(speed, 0)


# ============================================================
# STANDALONE VL53L0X DRIVER
# ============================================================
#
# This is a compact original implementation for CamBot based on
# the VL53L0X register interface and initialization requirements.
#
# It:
# - checks the model ID
# - enables 2.8 V I/O mode
# - obtains the internal stop variable
# - configures reference SPADs
# - applies the normal reference tuning configuration
# - performs the two reference calibrations
# - starts continuous ranging
#
# It intentionally exposes only the functions CamBot needs.

class VL53L0X:

    ADDRESS = 0x29

    SYSRANGE_START = 0x00
    SYSTEM_SEQUENCE_CONFIG = 0x01
    SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
    SYSTEM_INTERRUPT_CLEAR = 0x0B

    RESULT_INTERRUPT_STATUS = 0x13
    RESULT_RANGE_STATUS = 0x14

    MSRC_CONFIG_CONTROL = 0x60
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44

    GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
    GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6

    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E
    DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F

    VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV = 0x89
    IDENTIFICATION_MODEL_ID = 0xC0

    def __init__(self, i2c, address=0x29, timeout_ms=200):
        self.i2c = i2c
        self.address = address
        self.timeout_ms = timeout_ms
        self.stop_variable = 0

        if self.address not in self.i2c.scan():
            raise RuntimeError(
                "VL53L0X not found at address 0x29"
            )

        model_id = self.read_u8(self.IDENTIFICATION_MODEL_ID)

        # The VL53L0X model ID should be 0xEE.
        if model_id != 0xEE:
            raise RuntimeError(
                "Unexpected VL53L0X model ID: 0x%02X" % model_id
            )

        self._data_init()
        self._static_init()
        self._reference_calibration()

    # --------------------------------------------------------
    # RAW I2C
    # --------------------------------------------------------

    def write_u8(self, reg, value):
        self.i2c.writeto_mem(
            self.address,
            reg,
            bytes((value & 0xFF,))
        )

    def write_u16(self, reg, value):
        data = bytes((
            (value >> 8) & 0xFF,
            value & 0xFF
        ))

        self.i2c.writeto_mem(
            self.address,
            reg,
            data
        )

    def read_u8(self, reg):
        return self.i2c.readfrom_mem(
            self.address,
            reg,
            1
        )[0]

    def read_u16(self, reg):
        data = self.i2c.readfrom_mem(
            self.address,
            reg,
            2
        )

        return (data[0] << 8) | data[1]

    def read_block(self, reg, length):
        return self.i2c.readfrom_mem(
            self.address,
            reg,
            length
        )

    def write_block(self, reg, data):
        self.i2c.writeto_mem(
            self.address,
            reg,
            data
        )

    # --------------------------------------------------------
    # TIMEOUT
    # --------------------------------------------------------

    def wait_for(self, function):
        start = time.ticks_ms()

        while not function():

            if time.ticks_diff(
                time.ticks_ms(),
                start
            ) > self.timeout_ms:

                raise RuntimeError(
                    "VL53L0X timeout"
                )

            time.sleep_ms(1)

    # --------------------------------------------------------
    # INITIAL DATA SETUP
    # --------------------------------------------------------

    def _data_init(self):

        # Enable 2.8 V-compatible I/O pads.
        value = self.read_u8(
            self.VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV
        )

        self.write_u8(
            self.VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
            value | 0x01
        )

        # Standard I2C mode.
        self.write_u8(0x88, 0x00)

        # Access hidden register bank and preserve StopVariable.
        self.write_u8(0x80, 0x01)
        self.write_u8(0xFF, 0x01)
        self.write_u8(0x00, 0x00)

        self.stop_variable = self.read_u8(0x91)

        self.write_u8(0x00, 0x01)
        self.write_u8(0xFF, 0x00)
        self.write_u8(0x80, 0x00)

        # Disable two early signal-rate checks.
        self.write_u8(
            self.MSRC_CONFIG_CONTROL,
            self.read_u8(
                self.MSRC_CONFIG_CONTROL
            ) | 0x12
        )

        # Approx. 0.25 MCPS in 9.7 fixed-point form.
        self.write_u16(
            self.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
            int(0.25 * (1 << 7))
        )

        self.write_u8(
            self.SYSTEM_SEQUENCE_CONFIG,
            0xFF
        )

    # --------------------------------------------------------
    # SPAD INFORMATION
    # --------------------------------------------------------

    def _get_spad_info(self):

        self.write_u8(0x80, 0x01)
        self.write_u8(0xFF, 0x01)
        self.write_u8(0x00, 0x00)

        self.write_u8(0xFF, 0x06)

        value = self.read_u8(0x83)
        self.write_u8(0x83, value | 0x04)

        self.write_u8(0xFF, 0x07)
        self.write_u8(0x81, 0x01)

        self.write_u8(0x80, 0x01)

        self.write_u8(0x94, 0x6B)
        self.write_u8(0x83, 0x00)

        self.wait_for(
            lambda: self.read_u8(0x83) != 0
        )

        self.write_u8(0x83, 0x01)

        tmp = self.read_u8(0x92)

        count = tmp & 0x7F
        is_aperture = bool((tmp >> 7) & 0x01)

        self.write_u8(0x81, 0x00)

        self.write_u8(0xFF, 0x06)

        value = self.read_u8(0x83)
        self.write_u8(
            0x83,
            value & ~0x04
        )

        self.write_u8(0xFF, 0x01)
        self.write_u8(0x00, 0x01)

        self.write_u8(0xFF, 0x00)
        self.write_u8(0x80, 0x00)

        return count, is_aperture

    # --------------------------------------------------------
    # STATIC INITIALIZATION
    # --------------------------------------------------------

    def _static_init(self):

        spad_count, aperture = self._get_spad_info()

        spad_map = bytearray(
            self.read_block(
                self.GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                6
            )
        )

        self.write_u8(0xFF, 0x01)

        self.write_u8(
            self.DYNAMIC_SPAD_REF_EN_START_OFFSET,
            0x00
        )

        self.write_u8(
            self.DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD,
            0x2C
        )

        self.write_u8(0xFF, 0x00)

        self.write_u8(
            self.GLOBAL_CONFIG_REF_EN_START_SELECT,
            0xB4
        )

        first_spad = 12 if aperture else 0

        enabled = 0

        for i in range(48):

            byte_index = i // 8
            bit_index = i % 8

            mask = 1 << bit_index

            if i < first_spad or enabled >= spad_count:

                spad_map[byte_index] &= ~mask

            elif spad_map[byte_index] & mask:

                enabled += 1

        self.write_block(
            self.GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
            spad_map
        )

        # Reference/tuning setup.
        #
        # These register settings put the sensor into the normal
        # ranging configuration used after ST's data/static init.

        settings = (
            (0xFF, 0x01),
            (0x00, 0x00),

            (0xFF, 0x00),
            (0x09, 0x00),
            (0x10, 0x00),
            (0x11, 0x00),

            (0x24, 0x01),
            (0x25, 0xFF),
            (0x75, 0x00),

            (0xFF, 0x01),
            (0x4E, 0x2C),
            (0x48, 0x00),
            (0x30, 0x20),

            (0xFF, 0x00),
            (0x30, 0x09),
            (0x54, 0x00),
            (0x31, 0x04),
            (0x32, 0x03),
            (0x40, 0x83),
            (0x46, 0x25),
            (0x60, 0x00),
            (0x27, 0x00),
            (0x50, 0x06),
            (0x51, 0x00),
            (0x52, 0x96),
            (0x56, 0x08),
            (0x57, 0x30),
            (0x61, 0x00),
            (0x62, 0x00),
            (0x64, 0x00),
            (0x65, 0x00),
            (0x66, 0xA0),

            (0xFF, 0x01),
            (0x22, 0x32),
            (0x47, 0x14),
            (0x49, 0xFF),
            (0x4A, 0x00),

            (0xFF, 0x00),
            (0x7A, 0x0A),
            (0x7B, 0x00),
            (0x78, 0x21),

            (0xFF, 0x01),
            (0x23, 0x34),
            (0x42, 0x00),
            (0x44, 0xFF),
            (0x45, 0x26),
            (0x46, 0x05),
            (0x40, 0x40),
            (0x0E, 0x06),
            (0x20, 0x1A),
            (0x43, 0x40),

            (0xFF, 0x00),
            (0x34, 0x03),
            (0x35, 0x44),

            (0xFF, 0x01),
            (0x31, 0x04),
            (0x4B, 0x09),
            (0x4C, 0x05),
            (0x4D, 0x04),

            (0xFF, 0x00),
            (0x44, 0x00),
            (0x45, 0x20),
            (0x47, 0x08),
            (0x48, 0x28),
            (0x67, 0x00),
            (0x70, 0x04),
            (0x71, 0x01),
            (0x72, 0xFE),
            (0x76, 0x00),
            (0x77, 0x00),

            (0xFF, 0x01),
            (0x0D, 0x01),

            (0xFF, 0x00),
            (0x80, 0x01),
            (0x01, 0xF8),

            (0xFF, 0x01),
            (0x8E, 0x01),
            (0x00, 0x01),

            (0xFF, 0x00),
            (0x80, 0x00),
        )

        for reg, value in settings:
            self.write_u8(reg, value)

        # GPIO interrupt polarity / "new sample ready".
        self.write_u8(
            self.SYSTEM_INTERRUPT_CONFIG_GPIO,
            0x04
        )

        gpio_hv = self.read_u8(0x84)

        self.write_u8(
            0x84,
            gpio_hv & ~0x10
        )

        self.write_u8(
            self.SYSTEM_INTERRUPT_CLEAR,
            0x01
        )

    # --------------------------------------------------------
    # REFERENCE CALIBRATION
    # --------------------------------------------------------

    def _perform_ref_calibration(self, vhv_init_byte):

        self.write_u8(
            self.SYSRANGE_START,
            0x01 | vhv_init_byte
        )

        self.wait_for(
            lambda: (
                self.read_u8(
                    self.RESULT_INTERRUPT_STATUS
                ) & 0x07
            ) != 0
        )

        self.write_u8(
            self.SYSTEM_INTERRUPT_CLEAR,
            0x01
        )

        self.write_u8(
            self.SYSRANGE_START,
            0x00
        )

    def _reference_calibration(self):

        # VHV calibration.
        self.write_u8(
            self.SYSTEM_SEQUENCE_CONFIG,
            0x01
        )

        self._perform_ref_calibration(0x40)

        # Phase calibration.
        self.write_u8(
            self.SYSTEM_SEQUENCE_CONFIG,
            0x02
        )

        self._perform_ref_calibration(0x00)

        self.write_u8(
            self.SYSTEM_SEQUENCE_CONFIG,
            0xE8
        )

    # --------------------------------------------------------
    # CONTINUOUS MODE
    # --------------------------------------------------------

    def start_continuous(self):

        # Restore the private stop variable before ranging.
        self.write_u8(0x80, 0x01)
        self.write_u8(0xFF, 0x01)
        self.write_u8(0x00, 0x00)
        self.write_u8(0x91, self.stop_variable)
        self.write_u8(0x00, 0x01)
        self.write_u8(0xFF, 0x00)
        self.write_u8(0x80, 0x00)

        # Back-to-back continuous ranging.
        self.write_u8(
            self.SYSRANGE_START,
            0x02
        )

    def read_continuous_mm(self):

        self.wait_for(
            lambda: (
                self.read_u8(
                    self.RESULT_INTERRUPT_STATUS
                ) & 0x07
            ) != 0
        )

        # Range value in RESULT_RANGE_STATUS + 10.
        distance = self.read_u16(
            self.RESULT_RANGE_STATUS + 10
        )

        self.write_u8(
            self.SYSTEM_INTERRUPT_CLEAR,
            0x01
        )

        return distance


# ============================================================
# INITIALIZE VL53L0X
# ============================================================

i2c = I2C(4, freq=400000)

print("I2C scan:", i2c.scan())

tof = VL53L0X(
    i2c,
    address=0x29,
    timeout_ms=TOF_TIMEOUT_MS
)

tof.start_continuous()

print("VL53L0X initialized")
print("VL53L0X continuous ranging started")


def distance_mm():

    readings = []

    for _ in range(TOF_FILTER_SAMPLES):

        try:
            d = int(
                tof.read_continuous_mm()
            )

            if (
                TOF_MIN_VALID_MM
                <= d
                <= TOF_MAX_VALID_MM
            ):
                readings.append(d)

        except Exception as error:

            if DEBUG:
                print(
                    "TOF error:",
                    error
                )

        time.sleep_ms(2)

    required = (
        TOF_FILTER_SAMPLES // 2
    ) + 1

    if len(readings) < required:
        return None

    return median(readings)


# ============================================================
# VISION
# ============================================================

def find_target(img):

    blobs = img.find_blobs(
        [TARGET_THRESHOLD],
        pixels_threshold=PIXELS_THRESHOLD,
        area_threshold=AREA_THRESHOLD,
        merge=True
    )

    best = None
    best_area = 0

    for blob in blobs:

        area = (
            blob.w()
            * blob.h()
        )

        if (
            area >= MIN_TARGET_AREA
            and area > best_area
        ):
            best = blob
            best_area = area

    return best


# ============================================================
# STEERING
# ============================================================

def drive_toward_target(target, base_speed):

    error = target.cx() - CENTER_X

    if INVERT_STEERING:
        error = -error

    correction = int(
        clamp(
            KP_STEERING * error,
            -MAX_STEERING,
            MAX_STEERING
        )
    )

    left_speed = int(
        clamp(
            base_speed + correction,
            0,
            MAX_SPEED
        )
    )

    right_speed = int(
        clamp(
            base_speed - correction,
            0,
            MAX_SPEED
        )
    )

    set_motors(
        left_speed,
        right_speed
    )

    return error


# ============================================================
# STATE MACHINE
# ============================================================

EXPLORE = 0
AVOID_TURN = 1
TRACK = 2
APPROACH = 3
INSPECT = 4

STATE_NAME = {
    EXPLORE: "EXPLORE",
    AVOID_TURN: "AVOID_TURN",
    TRACK: "TRACK",
    APPROACH: "APPROACH",
    INSPECT: "INSPECT",
}

state = EXPLORE

state_started = (
    time.ticks_ms()
)

last_target_seen = (
    time.ticks_ms()
)

avoid_right = True

next_random_turn = time.ticks_add(
    time.ticks_ms(),
    random.randint(
        RANDOM_TURN_MIN_MS,
        RANDOM_TURN_MAX_MS
    )
)


def change_state(new_state):

    global state
    global state_started

    if state != new_state:

        state = new_state

        state_started = (
            time.ticks_ms()
        )

        print(
            "STATE ->",
            STATE_NAME[state]
        )


# ============================================================
# DEBUG OVERLAY
# ============================================================

def draw_debug(
    img,
    target,
    distance
):

    if not DEBUG:
        return

    img.draw_line(
        CENTER_X,
        0,
        CENTER_X,
        FRAME_HEIGHT,
        color=(0, 255, 0)
    )

    if target:

        img.draw_rectangle(
            target.rect(),
            color=(255, 0, 0),
            thickness=2
        )

        img.draw_cross(
            target.cx(),
            target.cy(),
            color=(255, 255, 0),
            size=10
        )

    img.draw_string(
        4,
        4,
        "S:" + STATE_NAME[state],
        color=(255, 255, 255)
    )

    if distance is None:
        d_text = "---"
    else:
        d_text = str(distance)

    img.draw_string(
        4,
        18,
        "D:" + d_text + "mm",
        color=(255, 255, 255)
    )

    img.draw_string(
        4,
        32,
        "FPS:%.1f" % clock.fps(),
        color=(255, 255, 255)
    )


# ============================================================
# START
# ============================================================

stop_motors()

print("")
print("================================")
print(" CamBot Autonomous Visual Explorer")
print(" STANDALONE VL53L0X BUILD")
print("================================")
print("")

time.sleep_ms(1000)


# ============================================================
# MAIN LOOP
# ============================================================

while True:

    clock.tick()

    now = time.ticks_ms()

    img = sensor.snapshot()

    target = find_target(img)

    distance = distance_mm()

    # --------------------------------------------------------
    # PRIORITY 1: OBSTACLE AVOIDANCE
    # --------------------------------------------------------

    if (
        distance is not None
        and distance <= OBSTACLE_DISTANCE_MM
        and state not in (
            AVOID_TURN,
            INSPECT
        )
    ):

        stop_motors()

        avoid_right = bool(
            random.getrandbits(1)
        )

        change_state(
            AVOID_TURN
        )

    # --------------------------------------------------------
    # EXPLORE
    # --------------------------------------------------------

    if state == EXPLORE:

        set_motors(
            BASE_SPEED,
            BASE_SPEED
        )

        if target is not None:

            last_target_seen = now

            change_state(
                TRACK
            )

        elif (
            ENABLE_RANDOM_TURNS
            and time.ticks_diff(
                now,
                next_random_turn
            ) >= 0
        ):

            avoid_right = bool(
                random.getrandbits(1)
            )

            change_state(
                AVOID_TURN
            )

    # --------------------------------------------------------
    # AVOID
    # --------------------------------------------------------

    elif state == AVOID_TURN:

        if avoid_right:
            pivot_right()
        else:
            pivot_left()

        if time.ticks_diff(
            now,
            state_started
        ) >= TURN_TIME_MS:

            stop_motors()

            next_random_turn = (
                time.ticks_add(
                    now,
                    random.randint(
                        RANDOM_TURN_MIN_MS,
                        RANDOM_TURN_MAX_MS
                    )
                )
            )

            change_state(
                EXPLORE
            )

    # --------------------------------------------------------
    # TRACK
    # --------------------------------------------------------

    elif state == TRACK:

        if target is not None:

            last_target_seen = now

            error = (
                drive_toward_target(
                    target,
                    TRACK_SPEED
                )
            )

            if (
                abs(error)
                <= CENTER_TOLERANCE
            ):
                change_state(
                    APPROACH
                )

        elif time.ticks_diff(
            now,
            last_target_seen
        ) > TARGET_LOST_MS:

            stop_motors()

            change_state(
                EXPLORE
            )

    # --------------------------------------------------------
    # APPROACH
    # --------------------------------------------------------

    elif state == APPROACH:

        if target is not None:

            last_target_seen = now

            if (
                distance is not None
                and distance <= INSPECTION_DISTANCE_MM
                and distance > OBSTACLE_DISTANCE_MM
            ):

                stop_motors()

                change_state(
                    INSPECT
                )

            else:

                drive_toward_target(
                    target,
                    APPROACH_SPEED
                )

        elif time.ticks_diff(
            now,
            last_target_seen
        ) > TARGET_LOST_MS:

            stop_motors()

            change_state(
                EXPLORE
            )

    # --------------------------------------------------------
    # INSPECT
    # --------------------------------------------------------

    elif state == INSPECT:

        stop_motors()

        if time.ticks_diff(
            now,
            state_started
        ) >= INSPECTION_TIME_MS:

            avoid_right = bool(
                random.getrandbits(1)
            )

            change_state(
                AVOID_TURN
            )

    # --------------------------------------------------------
    # OPENMV IDE OVERLAY
    # --------------------------------------------------------

    draw_debug(
        img,
        target,
        distance
    )
