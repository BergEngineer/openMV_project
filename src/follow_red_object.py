# ------------------------------------------------------------------------------------------------
# PROGRAMMA: ROBOT SEGUE LA PALLINA ROSSA E SE LA PRENDE SI FERMA E BUZZER
# ------------------------------------------------------------------------------------------------
import sensor
import time
from pyb import Pin, Timer, ADC
import pyb

sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.VGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time=2000)  # Wait for settings take effect.
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.set_auto_exposure(False, exposure_us=50000)
clock = time.clock()  # Create a clock object to track the FPS.
proximity = ADC(Pin('P6', Pin.IN))
balle = Pin('P8', Pin.IN)
buzzer = Pin('P9', Pin.OUT_PP)

# ------------------------------------------------------------
# COLOR THRESHOLDS
#   Format: (L_min, L_max, A_min, A_max, B_min, B_max)
#   IMPORTANT! Set threshold before using
#               "Tools > Machine Vision > Threshold Editor" in OpenMV IDE
# ------------------------------------------------------------
red = (0, 70, 23, 127, -19, 53)   # red — A +
green = (30, 100, -64, -8, -32, 32)   # green  — A -
yellow = (50, 100, -20, 20, 30, 100)  # yellow — B +, L +
white = (75, 100, -15, 15, -15,  15)   
# ------------------------------------------------------------
# CONSTANTS TO DETECT A BALL 
# ------------------------------------------------------------
BALL_PIXELS_MIN = 80     # area minima pixel per considerare un blob palla
BALL_PIXELS_MAX = 8000   # area massima (palla troppo grande = falso positivo)
BALL_ROUNDNESS = 0.35   # elongation minima (1.0 = cerchio perfetto, abbassare se necessario)
# ------------------------------------------------------------
# FRAME CENTRE
# ------------------------------------------------------------
CX = sensor.width()//2   # 160
CY = sensor.height()//2   # 120
# ------------------------------------------------------------
# MOVEMENT FUNCIONS
# ------------------------------------------------------------
EN = Pin("P7", Pin.OUT_PP)
EN.low()

# PWM su P4 (M12) e P5 (M11)
# P4 = B10 → TIM2 CH2
# P5 = B11 → TIM2 CH3
tim = Timer(2, freq=1000)
M12 = tim.channel(4, Timer.PWM, pin=Pin("P4"))  # B10
M11 = tim.channel(3, Timer.PWM, pin=Pin("P5"))  # B11


def FORWARD():
    M11.pulse_width_percent(0)
    M12.pulse_width_percent(0)


def LEFT():
    M11.pulse_width_percent(50)
    M12.pulse_width_percent(100)


def RIGHT():
    M11.pulse_width_percent(100)
    M12.pulse_width_percent(50)


def FIND_BIGGEST_RED_OBJECT(img, red):

    img.draw_cross(CX, CY, color=(0, 0, 255), size=12, thickness=2)

    red_blobs = img.find_blobs([red], pixel_threshold=200, area_threshold=200)

    for blob in red_blobs:
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 0))
        if len(red_blobs) > 0:
            big_red = max(red_blobs, key=lambda x: x.pixels())

            if (big_red in red_blobs):
                distXred = (CX - big_red.cx())
                distYred = (CY - big_red.cy())
                img.draw_string(big_red.x(), big_red.y()-10, "dist CX=%.0f" % distXred, color=(255, 220, 0), scale=2)
                img.draw_string(big_red.x(), big_red.y()+70, "dist CY=%.0f" % distYred, color=(255, 220, 0), scale=2)
            return big_red
        else:
            return None


while True:
    clock.tick()  # Update the FPS clock.
    img = sensor.snapshot()  # Take a picture and return the image.

    big_red = FIND_BIGGEST_RED_OBJECT(img, red)

    if (big_red is not None):
        if (CX-big_red.cx() < 80):
            RIGHT()
            print('i go right')
        if (CX-big_red.cx() > 80):
            LEFT()
            print('i go left')
        else:
            FORWARD()
                                 
    else:
        RIGHT()
        print('looking for a red object')
            
  
           













