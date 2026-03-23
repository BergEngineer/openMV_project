import sensor
import image
import math
import time
import pyb 



# ------------------------------------------------------------
# SENSOR SETUP
# ------------------------------------------------------------
sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA (320x240) (30 FPS stabili OK!)
sensor.set_auto_whitebal(False)      # DISABILITARE per colori stabili CRUCIALE! bianco automatico
sensor.set_auto_gain(False)          # DISABILITARE per colori stabili CRUCIALE! luminosita autom.
#sensor.set_auto_exposure(False, exposure_us=30000)  # Esposizione fissa (regola sul campo)
sensor.skip_frames(time=2000)  # Wait for settings take effect. (messo 100ms + fastttt!!)
clock = time.clock()  # Create a clock object to track the FPS.

#-------------------------------------------------------------
# SETTINGS BOARD LED
#-------------------------------------------------------------
red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)  #  pyb.LED.off() / pyb.LED.on()
pyb.LED.off(red_led)
pyb.LED.off(green_led)
pyb.LED.off(blue_led)

# ------------------------------------------------------------
# TRESHOLDS BASIC COLORS
# ------------------------------------------------------------
red = (0, 70, 23, 127, -19, 53)  
green = (30, 100, -64, -8, -32, 32)   
yellow = (50, 100, -20, 20, 30, 100)  
white = (75, 100, -15, 15, -15,  15)   
black = (0,  40,  -20, 20, -20,  20)  
# ------------------------------------------------------------
# BLOB CONST. TO FIND BALLS
# ------------------------------------------------------------
BALL_PIXELS_MIN = 80    
BALL_PIXELS_MAX = 8000   
BALL_ROUNDNESS = 0.35   
# ------------------------------------------------------------
# FRAME CENTER
# ------------------------------------------------------------
CX = sensor.width()//2   # 160
CY = sensor.height()//2   # 120

# ------------------------------------------------------------
# FINDING BIG RED OBJECT AND ITS DISTANCE FROM CENTER
# ------------------------------------------------------------

while True:
    clock.tick()  # Update the FPS clock.
    img = sensor.snapshot()  # Take a picture and return the image.
    sensor.set_vflip(True)
    sensor.set_auto_exposure(True, exposure_us=60000)
    sensor.set_hmirror(True)
    #sensor.set_auto_exposure(True, exposure_us=60000)
    #print(clock.fps())  # Note: OpenMV Cam runs about half as fast when connected
    # to the IDE. The FPS should increase once disconnected.
    img.draw_cross(CX, CY, color=(0, 0, 255), size=12, thickness=5)
    img.draw_line(0, 0, 320, 0, color=(0, 0, 0), thickness=100)
    red_blobs = img.find_blobs([red], pixel_threshold=200, area_threshold=200)
    green_blobs = img.find_blobs([green], pixel_threshold=200, area_threshold=200)
    yellow_blobs = img.find_blobs([yellow], pixel_threshold=200, area_threshold=200)
    # ritorna una lista (list) di blobs
    for blob in red_blobs:
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 0))

        big_red = max(red_blobs, key=lambda x: x.pixels())

        if (big_red in red_blobs):
            distXred = (CX - big_red.cx())
            distYred = (CY - big_red.cy())
            img.draw_string(big_red.x(), big_red.y()-10, "dist CX=%.0f" % distXred, color=(255, 220, 0), scale=2)
            img.draw_string(big_red.x(), big_red.y()+70, "dist CY=%.0f" % distYred, color=(255, 220, 0), scale=2)
        else:
            pass


