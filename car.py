# setup imports
import time, board, pulseio, busio, adafruit_gps, adafruit_lsm9ds1
from adafruit_motor import servo
from adafruit_hcsr04 import HCSR04

# other imports
import math as m

# ==========================================================================
# -------------------------------- SETUP -----------------------------------
# ==========================================================================
# -------- set up LSM9DS1 accelerometer, magnetometer, gyroscope -----------
# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

#SPI connection:
# from digitalio import DigitalInOut, Direction
# spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
# csag = DigitalInOut(board.D5)
# csag.direction = Direction.OUTPUT
# csag.value = True
# csm = DigitalInOut(board.D6)
# csm.direction = Direction.OUTPUT
# csm.value = True
# sensor = adafruit_lsm9ds1.LSM9DS1_SPI(spi, csag, csm)

# -------- set up servos -----------
# create a PWMOut object on the control pin.
pwm1 = pulseio.PWMOut(board.D31, duty_cycle=0, frequency=50) # left wheel
pwm2 = pulseio.PWMOut(board.D36, duty_cycle=0, frequency=50) # right wheel

# pulse widths exercise the full range of the 169 servo, other servos are different
servo1 = servo.ContinuousServo(pwm1, min_pulse=500, max_pulse=2500)
servo2 = servo.ContinuousServo(pwm2, min_pulse=500, max_pulse=2500)

# -------- set up GPS -----------
# Define RX and TX pins for the board's serial port connected to the GPS.
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
RX = board.RX1
TX = board.TX1

# Create a serial connection for the GPS connection using default veloc and
# a slightly higher timeout (GPS modules typically update once a second).
uart = busio.UART(TX, RX, baudrate=9600, timeout=30)

# for a computer, use the pyserial library for uart access
#import serial
#uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=3000)

# Create a GPS module instance.
gps = adafruit_gps.GPS(uart, debug=False)

# Initialize the GPS module by changing what data it sends and at what rate.
# These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
# PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
# the GPS module behavior:
#   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn on just minimum info (RMC only, location):
#gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn off everything:
#gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Tuen on everything (not all of it is parsed!)
#gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

# Set update rate to once a second (1hz) which is what you typically want.
gps.send_command(b'PMTK220,1000')
# Or decrease to once every two seconds by doubling the millisecond value.
# Be sure to also increase your UART timeout above!
#gps.send_command(b'PMTK220,2000')
# You can also veloc up the rate, but don't go too fast or else you can lose
# data during parsing.  This would be twice a second (2hz, 500ms delay):
#gps.send_command(b'PMTK220,500')

# ------------------- set up ultrasonic sensor --------------------
# sonar = HCSR04(trig, echo)
# try:
#     while True:
#         print(sonar.dist_cm())
#         sleep(2)
# except KeyboardInterrupt:
#     pass
# sonar.deinit()
# ===================================================================
# ----------------------- END SETUP ---------------------------------
# ===================================================================

def main():
    # v1, v2 = move(0.7)
    # print(v1, v2)
    # servo1.throttle = 0.9
    # servo2.throttle = -0.1
    # time.sleep(2.0)
    # stop()
    # time.sleep(0.5)
    # turn(1.6, v1, v2, 2.5)
    # move(1.0, 1.0)
    # stop()
    # time.sleep(1.0)
    driveStraight(0.5, 5.0)

def setV(v):
    if v < 0:
        return v if v >= -1.0 else -1.0
    return v if v <= 1.0 else 1.0

def stop():
    servo1.throttle = 0.0
    servo2.throttle = 0.0

def move(veloc, timer=0.5):
    v1 = veloc
    v2 = -veloc
    servo1.throttle = v1
    servo2.throttle = v2
    return (v1, v2)
    # time.sleep(timer)

# positive -> turn right
def turn(dv, v1, v2, timer=0.5, pivot=False):
    if pivot:
        # one wheel stationary
        servo1.throttle = dv if dv > 0 else 0
        servo2.throttle = dv if dv < 0 else 0
    else:
        # turn both wheels opposite directions
        vv1 = setV(v1 + dv)
        vv2 = setV(v2 + dv)
        servo1.throttle = vv1
        servo2.throttle = vv2
        print(vv1, vv2)
    # time.sleep(timer)

    # # restore original veloc
    # servo1.throttle = setV(v1)
    # servo2.throttle = setV(v2)
    return (vv1, vv2)

# move with drive correction
def driveStraight(v, timer=0.5):
    def fnc(k, x):
        dv = m.log(k * x + 1.0) if x >= 0 else m.log(k * -x + 1.0)
        return setV(dv)
    
    k = 1.0 # how aggressive the correction is
    refreshRate = 0.05
    startTime = time.monotonic()
    elapsedTime = time.monotonic() - startTime
    
    v1, v2 = move(v)
    
    while elapsedTime <= timer:
        accel_x, accel_y, accel_z = sensor.acceleration
        print(accel_x)
        if accel_x:
            # v1, v2 = turn(0.4, v1, v2)
            # turn(-1 * fnc(k, accel_x), refreshRate, pivot=True) if accel_x > 0 else turn(fnc(k, accel_x), refreshRate, pivot=True)
            if accel_x < 0: # veering right
            #     turn(fnc(k, accel_x))
                v1, v2 = turn(-0.1, v1, v2, refreshRate)
            else:
                v1, v2 = turn(0.1, v1, v2, refreshRate)
            #     turn(-1 *fnc(k, accel_x))
        time.sleep(refreshRate)

        elapsedTime = time.monotonic() - startTime
    stop()

main()