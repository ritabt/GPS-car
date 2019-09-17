
import time, board, pulseio, busio, adafruit_gps, adafruit_lsm9ds1, hcsr04_lib
from adafruit_motor import servo
import math as m

# -------- tweak parameters -----------
refreshRate = 0.1
lostSignalTimer = 5.0
sonarSensitivityFront = 50.0 # obstacle distance in cm before avoiding
defaultMoveSpeed = 0.7
defaultTurnSpeed = 0.

# -------- set up ultrasonic sensor -----------
trig1 = board.D
echo1 = board.D60
sonar_front = hcsr04_lib.HCSR04(trig1, echo1)

# -------- set up servos -----------

# dc motor
motor_pin = board.D31
pwm = pulseio.PWMOut(motor_pin, frequency=50)
servo_dc = servo.ContinuousServo(pwm)

# steering servo
servo_pin = board.D42
pwm1 = pulseio.PWMOut(servo_pin, duty_cycle=0, frequency=50)
servo_turn = servo.Servo(pwm1, min_pulse=500, max_pulse=2500)

# # create a PWMOut object on the control pin.
# pwm1 = pulseio.PWMOut(board.D31, duty_cycle=0, frequency=50)
# pwm2 = pulseio.PWMOut(board.D36, duty_cycle=0, frequency=50)

# # pulse widths exercise the full range of the 169 servo, other servos are different
# servo1 = servo.ContinuousServo(pwm1, min_pulse=500, max_pulse=2500)
# servo2 = servo.ContinuousServo(pwm2, min_pulse=500, max_pulse=2500)

# -------- set up GPS -----------
# Define RX and TX pins for the board's serial port connected to the GPS.
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
RX = board.RX1
TX = board.TX1


# Create a serial connection for the GPS connection using default speed and
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
# You can also speed up the rate, but don't go too fast or else you can lose
# data during parsing.  This would be twice a second (2hz, 500ms delay):
#gps.send_command(b'PMTK220,500')

# initialize gps to give it time to get the correct current coordinates


# Main loop runs forever printing the location, etc. every second.

# ---------------- copied ------------------
# startTime = time.monotonic()
# last_print = time.monotonic()
# while last_print - startTime < 3.0:
#     # Make sure to call gps.update() every loop iteration and at least twice
#     # as fast as data comes from the GPS unit (usually every second).
#     # This returns a bool that's true if it parsed new data (you can ignore it
#     # though if you don't care and instead look at the has_fix property).
#     gps.update()
#     # Every second print out current location details if there's a fix.
#     current = time.monotonic()
#     if current - last_print >= 0.1:
#         print("Setting up...")
#         last_print = current
#         if not gps.has_fix:
#             # Try again if we don't have a fix yet.
#             print('Waiting for fix...')
#             continue
#         # We have a fix! (gps.has_fix is true)
#         # Print out details about the fix like location, date, etc.
#         print('=' * 40)  # Print a separator line.
#         print('Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}'.format(
#             gps.timestamp_utc.tm_mon,   # Grab parts of the time from the
#             gps.timestamp_utc.tm_mday,  # struct_time object that holds
#             gps.timestamp_utc.tm_year,  # the fix time.  Note you might
#             gps.timestamp_utc.tm_hour,  # not get all data like year, day,
#             gps.timestamp_utc.tm_min,   # month!
#             gps.timestamp_utc.tm_sec))
#         print('Latitude: {0:.6f} degrees'.format(gps.latitude))
#         print('Longitude: {0:.6f} degrees'.format(gps.longitude))

class ComplementaryFilter:
    def __init__(self, K):
        self.K = K
        self.lastGyroTime = None
        self.theta = None
        
    def update_magn(self, heading):
        self.theta = heading if not self.theta else self.k * self.theta + (1 - self.k) * heading

    def update_gyro(self, omega):
        if not self.lastGyroTime:
            self.lastGyroTime = time()
            return
        if not self.lastSin or not self.lastCos:
            return

        dt = time() - self.lastGyroTime
        self.theta += omega * dt

        self.lastGyroTime = time()


# class Car:
#     def __init__(self):

def main():
    while True:
        move(1.0)

def moveToCoord(lat2, lon2):
    # refreshRate = 0.1
    print("moving to " + str(lat2) + ", " + str(lon2))
    startTime = time.monotonic()

    lat1 = lon1 = None

    last_print = time.monotonic()
    last_signal = time.monotonic()
    dlat = dlon = 1.0
    while True:
        gps.update()
        current = time.monotonic()
        if current - last_print >= refreshRate:
            last_print = current

            # if not gps.has_fix:
                # # Try again if we don't have a fix yet.
                # print('Waiting for fix...' + str(current - startTime))
                # continue

            if gps.has_fix:
                last_signal = time.monotonic()
                # Print out details about the fix like location, date, etc.
                print('=' * 40)  # Print a separator line.
                print('Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}'.format(
                    gps.timestamp_utc.tm_mon,   # Grab parts of the time from the
                    gps.timestamp_utc.tm_mday,  # struct_time object that holds
                    gps.timestamp_utc.tm_year,  # the fix time.  Note you might
                    gps.timestamp_utc.tm_hour,  # not get all data like year, day,
                    gps.timestamp_utc.tm_min,   # month!
                    gps.timestamp_utc.tm_sec))
                print('Latitude: {0:.10f} degrees'.format(gps.latitude))
                print('Longitude: {0:.10f} degrees'.format(gps.longitude))
                # error: ValueError: unknown format code 'f' for object of type 'str'
                # when lat and lon are unintialized, value = None
                if gps.satellites is not None:
                    print('# satellites: {}'.format(gps.satellites))
                lat1 = gps.latitude
                lon1 = gps.longitude
                dlat = (lat1 - lat2)
                dlon = (lon1 - lon2)
                print((dlat, dlon)) # if dist > 1.0 else move(1.0)

                if (abs(dlat) > 0.0001 or abs(dlon) > 0.0001):
                    move(1.0)
                else:
                    stop()

                # TODO: turns + move --> smoother transitions
                relativeBearing = bearing(lat1, lon1, lat2, lon2) - heading() # angle in degrees CW from heading to destination
                turn(relativeBearing() / 180) # want relativeBearing to oscillate around 0
                print(bearing())
                dist = coordDist(lat1, lon1, lat2, lon2)

            else:
                move()
                # lostSignalTimer = 5.0
                # if time.monotonic() - last_signal > lostSignalTimer:
                #     # if lost signal, stop after 5 sec timer
                #     stop()

        # check for obstacles
        if sonar_front.dist_cm() < sonarSensitivityFront:
            stop()
            # turn(defaultTurnSpeed)
    stop()

def setV(v):
    if v < 0:
        return v if v >= -1.0 else -1.0
    return v if v <= 1.0 else 1.0

def stop():
    servo_dc.throttle = 0.0

def move(v=defaultMoveSpeed, theta=90):
    my_servo.throttle = v
    servo_turn.angle = theta - 90

# angle in degrees CW from North to direction car is pointing
def heading():
    mag_x, mag_y, mag_z = sensor.magnetic
    return m.degrees(m.atan(mag_x / mag_y))

# angle in degrees CW from North to destination
def bearing(lat1, lon1, lat2, lon2):
    # REF: https://www.movable-type.co.uk/scripts/latlong.html
    phi1, phi2, lambda1, lambda2 = map(m.radians, [lat1, lat2, lon1, lon2])
    y = m.sin(lambda2 - lambda1) * m.cos(phi2)
    x = m.cos(phi1) * m.sin(phi2) \
        - m.sin(phi1) * m.cos(phi2) \
        * m.cos(phi2 - phi1)
    return m.degrees(m.atan2(y, x))

def deinit():
    sonar.deinit()

main()
deinit()