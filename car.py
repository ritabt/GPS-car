# last edited 20190911-00h18 by Ha Tran

# setup imports
import time, board, pulseio, busio, adafruit_gps, adafruit_lsm9ds1
from adafruit_motor import servo

# other imports
import math as m


# TODO: make class wrapper for car nav & ctrl suite
def main():
	last_print = time.monotonic()
	while True:
			# ------------ magnetometer ------------
	    # Read acceleration, magnetometer, gyroscope, temperature.
	    # accel_x, accel_y, accel_z = sensor.acceleration
	    mag_x, mag_y, mag_z = sensor.magnetic
	    # gyro_x, gyro_y, gyro_z = sensor.gyro
	    # temp = sensor.temperature
	    # print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
	    print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
	    # print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
	    # print('Temperature: {0:0.3f}C'.format(temp))
	    # Delay for a second.
	    time.sleep(1.0)


	    # ------------ GPS ------------
	    # Make sure to call gps.update() every loop iteration and at least twice
	    # as fast as data comes from the GPS unit (usually every second).
	    # This returns a bool that's true if it parsed new data (you can ignore it
	    # though if you don't care and instead look at the has_fix property).
	    gps.update()
	    # Every second print out current location details if there's a fix.
	    current = time.monotonic()
	    if current - last_print >= 1.0:
	        last_print = current
	        if not gps.has_fix:
	            # Try again if we don't have a fix yet.
	            print('Waiting for fix...')
	            continue
	        # We have a fix! (gps.has_fix is true)
	        # Print out details about the fix like location, date, etc.
	        print('=' * 40)  # Print a separator line.
	        print('Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}'.format(
	            gps.timestamp_utc.tm_mon,   # Grab parts of the time from the
	            gps.timestamp_utc.tm_mday,  # struct_time object that holds
	            gps.timestamp_utc.tm_year,  # the fix time.  Note you might
	            gps.timestamp_utc.tm_hour,  # not get all data like year, day,
	            gps.timestamp_utc.tm_min,   # month!
	            gps.timestamp_utc.tm_sec))
	        print('Latitude: {0:.6f} degrees'.format(gps.latitude))
	        print('Longitude: {0:.6f} degrees'.format(gps.longitude))
	        print('Fix quality: {}'.format(gps.fix_quality))
	        # Some attributes beyond latitude, longitude and timestamp are optional
	        # and might not be present.  Check if they're None before trying to use!
	        if gps.satellites is not None:
	            print('# satellites: {}'.format(gps.satellites))
	        if gps.altitude_m is not None:
	            print('Altitude: {} meters'.format(gps.altitude_m))
	        if gps.speed_knots is not None:
	            print('Speed: {} knots'.format(gps.speed_knots))
	        if gps.track_angle_deg is not None:
	            print('Track angle: {} degrees'.format(gps.track_angle_deg))
	        if gps.horizontal_dilution is not None:
	            print('Horizontal dilution: {}'.format(gps.horizontal_dilution))
	        if gps.height_geoid is not None:
	            print('Height geo ID: {} meters'.format(gps.height_geoid))


	# We sleep in the loops to give the servo time to move into position.
	print("Sweep from 0 to 180")
	for i in range(180):
	    servo.angle = i
	    time.sleep(0.01)
	print("Sweep from 180 to 0")
	for i in range(180):
	    servo.angle = 180 - i
	    time.sleep(0.01)

	print("Move to 90 degrees")
	servo.angle = 90
	time.sleep(1)
	print("Release servo motor for 1 seconds")
	servo.fraction = None
	time.sleep(1)

	# You can also specify the movement fractionally.
	print("Sweep from 0 to 1.0 fractionally")
	fraction = 0.0
	while fraction < 1.0:
	    servo.fraction = fraction
	    fraction += 0.01
	    time.sleep(0.01)


def setup():
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
	pwm1 = pulseio.PWMOut(board.D42, duty_cycle=0, frequency=50)
	pwm2 = pulseio.PWMOut(board.D41, duty_cycle=0, frequency=50)

	# pulse widths exercise the full range of the 169 servo, other servos are different
	servo1 = servo.Servo(pwm1, min_pulse=500, max_pulse=2500)
	servo2 = servo.Servo(pwm2, min_pulse=500, max_pulse=2500)

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


# move to a given destination Longitude and Latitude
def moveToCoord(lat2, lon2):
	lat1 = gps.latitude
	lon1 = gps.longitude
	dist = coordDist(lat1, lon1, lat2, lon2)
	relativeBearing = heading() + bearing() # angle in degrees CW from heading to destination
		
	# TODO: turns + move --> smoother transitions
	def turn(theta):
		# v1 turn in-place
		servo1.angle = theta / 2
		servo2.angle = theta / 2

	def move(dist):
		wheelDiam = 0.05 # meters
		theta = dist / (m.pi * wheelDiam)
		servo1.angle = servo2.angle = theta

	turn(relativeBearing)
	move(dist)

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

# orthodromic distance between two lon/lat coordinates
def coordDist(lat1, lon1, lat2, lon2):
	# Haversine formula
	# REF: https://en.wikipedia.org/wiki/Haversine_formula

	r = 6371000 # earth radius in meters
	phi1, phi2, lambda1, lambda2 = map(m.radians, [lat1, lat2, lon1, lon2])

	def hav(theta):
		# TODO: check floating point accuracy on mathematically equivalent trig operations
		# yield slightly different results
		return  (1 - m.cos(theta)) / 2 # OR m.sin(theta/2) **2
		
	try:
		h = hav(phi2 - phi1) + m.cos(phi1) * m.cos(phi2) * hav(lambda2 - lambda1)
	except ValueError:
		# h approaches 1 for antipodal points,
		# might exceed 1 due to floating point rounding error
		# --> use Vincenty's formulae for higher accuracy (esp at antipodal points)
		# 		at the expense of computational cost
		return 2 * r
	return 2 * r * m.asin(m.sqrt(h))
# -----------------------------------------------------------------

setup()
main()