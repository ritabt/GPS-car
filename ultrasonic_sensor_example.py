import time, board, pulseio, busio, adafruit_gps, adafruit_lsm9ds1
#from adafruit_hcsr04 import HCSR04
import hcsr04_lib

trig = board.D59
echo = board.D60
sonar = hcsr04_lib.HCSR04(trig, echo)
try:
    while True:
        print(sonar.dist_cm())
        time.sleep(1)
except KeyboardInterrupt:
    pass
sonar.deinit()



