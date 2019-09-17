import time
import board
import pulseio
from adafruit_motor import servo

##dc motor
motor_pin = board.D31
pwm = pulseio.PWMOut(motor_pin, frequency=50)
my_servo = servo.ContinuousServo(pwm)

##steering servo
servo_pin = board.D42
pwm1 = pulseio.PWMOut(servo_pin, duty_cycle=0, frequency=50)
servo = servo.Servo(pwm1, min_pulse=500, max_pulse=2500)

def forward():
    my_servo.throttle = 0.1
    return

def stop():
    my_servo.throttle = 0.0
    return

def turn_right():
    servo.angle = 135
    return

def turn_left():
    servo.angle = 45
    return

def go_straight():
    servo.angle = 90
    return

def steering_test():
    while True:
        print("straight")
        go_straight()
        time.sleep(1)
        print("left")
        turn_left()
        time.sleep(1)
        print("right")
        turn_right()
        time.sleep(1)

def motor_test():
    while True:
        print("forward")
        forward()
        time.sleep(2.0)
        print("stop")
        stop()
        time.sleep(2.0)


motor_test()