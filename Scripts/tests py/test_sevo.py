import RPi.GPIO as GPIO
import time

servo_pin_rz = 12
servo_pin_ry = 13
servo_pin_rx = 19

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(servo_pin_rz, GPIO.OUT)
GPIO.setup(servo_pin_ry, GPIO.OUT)
GPIO.setup(servo_pin_rx, GPIO.OUT)


p_rz = GPIO.PWM(servo_pin_rz , 50)
p_ry = GPIO.PWM(servo_pin_ry , 50)
p_rx = GPIO.PWM(servo_pin_rx , 50)

p_ry.start(0)
p_rz.start(0)
p_rx.start(0)

def set_angle_rz(angle):
    duty = angle/18 + 2
    GPIO.output(servo_pin_rz, True)
    p_rz.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(servo_pin_rz, False)
    p_rz.ChangeDutyCycle(0)
    
def set_angle_ry(angle):
    duty = angle/18 + 2
    GPIO.output(servo_pin_ry, True)
    p_ry.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(servo_pin_ry, False)
    p_ry.ChangeDutyCycle(0)
    
def set_angle_rx(angle):
    duty = angle/18 + 2
    GPIO.output(servo_pin_rx, True)
    p_rx.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(servo_pin_rx, False)
    p_rx.ChangeDutyCycle(0)

#set_angle_rz(90) # links
#set_angle_rz(60) # midden
#set_angle_rz(120) #rechts

#set_angle_ry(0)
#set_angle_ry(90)
#set_angle_ry(180)

set_angle_rx(90)

p_rx.stop()
p_ry.stop()
p_rz.stop()

GPIO.cleanup()
