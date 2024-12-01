import RPi.GPIO as GPIO
import queue

###################################################################
# GPIO Setup
###################################################################
#GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

###################################################################
# Chassis
###################################################################
SERVO_PIN = 23

BACKMOTOR_PWM = 18
MOTOR_INPUT1 = 2
MOTOR_INPUT2 = 3

ChassisQueue = queue.Queue()

#For generating Preset actions
class ChassisQueuePreset():
    test = 1
    ExitThread = 0
    RemoteAction = 2
    
    
    MotorSpeed = 0
    SteeringAngle = 0
