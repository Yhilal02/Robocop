import RPi.GPIO as GPIO
import time

from gpiozero import PWMOutputDevice, DigitalOutputDevice


from GPIOSetup import SERVO_PIN, BACKMOTOR_PWM, MOTOR_INPUT1, MOTOR_INPUT2
from GPIOSetup import ChassisQueue, ChassisQueuePreset


class MotorController:
    def __init__(self, pwm_pin, input1_pin, input2_pin):
        # Setup PWM output
        self.pwm_device = PWMOutputDevice(pwm_pin, frequency=100)  # 100 Hz frequency
        # Direction control pins
        self.input1 = DigitalOutputDevice(input1_pin)
        self.input2 = DigitalOutputDevice(input2_pin)

    def set_motor_speed(self, speed):
        if 0 <= speed <= 100:
            self.pwm_device.value = speed / 100.0  # Convert to 0 to 1 scale for PWM
            print(f"Motor speed set to: {speed}%")
        else:
            print("Invalid speed. Please enter a value between 0 and 100.")

    def set_direction(self, direction):
        if direction == 'forward':
            self.input1.on()
            self.input2.off()
        elif direction == 'backward':
            self.input1.off()
            self.input2.on()
        else:
            print("Invalid direction. Use 'forward' or 'backward'.")

    def motor_control(self, speedUser):

        desired_speed = int(speedUser)
        
        if speedUser >= 0:
            direction_input = 'forward'
        if speedUser < 0:
            direction_input = 'backward'

        desired_speed = abs(desired_speed) 
        self.set_motor_speed(desired_speed) 
        self.set_direction(direction_input)

    def cleanup(self):
        self.pwm_device.off()  # Turn off the motor
        self.input1.off()
        self.input2.off()
        print("Motor stopped and cleaned up.")

# add setters for step_delay and step_size
class ServoController:
    def __init__(self, 
                 servo_pin, 
                 step_delay=0.01, 
                 step_size=1):
        self.servo_pin     = servo_pin
        self.step_delay    = step_delay
        self.step_size     = step_size
        self.current_angle = 0
        self.setup()

    def setup(self):
        #GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm_servo = GPIO.PWM(self.servo_pin, 50)  # 20ms period (50Hz)
        self.pwm_servo.start(0)
        self.set_angle(self.current_angle)             # Set to initial angle

    def angle_to_duty_cycle(self, angle):
        # Convert angle (-90 to 90) to duty cycle (2.5 to 12.5)
        return 2.5 + 10 * ((angle + 90) / 180)

    def set_angle(self, angle):
        # User can input angle from -90 to 90
        # The function will move the servo to the desired angle smoothly
        while self.current_angle != angle:
            if self.current_angle < angle:
                if self.current_angle + self.step_size > angle:
                    self.current_angle = angle
                else:
                    self.current_angle += self.step_size
            else:
                if self.current_angle - self.step_size < angle:
                    self.current_angle = angle
                else:
                    self.current_angle -= self.step_size
            duty_cycle = self.angle_to_duty_cycle(self.current_angle)
            self.pwm_servo.ChangeDutyCycle(duty_cycle)
            time.sleep(self.step_delay)

    def reset_to_center(self):
        self.set_angle(0)
    
    def sweep_servo(self):
        self.set_angle(-90)
        self.set_angle(90)
        self.set_angle(-90)
        self.set_angle(0)

    def cleanup(self):
        self.pwm_servo.stop()
        GPIO.cleanup()


def ChassisThread():
    print("ChassisThread Started")
    steerControl = ServoController(SERVO_PIN)
    backControl = MotorController(BACKMOTOR_PWM, MOTOR_INPUT1, MOTOR_INPUT2)
    
    while True:
        chassisCmd = ChassisQueue.get()

        if chassisCmd == ChassisQueuePreset.ExitThread:
            backControl.cleanup()
            steerControl.cleanup()
            break
        elif chassisCmd == ChassisQueuePreset.RemoteAction:
            steerControl.set_angle(ChassisQueuePreset.SteeringAngle)
            backControl.motor_control(ChassisQueuePreset.MotorSpeed)
            print(ChassisQueuePreset.SteeringAngle)
            print(ChassisQueuePreset.MotorSpeed)
    
    print("ChassisThread Exited")
    

    










""" if __name__ == '__main__':
    steerControl = ServoController(SERVO_PIN)
    backControl = MotorController(BACKMOTOR_PWM, MOTOR_INPUT1, MOTOR_INPUT2)
    update_value()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        root.quit()
        steerControl.cleanup()
        backControl.cleanup() """
