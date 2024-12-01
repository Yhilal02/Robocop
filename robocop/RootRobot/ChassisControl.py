import RPi.GPIO as GPIO
import time

from gpiozero import PWMOutputDevice, DigitalOutputDevice

from tkinter import *
from tkinter import Tk, Scale, HORIZONTAL


from GPIOSetup import SERVO_PIN, BACKMOTOR_PWM, MOTOR_INPUT1, MOTOR_INPUT2



root = Tk()

# Create vertical scale
vertical = Scale(root, from_=-100, to=100)
vertical.pack()

# Create horizontal scale
horizontal = Scale(root, from_=-90, to=90, orient=HORIZONTAL)
horizontal.pack()

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
        GPIO.setmode(GPIO.BCM)
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


def update_value():
    steeringAngle = horizontal.get()
    speedInput = vertical.get()
    print(f"Horizontal Scale Value: {steeringAngle}")
    print(f"Vertical Scale Value: {speedInput}")
    steerControl.set_angle(steeringAngle)
    backControl.motor_control(speedInput)
    # Call this function again after 100ms
    root.after(100, update_value)


def print_values():
    vertical_value = vertical.get()
    horizontal_value = horizontal.get()
    print(f"Vertical Scale Value: {vertical_value}, Horizontal Scale Value: {horizontal_value}")


def cleanupAll():
    print("Cleanup")
    root.quit()
    steerControl.cleanup()
    backControl.cleanup()

# Add a button that calls print_values when clicked
button1 = Button(root, text="Print Scale Values", command=print_values)
button1.pack()

# Add another button that calls a custom function when clicked
button2 = Button(root, text="End", command=cleanupAll)
button2.pack()


if __name__ == '__main__':
    steerControl = ServoController(SERVO_PIN)
    backControl = MotorController(BACKMOTOR_PWM, MOTOR_INPUT1, MOTOR_INPUT2)
    update_value()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        root.quit()
        steerControl.cleanup()
        backControl.cleanup()
    
    
    """ try:
        while True:
            command = input("Enter the angle (-90 to 90) or 'sweep' to sweep the servo: ")
            if command.lower() == 'sweep':
                controller.sweep_servo()
            else:
                try:
                    angle = float(command)
                    if -90 <= angle <= 90:
                        controller.set_angle(angle)
                    else:
                        print("Please enter a valid angle between -90 and 90.")
                except ValueError:
                    print("Invalid input. Please enter a numeric value or 'sweep'.")
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup() """
