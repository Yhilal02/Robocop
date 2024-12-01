import threading
from time import sleep
import queue
import RPi.GPIO as GPIO
import time

from GPIOSetup import ChassisQueue, ChassisQueuePreset
from ChassisControlThread import ChassisThread

from WebServer import WebServerControllerThread, WebServerQueue

from tkinter import *
from tkinter import Tk, Scale, HORIZONTAL



#Creating threads
threadChassis = threading.Thread(target=ChassisThread)
threadWebServerController = threading.Thread(target=WebServerControllerThread)


#Starting threads
def StartThreads():
    threadChassis.start()
    threadWebServerController.start()
    sleep(1)
    WebServerQueue.put(1)

#Stopping threads
def StopThreads():
    #global uiThreadShutdown
    #uiThreadShutdown = True
    ChassisQueue.put(ChassisQueuePreset.ExitThread)
    WebServerQueue.put(2)
    sleep(1)
    WebServerQueue.put(0)
    window.quit()


def cleanupAll():
    StopThreads()

#Pop-Up
###############################################################
window = Tk()
window.geometry("800x480+0+0")
window.configure(background='white')

# Create vertical scale
vertical = Scale(window, from_=-100, to=100)
#vertical.pack()
vertical.pack(side='left', fill='y', padx=10, pady=10)

# Create horizontal scale
horizontal = Scale(window, from_=-90, to=90, orient=HORIZONTAL)
#horizontal.pack()
horizontal.pack(side='top', fill='x', expand=True, padx=10, pady=10)
# Add another button that calls a custom function when clicked
button2 = Button(window, text="End", command=cleanupAll)
button2.pack()
###############################################################


def update_value():
    ChassisQueuePreset.SteeringAngle = horizontal.get()
    ChassisQueuePreset.MotorSpeed = vertical.get()
    #print(f"Horizontal Scale Value: {steeringAngle}")
    #print(f"Vertical Scale Value: {speedInput}")
    ChassisQueue.put(ChassisQueuePreset.RemoteAction)

    #steerControl.set_angle(steeringAngle)
    #backControl.motor_control(speedInput)
    # Call this function again after 2s
    window.after(2000, update_value)

#############################
#Main
#############################
StartThreads()
update_value()


try:
    window.mainloop()
except KeyboardInterrupt:
        print("KeyboardInterrupt")
        StopThreads()
        #root.quit()