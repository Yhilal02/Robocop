from tkinter import *

# Create root window
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

# Function to print the current scale values when the button is clicked
def print_values():
    vertical_value = vertical.get()
    horizontal_value = horizontal.get()
    print(f"Vertical Scale Value: {vertical_value}, Horizontal Scale Value: {horizontal_value}")

# Your custom function to be called by the second button
def custom_function():
    print("Custom function executed!")
    window.quit()

# Add a button that calls print_values when clicked
button1 = Button(window, text="Print Scale Values", command=print_values)
button1.pack()

# Add another button that calls a custom function when clicked
button2 = Button(window, text="End", command=custom_function)
button2.pack()



def update_value():
    steeringAngle = horizontal.get()
    speedInput = vertical.get()
    #print(f"Horizontal Scale Value: {steeringAngle}")
    #print(f"Vertical Scale Value: {speedInput}")
    #steerControl.set_angle(steeringAngle)
    #backControl.motor_control(speedInput)
    # Call this function again after 100ms
    window.after(100, update_value)


if __name__ == '__main__':
# Start the GUI event loop
    update_value()
    
    try:
        window.mainloop()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        window.quit()
