import struct
import os
import sys
import signal
import datetime
import time
import sys
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import joystick_t

# Define a function to handle the SIGINT signal (i.e. Ctrl-C)
def signal_handler(sig, frame):
    print('Exiting program...')
    sys.exit(0)

# Register the signal handler function for the SIGINT signal
signal.signal(signal.SIGINT, signal_handler)

lc = lcm.LCM()

# Open the joystick device file
joystick_dev = open('/dev/input/js1', 'rb')
# os.set_blocking(joystick_dev.fileno(), False)
# Create a list to hold the current state of the joystick
joystick_state = [0] * 24

# Define the axis and button labels for formatting the output
axis_labels = ['LA_X', 'LA_Y', 'RA_X', 'RA_Y', 'R2', 'L2', 'D_X', 'D_Y']
button_labels_1 = ['Btn{}'.format(i) for i in range(8)]
button_labels_2 = ['Btn{}'.format(i) for i in range(8, 16)]

# Read the joystick state and display the values in the terminal
try:
    while True:
        # Read the raw joystick input data
        input_data = joystick_dev.read(8)
        # Unpack the raw input data into usable values
        timestamp, value, event_type, axis = struct.unpack('IhBB', input_data)
        # Determine whether the input is an axis or button event
        if event_type & 0x01:
            # Button event
            joystick_state[axis + 8] = value
        else:
            # Axis event
            joystick_state[axis] = value

        joy_msg = joystick_t()
        joy_msg.timestamp = int(datetime.datetime.now().timestamp() * 1000000)
        joy_msg.left_analog_X = joystick_state[0]/32767.0
        joy_msg.left_analog_Y = joystick_state[1]/32767.0
        joy_msg.right_analog_X = joystick_state[2]/32767.0
        joy_msg.right_analog_Y = joystick_state[3]/32767.0
        joy_msg.right_trigger = joystick_state[4]/32767.0
        joy_msg.left_trigger = joystick_state[5]/32767.0
        joy_msg.dpad_X = joystick_state[6]/32767.0
        joy_msg.dpad_Y = joystick_state[7]/32767.0
        joy_msg.button_A = joystick_state[8]
        joy_msg.button_B = joystick_state[9]
        joy_msg.button_2 = joystick_state[10]
        joy_msg.button_X = joystick_state[11]
        joy_msg.button_Y = joystick_state[12]
        joy_msg.button_5 = joystick_state[13]
        joy_msg.button_l1 = joystick_state[14]
        joy_msg.button_r1 = joystick_state[15]
        joy_msg.button_l2 = joystick_state[16]
        joy_msg.button_r2 = joystick_state[17]
        joy_msg.button_select = joystick_state[18]
        joy_msg.button_start = joystick_state[19]
        joy_msg.button_12 = joystick_state[20]
        joy_msg.button_left_analog = joystick_state[21]
        joy_msg.button_right_analog = joystick_state[22]
        joy_msg.button_15 = joystick_state[23]
        lc.publish("MBOT_JOYSTICK_COMMAND",joy_msg.encode())
        # Print the current joystick state to the terminal
        os.system('clear')
        print('Axis:', end='\t')
        # Print the axis labels
        for label in axis_labels:
            print(label, end='\t')
        print()
        # Print the axis values
        print('Value:', end='\t')
        for i in range(8):
            print(joystick_state[i], end='\t')
        print()
        print('Button:', end=' ')
        # Print the first row of button labels
        for label in button_labels_1:
            print(label, end='\t')
        print()
        # Print the first row of button values
        print('Value:', end='\t')
        for i in range(8):
            print(joystick_state[i+8], end='\t')
        print()
        print('Button:', end=' ')
        # Print the second row of button labels
        for label in button_labels_2:
            print(label, end='\t')
        print()
        # Print the second row of button values
        print('Value:', end='\t')
        for i in range(8, 16):
            print(joystick_state[i+8], end='\t')
        print()
        time.sleep(0.1)

# Catch the KeyboardInterrupt exception and exit the program gracefully
except KeyboardInterrupt:
    print('Exiting program...')
    lc.destroy()
    sys.exit(0)

# Close the joystick device file
joystick.close()
