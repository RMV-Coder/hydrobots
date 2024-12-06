## l298_driver_wasd.py
import gpiod
import sys
import tty
import termios
import asyncio
### ultrasonic
from gpiozero import DistanceSensor
from time import sleep

#pin

#3V3 POWER = 
#GPIO2 = 
#GPIO3 = 
#GPIO4 = 
#GROUND = 
ENA = 17  #GPIO17 = PWM for Motor A
IN1 = 27  #GPIO27 = back left forward
IN2 = 22   #GPIO22 = back left reverse
#3V3 POWER = 
#GPIO10(MOSI)
#GPIO9(MISO)
ENC = 11  # PWM for Motor C GPIO11 (SCLK) 
#GROUND
#GPIO0(ID_SD)
RELAY_PIN = 5 #GPIO5 = WATER PUMP
IN5 = 6    #GPIO6 front right forward
IN6 = 13   #GPIO13(PWM1) front right reverse
IN7 = 19  #GPIO19(PCM_FS) front left forward
LINE = 26  #GPIO26 WATER LEVEL SENSOR
#GROUND
#GPIO5V
#GPIO5V
#GROUND
#GPIO14(TXD)
#GPIO15(RXD)
#GPIO18(PCM_CLK)
#GROUND
ENB = 23  # PWM for Motor B #GPIO23
IN3 = 24  # back right forward#GPIO24
#GROUND
IN4 = 25 # back right reverse #GPIO25
#GPIO8(CE0)
#GPIO7(CE1)
#GPIO1(ID_SC)
#GROUND
IN8 = 12  #GPIO12 Front left reverse
#GROUND
END = 16  # PWM for Motor D #GPIO16
echo=20 #GPIO20(PCM_DIN)
trigger=21 #GPIO21(PCM_DOUT)










# Initialize ultrasonic sensor
sensor1 = DistanceSensor(echo=echo, trigger=trigger)  # Sensor 1

try:
    while True:
        # Read distances from each sensor
        distance1 = sensor1.distance * 100  # Convert to cm
        
        # Print the distances
        print(f"Sensor 1: {distance1:.2f} cm")
        print("-" * 30)
        
        sleep(0.5)

except KeyboardInterrupt:
    print("Measurement stopped by User")

# Open the GPIO chip (0 refers to the first GPIO chip, usually the Raspberry Pi's GPIO)
chip = gpiod.Chip('gpiochip0')

# Set up the GPIO lines for the motors
ena_line = chip.get_line(ENA)
in1_line = chip.get_line(IN1)
in2_line = chip.get_line(IN2)
enb_line = chip.get_line(ENB)
in3_line = chip.get_line(IN3)
in4_line = chip.get_line(IN4)
enc_line = chip.get_line(ENC)
in5_line = chip.get_line(IN5)
in6_line = chip.get_line(IN6)
end_line = chip.get_line(END)
in7_line = chip.get_line(IN7)
in8_line = chip.get_line(IN8)

# Set the GPIO lines as outputs
lines = [ena_line, in1_line, in2_line, enb_line, in3_line, in4_line, enc_line, in5_line, in6_line, end_line, in7_line, in8_line]
for line in lines:
    line.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)

# Function to move all motors forward
def forward():
    ena_line.set_value(1)  # Enable motor A
    enb_line.set_value(0)  # Enable motor B
    enc_line.set_value(1)  # Enable motor C
    end_line.set_value(0)  # Enable motor D

    # Motors A & B forward
    in1_line.set_value(0)
    in2_line.set_value(1)
    
    # B
    in3_line.set_value(0)
    in4_line.set_value(1)

    # Motors C & D forward
    in5_line.set_value(0)
    in6_line.set_value(1)
    # D
    in7_line.set_value(0)
    in8_line.set_value(1)

# Function to move all motors backward
def backward():
    ena_line.set_value(1)
    enb_line.set_value(0)
    enc_line.set_value(0)
    end_line.set_value(0)

    # Motors A & B backward
    in1_line.set_value(1)
    in2_line.set_value(0)
    # B
    in3_line.set_value(1)
    in4_line.set_value(0)

    # Motors C & D backward
    # C
    in5_line.set_value(1)
    in6_line.set_value(0)
    # D
    in7_line.set_value(1)
    in8_line.set_value(0)


def right():
    ena_line.set_value(1)  # Enable motor A
    enb_line.set_value(0)  # Enable motor B
    enc_line.set_value(0)  # Enable motor C
    end_line.set_value(0)  # Enable motor D

    # Motors A & B right
    in1_line.set_value(1)
    in2_line.set_value(0)
    
    in3_line.set_value(0)
    in4_line.set_value(1)

    # Motors C & D right
    in5_line.set_value(1)
    in6_line.set_value(0)

    in7_line.set_value(0)
    in8_line.set_value(1)


# def left
def left():
    ena_line.set_value(1)
    enb_line.set_value(0)
    enc_line.set_value(1)
    end_line.set_value(0)

     # Motors A & B left
    in1_line.set_value(0)
    in2_line.set_value(1)
    
    in3_line.set_value(1)
    in4_line.set_value(0)

    # Motors C & D left
    in5_line.set_value(0)
    in6_line.set_value(1)

    in7_line.set_value(1)
    in8_line.set_value(0)

def diagonal_forward_right():
    ena_line.set_value(0)
    enb_line.set_value(1)
    enc_line.set_value(0)
    end_line.set_value(1)

     # Motors A & B left
    in1_line.set_value(0)
    in2_line.set_value(1)
    
    in3_line.set_value(0)
    in4_line.set_value(1)

    # Motors C & D left
    in5_line.set_value(0)
    in6_line.set_value(1)

    in7_line.set_value(0)
    in8_line.set_value(1)

def diagonal_forward_left():
    ena_line.set_value(1)
    enb_line.set_value(0)
    enc_line.set_value(1)
    end_line.set_value(0)

     # Motors A & B left
    in1_line.set_value(0)
    in2_line.set_value(1)
    
    in3_line.set_value(0)
    in4_line.set_value(1)

    # Motors C & D left
    in5_line.set_value(0)
    in6_line.set_value(1)

    in7_line.set_value(0)
    in8_line.set_value(1)

def diagonal_backward_left():
    ena_line.set_value(0)
    enb_line.set_value(1)
    enc_line.set_value(0)
    end_line.set_value(1)

     # Motors A & B left
    in1_line.set_value(0)
    in2_line.set_value(1)
    
    in3_line.set_value(0)
    in4_line.set_value(1)

    # Motors C & D left
    in5_line.set_value(0)
    in6_line.set_value(1)

    in7_line.set_value(1)
    in8_line.set_value(0)

def diagonal_backward_right():
    ena_line.set_value(1)
    enb_line.set_value(0)
    enc_line.set_value(1)
    end_line.set_value(0)

     # Motors A & B left
    in1_line.set_value(1)
    in2_line.set_value(0)
    
    in3_line.set_value(0)
    in4_line.set_value(1)

    # Motors C & D left
    in5_line.set_value(0)
    in6_line.set_value(1)

    in7_line.set_value(1)
    in8_line.set_value(0)

def rotate_ccw(): #rotate towards left
    ena_line.set_value(1)
    enb_line.set_value(1)
    enc_line.set_value(1)
    end_line.set_value(1)

     # Motors A & B left
    in1_line.set_value(1)
    in2_line.set_value(0)
    
    in3_line.set_value(0)
    in4_line.set_value(1)

    # Motors C & D left
    in5_line.set_value(0)
    in6_line.set_value(1)

    in7_line.set_value(1)
    in8_line.set_value(0)

# Function to stop all motors
def stop_all():
    ena_line.set_value(0)
    enb_line.set_value(0)
    enc_line.set_value(0)
    end_line.set_value(0)

    # Set all control lines low
    for line in [in1_line, in2_line, in3_line, in4_line, in5_line, in6_line, in7_line, in8_line]:
        line.set_value(0)

def getch():
    """Get a single character from the user without requiring Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

async def main_loop():
    try:
        print("Press 'w' to move forward, 's' to move backward, 'q' to stop, and Ctrl+C to exit.")
        while True:
            key = getch()
            if key == 'w':
                print("Moving forward")
                forward()
            elif key == 'x':
                print("Moving backward")
                backward()
            elif key == 'a':
                print("Moving left")
                left()
            elif key == 'd':
                print("Moving right")
                right()
            elif key == 'e':
                print("Moving diagonal forward right")
                diagonal_forward_right()
            elif key == 'q':
                print("Moving diagonal forward left")
                diagonal_forward_left()
            elif key == 'c':
                print("Moving diagonal backward left")
                diagonal_backward_left()
            elif key == 'z':
                print("Moving diagonal backward right")
                diagonal_backward_right()
            elif key == 's':
                print("Stopping motors")
                stop_all()
            else:
                print("Invalid key. Use 'w', 's', or 'q'.")
            await asyncio.sleep(0.1)  # Small delay to allow smooth control
    except KeyboardInterrupt:
        print("\nProgram interrupted. Exiting...")
        stop_all()
    finally:
        # Cleanup GPIO resources
        for line in lines:
            line.release()
        chip.close()
        print("Released GPIO resources.")

# Run the main loop
asyncio.run(main_loop())

###waterlevel sensor

import gpiod
import time

# Define GPIO chip and line for the water sensor
CHIP = "/dev/gpiochip0"  # GPIO chip device file


# Initialize the GPIO chip and line
chip = gpiod.Chip(CHIP)
line = chip.get_line(LINE)

# Request the line as input
line.request(consumer="water_sensor", type=gpiod.LINE_REQ_DIR_IN)

try:
    while True:
        # Read the value from the water sensor
        water_level = line.get_value()

        # Print the value
        print("Water Level:", water_level)

        # Sleep for a short time to avoid excessive CPU usage
        time.sleep(0.5)
finally:
    # Release the GPIO line when done
    line.release()
    chip.close()

#WATERPUMP 

import gpiod
import time

# Define the GPIO pin for the relay
RELAY_PIN = 6

# Create a chip object for GPIO access
chip = gpiod.Chip('gpiochip0')  # 'gpiochip0' is usually the default GPIO chip
line = chip.get_line(RELAY_PIN)

# Set the line as an output
line.request(consumer='pump_control', type=gpiod.LINE_REQ_DIR_OUT)

def turn_pump_on():
    print("Turning the pump ON")
    line.set_value(1)  # Turn on the relay (and pump)

def turn_pump_off():
    print("Turning the pump OFF")
    line.set_value(0)  # Turn off the relay (and pump)

try:
    while True:
        turn_pump_on()
        time.sleep(10)  # Pump on for 10 seconds
        turn_pump_off()
        time.sleep(5)   # Pump off for 5 seconds

except KeyboardInterrupt:
    print("Program stopped by user")
    turn_pump_off()

finally:
    line.release()  # Release the line







