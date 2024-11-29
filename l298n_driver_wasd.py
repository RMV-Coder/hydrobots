import gpiod
import sys
import tty
import termios
import asyncio


# Motor control pins
# Motor A
ENA = 17  # PWM for Motor A
IN1 = 27  # back left forward
IN2 = 22  # back left reverse

# Motor B
ENB = 23  # PWM for Motor B
IN3 = 24  # back right forward
IN4 = 25 # back right reverse

# Motor C
ENC = 11  # PWM for Motor C
IN5 = 6   # front right forward
IN6 = 13    # front right reverse

# Motor D
END = 16  # PWM for Motor D
IN7 = 19 # front left forward
IN8 = 12 # front left reverse

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
