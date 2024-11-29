import gpiod
import time
import sys
import tty
import termios

# GPIO chip and lines (adjust to your GPIO pins)
CHIP = "/dev/gpiochip0"  # GPIO chip, typically gpiochip0 on Raspberry Pi
ENA = 17  # PWM for Motor A
IN1 = 22
IN2 = 27
# ENB = 23  # PWM for Motor B
# IN3 = 24
# IN4 = 25

# Initialize GPIO lines
chip = gpiod.Chip(CHIP)
lines = {
    "ENA": chip.get_line(ENA),
    "IN1": chip.get_line(IN1),
    "IN2": chip.get_line(IN2),
    # "ENB": chip.get_line(ENB),
    # "IN3": chip.get_line(IN3),
    # "IN4": chip.get_line(IN4),
}

# Request GPIO lines as output
for line in lines.values():
    line.request(consumer="motor_control", type=gpiod.LINE_REQ_DIR_OUT)

# Motor control functions
def forward_a():
    lines["IN1"].set_value(1)
    lines["IN2"].set_value(0)

def backward_a():
    lines["IN1"].set_value(0)
    lines["IN2"].set_value(1)

# def forward_b():
#     lines["IN3"].set_value(1)
#     lines["IN4"].set_value(0)

# def backward_b():
#     lines["IN3"].set_value(0)
#     lines["IN4"].set_value(1)

def stop_all():
    for line in ["IN1", "IN2"]:#, "IN3", "IN4"]:
        lines[line].set_value(0)

def getch():
    """Get a single character from the user without requiring Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

try:
    mode = input("Choose mode (1 for timed control, 2 for keyboard control): ").strip()
    if mode == "1":
        print("Timed motor control mode activated.")
        while True:
            forward_a()
            time.sleep(2)
            backward_a()
            time.sleep(2)
    elif mode == "2":
        print("Keyboard motor control mode activated.")
        print("Controls: 'w' for forward, 's' for backward, 'x' to exit.")
        while True:
            key = getch()
            if key == 'w':
                print("Pressed w")
                forward_a()
                # Optional: forward_b() to move both motors
            elif key == 's':
                print("Pressed s")
                backward_a()
                # Optional: backward_b() to move both motors
            elif key == 'x' or key == '\x03':  # Ctrl+C to exit
                break
            else:
                stop_all()  # Stop motors if no valid key is pressed
    else:
        print("Invalid mode selected.")
except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    stop_all()
    print("GPIO cleaned up.")

