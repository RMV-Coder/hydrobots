import sys
import tty
import termios
import asyncio
from gpiozero import OutputDevice, PWMOutputDevice

# Motor A
ENA = PWMOutputDevice(17)  # PWM for Motor A
IN1 = OutputDevice(27)  # back left forward
IN2 = OutputDevice(22)  # back left reverse

# Motor B
ENB = PWMOutputDevice(23)  # PWM for Motor B
IN3 = OutputDevice(24)  # back right forward
IN4 = OutputDevice(25)  # back right reverse

# Motor C
ENC = PWMOutputDevice(11)  # PWM for Motor C
IN5 = OutputDevice(6)   # front right forward
IN6 = OutputDevice(13)  # front right reverse

# Motor D
END = PWMOutputDevice(16)  # PWM for Motor D
IN7 = OutputDevice(19)  # front left forward
IN8 = OutputDevice(12)  # front left reverse


def forward(speed=0.2):
    # Set motor speed using PWM
    ENA.value = speed
    ENB.value = speed
    ENC.value = speed
    END.value = speed

    # Motors A & B forward
    IN1.off()
    IN2.on()
    IN3.on()
    IN4.off()

    # Motors C & D forward
    IN5.off()
    IN6.on()
    IN7.off()
    IN8.on()

# Function to move all motors backward
def backward(speed=0.2):
    # Set motor speed using PWM
    ENA.value = speed
    ENB.value = speed
    ENC.value = speed
    END.value = speed

    # Motors A & B forward
    IN1.on()
    IN2.off()
    IN3.off()
    IN4.on()

    # Motors C & D forward
    IN5.on()
    IN6.off()
    IN7.on()
    IN8.off()

def right(speed=0.2):
    # Set motor speed using PWM
    ENA.value = speed
    ENB.value = speed
    ENC.value = speed
    END.value = speed

    # Motors A & B forward
    IN1.on()
    IN2.off()
    IN3.off()
    IN4.on()

    # Motors C & D forward
    IN5.on()
    IN6.off()
    IN7.off()
    IN8.on()

def left(speed=0.2):
    # Set motor speed using PWM
    ENA.value = speed
    ENB.value = speed
    ENC.value = speed
    END.value = speed

    # Motors A & B forward
    IN1.off()
    IN2.on()
    IN3.on()
    IN4.off()

    # Motors C & D forward
    IN5.off()
    IN6.on()
    IN7.on()
    IN8.off()

# def diagonal_forward_right():
#     # Set motor speed using PWM
#     ENA.off()
#     ENB.value = speed
#     ENC.off()
#     END.value = speed

#     # Motors A & B forward
#     IN1.off()
#     IN2.off()
#     IN3.off()
#     IN4.on()

#     # Motors C & D forward
#     IN5.off()
#     IN6.off()
#     IN7.off()
#     IN8.on()

# def diagonal_forward_left():
#     # Set motor speed using PWM
#     ENA.value = speed
#     ENB.off()
#     ENC.value = speed
#     END.off()

#     # Motors A & B forward
#     IN1.off()
#     IN2.on()
#     IN3.off()
#     IN4.off()

#     # Motors C & D forward
#     IN5.off()
#     IN6.on()
#     IN7.off()
#     IN8.off()

# def diagonal_backward_left():
#     # Set motor speed using PWM
#     ENA.off()
#     ENB.value = speed
#     ENC.off()
#     END.value = speed

#     # Motors A & B forward
#     IN1.off()
#     IN2.off()
#     IN3.off()
#     IN4.on()

#     # Motors C & D forward
#     IN5.off()
#     IN6.off()
#     IN7.on()
#     IN8.off()

# def diagonal_backward_left():
#     ena_line.set_value(0)
#     enb_line.set_value(1)
#     enc_line.set_value(0)
#     end_line.set_value(1)

#      # Motors A & B left
#     in1_line.set_value(0)
#     in2_line.set_value(1)
    
#     in3_line.set_value(0)
#     in4_line.set_value(1)

#     # Motors C & D left
#     in5_line.set_value(0)
#     in6_line.set_value(1)

#     in7_line.set_value(1)
#     in8_line.set_value(0)

# def diagonal_backward_right():
#     ena_line.set_value(1)
#     enb_line.set_value(0)
#     enc_line.set_value(1)
#     end_line.set_value(0)

#      # Motors A & B left
#     in1_line.set_value(1)
#     in2_line.set_value(0)
    
#     in3_line.set_value(0)
#     in4_line.set_value(1)

#     # Motors C & D left
#     in5_line.set_value(0)
#     in6_line.set_value(1)

#     in7_line.set_value(1)
#     in8_line.set_value(0)

# # Function to stop all motors
def stop_all():
    ENA.off()
    ENB.off()
    ENC.off()
    END.off()
    IN1.off()
    IN2.off()
    IN3.off()
    IN4.off()
    IN5.off()
    IN6.off()
    IN7.off()
    IN8.off()

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
    speed = 0.00
    forward_, backward_, right_, left_ = False, False, False, False
    try:
        print("Press 'w' to move forward, 's' to move backward, 'q' to stop, and Ctrl+C to exit.")
        while True:
            key = getch()
            if key == 'w':
                print("Moving forward")
                if forward_:
                    if speed <= 0.80:
                        speed += 0.1
                else:
                    forward_ = True
                    backward_, right_, left_ = False, False, False
                    speed = 0.1
                forward(speed)
            elif key == 'x':
                print("Moving backward")
                if backward_:
                    if speed <= 0.80:
                        speed += 0.1
                else:
                    backward_ = True
                    forward_, right_, left_ = False, False, False
                    speed = 0.1
                backward(speed)
            elif key == 'a':
                print("Moving left")
                if left_:
                    if speed <= 0.80:
                        speed += 0.1
                else:
                    left_ = True
                    backward_, right_, forward_ = False, False, False
                    speed = 0.1
                left(speed)
            elif key == 'd':
                print("Moving right")
                if right_:
                    if speed <= 0.80:
                        speed += 0.1
                else:
                    right_ = True
                    backward_, forward_, left_ = False, False, False
                    speed = 0.1
                right(speed)
            # elif key == 'e':
            #     print("Moving diagonal forward right")
            #     diagonal_forward_right()
            # elif key == 'q':
            #     print("Moving diagonal forward left")
            #     diagonal_forward_left()
            # elif key == 'c':
            #     print("Moving diagonal backward left")
            #     diagonal_backward_left()
            # elif key == 'z':
            #     print("Moving diagonal backward right")
            #     diagonal_backward_right()
            elif key == 's':
                print("Stopping motors")
                stop_all()
            else:
                print("Invalid key. Use 'w', 's', or 'q'.")
            print(f"speed: {speed}")
            await asyncio.sleep(0.1)  # Small delay to allow smooth control

    except KeyboardInterrupt:
        print("\nProgram interrupted. Exiting...")
        stop_all()
    finally:
        print("Program terminated cleanly.")

# Run the main loop
asyncio.run(main_loop())
