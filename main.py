## l298_driver_wasd.py
from gpiozero import OutputDevice, PWMOutputDevice, DistanceSensor
import sys
import tty
import termios
import asyncio

import gpiod
# from time import sleep

import torch
import cv2
import numpy as np
from picamera2 import Picamera2
from torchvision import transforms
from PIL import Image
from ultralytics import YOLO
import asyncio
# import websockets
import json
import sys
from aiortc import VideoStreamTrack, RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
from av import VideoFrame
import os
import time
import pytz
import jwt
from datetime import datetime, timedelta, timezone
import requests
from pathlib import Path
import atexit
import logging

# --- GPIO PINS --- #
#3V3 POWER = 
#GPIO2 = 
#GPIO3 = 
#GPIO4 = 
#GROUND = 
ENA = PWMOutputDevice(17)  #GPIO17 = PWM for Motor A
IN1 = OutputDevice(27)  #GPIO27 = back left forward
IN2 = OutputDevice(22)   #GPIO22 = back left reverse
#3V3 POWER = 
#GPIO10(MOSI)
#GPIO9(MISO)
ENC = PWMOutputDevice(11)  # PWM for Motor C GPIO11 (SCLK) 
#GROUND
#GPIO0(ID_SD)
RELAY_PIN = 5 #GPIO5 = WATER PUMP
IN5 = OutputDevice(6)    #GPIO6 front = right forward
IN6 = OutputDevice(13)   #GPIO13(PWM1) = front right reverse
IN7 = OutputDevice(19)  #GPIO19(PCM_FS) = front left forward
LINE = 26  #GPIO26 = WATER LEVEL SENSOR
#GROUND
#GPIO5V
#GPIO5V
#GROUND
#GPIO14(TXD)
#GPIO15(RXD)
#GPIO18(PCM_CLK)
#GROUND
ENB = PWMOutputDevice(23)  #GPIO23 = PWM for Motor B
IN3 = OutputDevice(24)  #GPIO24 = back right forward
#GROUND
IN4 = OutputDevice(25)  #GPIO25 = back right reverse
#GPIO8(CE0)
#GPIO7(CE1)
#GPIO1(ID_SC)
#GROUND
IN8 = OutputDevice(12)  #GPIO12 = Front left reverse
#GROUND
END = PWMOutputDevice(16)  #GPIO16 = PWM for Motor D 
echo=20 #GPIO20(PCM_DIN)
trigger=21 #GPIO21(PCM_DOUT)



# Define GPIO chip and line for the water sensor
CHIP = "/dev/gpiochip0"  # GPIO chip device file


# Initialize the GPIO chip and line
chip = gpiod.Chip(CHIP)
line = chip.get_line(LINE)

# Request the line as input
line.request(consumer="water_sensor", type=gpiod.LINE_REQ_DIR_IN)


# --- LOGGING CONFIGURATION --- #










# Load the best model weights
model_path = "YOLOv11/runs/detect/train5/weights/best.pt"  # Path to the best model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = YOLO(model_path)
frame_width = 1280
frame_height = 720
# Initialize Picamera2
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (frame_width, frame_height)})
picam2.configure(camera_config)
picam2.start()

def calculate_combined_center_x(pot_x, plant_x, frame_width):
    # Calculate combined center x
    combined_center_x = (pot_x + plant_x) / 2
    # Calculate frame center
    frame_center_x = frame_width / 2
    # Determine position relative to frame center
    position = "right" if combined_center_x > frame_center_x else "left" if combined_center_x < frame_center_x else "center"
    return combined_center_x, position

def detect_objects(frame):
    # Convert frame from BGR (OpenCV) to RGB
    # image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # img = Image.fromarray(image)
    img = Image.fromarray(frame)

    # Perform inference
    results = model.predict(source=img, save=True)
    
    plant_detected = False
    pot_detected = False
    x_plant, y_plant, x_pot, y_pot = 0, 0, 0, 0
    w_plant, h_plant, w_pot, h_pot = 0, 0, 0, 0
    condition = 0
    for result in results:
        # Extract predictions
        detections = result.boxes.xywh.cpu().numpy()  # xywh format: [x_center, y_center, width, height]
        confidences = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy()  # class id = 0: plant, 1: pot
        
        for detection, confidence, class_id in zip(detections, confidences, class_ids):
            x_center, y_center, width, height = detection
            conf = confidence

            if class_id == 1 and conf > 0.5:  # Pot
                pot_detected = True
                x_pot, x_pot = x_center, y_center
                w_pot, h_pot = width, height
                condition = w_pot * 2
                print(f"Pot -> X: {x_pot:.2f}, Y: {y_pot:.2f}, "
                      f"W: {width:.2f}, H: {height:.2f}, Confidence: {conf:.2f}")

            elif class_id == 0 and conf > 0.5 and width < condition:  # Plant
                plant_detected = True
                x_plant, y_plant = x_center, y_center
                print(f"Plant -> X: {x_plant:.2f}, Y: {y_center:.2f}, "
                      f"W: {width:.2f}, H: {height:.2f}, Confidence: {conf:.2f}")
            
    
    if plant_detected and pot_detected:
        print("Detected potted plant!")
        picam2.stop()
        print("Camera stopped.")
        combined_center_x, position = calculate_combined_center_x(pot_x, plant_x, frame_width)
        print(f"Combined Center X: {combined_center_x:.2f}, Position Relative to Frame Center: {position}")
        distance = detect_distance()
        if distance >= 100:
            
        if position == "right":
            right(0.3)
        elif position == "left":
            left(0.3)
        else:
            forward()
        exit()

def detect_plant():
    try:
        print("Starting object detection...")
        
        while True:
            # Capture a frame
            frame = picam2.capture_array()

            # Perform object detection
            detect_objects(frame)

            # Add a small delay
            cv2.waitKey(10)

    except KeyboardInterrupt:
        print("Detection stopped by user.")
    finally:
        picam2.stop()
        print("Camera stopped.")

def detect_distance():
    # Initialize ultrasonic sensor
    sensor1 = DistanceSensor(echo=echo, trigger=trigger)  # Sensor 1

    try:
        distance1 = sensor1.distance * 100  # Convert to cm
            
        # Print the distances
        print(f"Sensor 1: {distance1:.2f} cm")
        print("-" * 30)
        return distance1
        # while True:
            # Read distances from each sensor
            # distance1 = sensor1.distance * 100  # Convert to cm
            
            # # Print the distances
            # print(f"Sensor 1: {distance1:.2f} cm")
            # print("-" * 30)
            
            # sleep(0.5)

    except KeyboardInterrupt:
        print("Measurement stopped by User")
async def set_value(pin, value):
    pin.value = value

async def forward_(speed=0.2):
    await stop()
    # Set motor speed using PWM
    await asyncio.gather(
        set_value(ENA, speed),
        set_value(ENB, speed),
        set_value(ENC, speed),
        set_value(END, speed)
    )

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

async def forward(speed=0.2):
    await stop()
    # Set motor speed using PWM
    ENA.value = speed+0.1
    ENB.value = speed+0.1
    if speed < 0.5:
        ENC.value = speed+0.5
    else:
        ENC.value = speed+0.3
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
async def backward(speed=0.2):
    # Set motor speed using PWM
    ENA.value = speed+0.1
    ENB.value = speed+0.1
    ENC.value = speed
    if speed < 0.5:
        END.value = speed+0.5
    else:
        END.value = speed+0.3

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

async def right(speed=0.2):
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

async def left(speed=0.2, time = 1):
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

#     in7_line.set_value(0)
#     in8_line.set_value(1)

# def diagonal_forward_left():
#     ena_line.set_value(1)
#     enb_line.set_value(0)
#     enc_line.set_value(1)
#     end_line.set_value(0)

#      # Motors A & B left
#     in1_line.set_value(0)
#     in2_line.set_value(1)
    
#     in3_line.set_value(0)
#     in4_line.set_value(1)

#     # Motors C & D left
#     in5_line.set_value(0)
#     in6_line.set_value(1)

#     in7_line.set_value(0)
#     in8_line.set_value(1)

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

async def rotate_ccw(speed=0.15): #rotate towards left
    ENA.value = speed
    ENB.value = speed
    ENC.value = speed
    END.value = speed

     # Motors A & B left
    IN1.on()
    IN2.off()
    
    IN3.off()
    IN4.on()

    # Motors C & D left
    IN5.off()
    IN6.on()

    IN7.on()
    IN8.off()

# Function to stop all motors
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

async def stop():
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
# def turn_pump_on():
#     print("Turning the pump ON")
#     line.set_value(1)  # Turn on the relay (and pump)

# def turn_pump_off():
#     print("Turning the pump OFF")
#     line.set_value(0)  # Turn off the relay (and pump)

# def pump_water():
#     try:
#         turn_pump_on()
#         time.sleep(3)  # Pump on for 10 seconds
#         turn_pump_off()
#     except KeyboardInterrupt:
#         print("Program stopped by user")
#         turn_pump_off()

#     finally:
#         line.release()  # Release the line
#         print("Released the line for water pump relay")
def check_water_level():
    try:
        water_level = line.get_value()
        print("Water Level:", water_level)
        time.sleep(0.5)
    finally:
        # Release the GPIO line when done
        line.release()
        chip.close()
        print("Released the line for water level sensor")
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
            elif key == 't':
                detect_plant()
            elif key == 'u':
                detect_distance()
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

###waterlevel sensor



#WATERPUMP 

# Define the GPIO pin for the relay
# RELAY_PIN = 6

# # Create a chip object for GPIO access
# chip = gpiod.Chip('gpiochip0')  # 'gpiochip0' is usually the default GPIO chip
# line = chip.get_line(RELAY_PIN)

# # Set the line as an output
# line.request(consumer='pump_control', type=gpiod.LINE_REQ_DIR_OUT)









