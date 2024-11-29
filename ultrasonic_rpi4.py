from gpiozero import DistanceSensor
from time import sleep

# Define pins for TRIG and ECHO
TRIG = 21
ECHO = 20

# Initialize DistanceSensor
sensor = DistanceSensor(echo=ECHO, trigger=TRIG, max_distance=4)  # max_distance in meters (400 cm)

print("Measuring distance with gpiozero's DistanceSensor. Press Ctrl+C to stop.")

try:
    while True:
        distance_cm = sensor.distance * 100  # Convert distance to cm
        if 2 <= distance_cm <= 400:  # Check if within the sensor's range
            print(f"Distance: {distance_cm:.2f} cm")
        else:
            print("Out of range")
        sleep(1)  # Measure every second
except KeyboardInterrupt:
    print("Measurement stopped by user.")
finally:
    sensor.close()
