from gpiozero import DistanceSensor
from time import sleep

# Initialize 4 ultrasonic sensors
sensor1 = DistanceSensor(echo=20, trigger=21)  # Sensor 1
sensor2 = DistanceSensor(echo=9, trigger=14)  # Sensor 2
sensor3 = DistanceSensor(echo=24, trigger=25)  # Sensor 3
sensor4 = DistanceSensor(echo=26, trigger=27)  # Sensor 4

try:
    while True:
        # Read distances from each sensor
        distance1 = sensor1.distance * 100  # Convert to cm
        distance2 = sensor2.distance * 100
        distance3 = sensor3.distance * 100
        distance4 = sensor4.distance * 100
        
        # Print the distances
        print(f"Sensor 1: {distance1:.2f} cm")
        print(f"Sensor 2: {distance2:.2f} cm")
        print(f"Sensor 3: {distance3:.2f} cm")
        print(f"Sensor 4: {distance4:.2f} cm")
        print("-" * 30)
        
        sleep(0.5)

except KeyboardInterrupt:
    print("Measurement stopped by User")
