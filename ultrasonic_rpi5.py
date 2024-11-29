import asyncio
import gpiod
import time

# Define GPIO chip and pin numbers
chip = gpiod.Chip("gpiochip0")
TRIG = 21  # GPIO pin for TRIG
ECHO = 20  # GPIO pin for ECHO

# Request GPIO lines
trig_line = chip.get_line(TRIG)
echo_line = chip.get_line(ECHO)

trig_line.request(consumer="UltrasonicSensor", type=gpiod.LINE_REQ_DIR_OUT)
echo_line.request(consumer="UltrasonicSensor", type=gpiod.LINE_REQ_DIR_IN)

async def measure_distance():
    try:
        # Ensure TRIG is low initially
        trig_line.set_value(0)
        await asyncio.sleep(0.002)  # Wait for sensor stabilization

        # Send a 10-microsecond pulse to TRIG
        trig_line.set_value(1)
        await asyncio.sleep(0.00001)  # 10 microseconds
        trig_line.set_value(0)

        # Wait for ECHO to go HIGH
        start_time = time.time()
        while echo_line.get_value() == 0:
            start_time = time.time()

        # Wait for ECHO to go LOW
        stop_time = time.time()
        while echo_line.get_value() == 1:
            stop_time = time.time()

        # Calculate distance
        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2  # Speed of sound: 34300 cm/s
        print(f"Distance: {distance:.2f} cm")

        return distance

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Release GPIO lines
        trig_line.release()
        echo_line.release()
        chip.close()

# Run the asyncio event loop
asyncio.run(measure_distance())
