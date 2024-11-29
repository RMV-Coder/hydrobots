import gpiod
import time

# Define GPIO chip and line for the water sensor
CHIP = "/dev/gpiochip0"  # GPIO chip device file
LINE = 26  # GPIO pin number

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
