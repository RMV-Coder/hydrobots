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