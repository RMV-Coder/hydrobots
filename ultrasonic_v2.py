import RPi.GPIO as GPIO
import asyncio

# Set pins
TRIG = 21  # Associate pin 23 to TRIG
ECHO = 20  # Associate pin 24 to ECHO

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

async def get_distance():
    # Set TRIG LOW
    GPIO.output(TRIG, False)
    print("high trig 0")
    await asyncio.sleep(2)

    # Send 10us pulse to TRIG
    GPIO.output(TRIG, True)
    print("high trig 1")
    await asyncio.sleep(0.00001)
    GPIO.output(TRIG, False)
    print("high trig 0")

    # Initialize pulse_start and pulse_end
    pulse_start = 0
    pulse_end = 0

    # Start recording the time when the wave is sent
    while GPIO.input(ECHO) == 0:
        pulse_start = asyncio.get_event_loop().time()
        print("ECHO 0")

    while GPIO.input(ECHO) == 1:
        pulse_end = asyncio.get_event_loop().time()
        print("ECHO 1")

    # Calculate the difference in times
    pulse_duration = pulse_end - pulse_start

    # Multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

async def main():
    try:
        while True:
            dist = await get_distance()
            print("Measured Distance = {:.2f} cm".format(dist))
            await asyncio.sleep(1)
    except Exception as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Measurement stopped by User")
    finally:
        GPIO.cleanup()

# Run the main function
asyncio.run(main())