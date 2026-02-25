import Jetson.GPIO as GPIO
import time

# Define the GPIO pin you want to use
GPIO_PIN = 22  # Change this to the GPIO pin number you are using

# Setup GPIO
GPIO.setmode(GPIO.BOARD)  # Use BOARD numbering
GPIO.setup(GPIO_PIN, GPIO.OUT)

try:
    print("Generating PPS signal. Press Ctrl+C to stop.")
    while True:
        GPIO.output(GPIO_PIN, GPIO.HIGH)  # Rising edge trigger
        time.sleep(1)  # Pulse width (1 ms, adjust if needed)
        GPIO.output(GPIO_PIN, GPIO.LOW)   # Falling edge
        time.sleep(1)  # Wait for the rest of the second

except KeyboardInterrupt:
    print("\nStopping PPS signal.")

finally:
    GPIO.cleanup()  # Cleanup GPIO on exit