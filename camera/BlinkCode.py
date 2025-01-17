import time
import serial

# Replace 'COM3' with your Arduino's port (e.g., '/dev/ttyACM0')
ARDUINO_PORT = 'COM3'
BAUD_RATE = 9600

def main():
    # Open serial port
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset
        print(f"Opened {ARDUINO_PORT} successfully.")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    print("Blinking LED via Python! Press Ctrl+C to stop.")

    try:
        while True:
            # Turn LED ON
            ser.write(b'1')
            print("LED ON")
            time.sleep(1)

            # Turn LED OFF
            ser.write(b'0')
            print("LED OFF")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting...")

    # Cleanup
    ser.close()

if __name__ == "__main__":
    main()
