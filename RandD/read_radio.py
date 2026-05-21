import serial
import time

PORT = "COM6"
BAUD = 420000

print(f"Opening {PORT} at {BAUD}...")

try:
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print("Opened.")
        print("Reading bytes. Press Ctrl+C to stop.")

        while True:
            data = ser.read(64)
            if data:
                print(data.hex(" "))
            else:
                print(".", end="", flush=True)
                time.sleep(0.2)

except Exception as e:
    print("Failed:")
    print(repr(e))
