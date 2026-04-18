"""Quick raw USB data test - read bytes from COM port and display them."""
import serial
import sys
import time

port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
print(f"Opening {port}...")

ser = serial.Serial(port, 115200, timeout=1.0)
time.sleep(0.5)

print(f"Reading... (in_waiting={ser.in_waiting})")
for i in range(10):
    data = ser.read(64)
    if data:
        hex_str = ' '.join(f'{b:02X}' for b in data[:32])
        print(f"  [{len(data)} bytes] {hex_str}")
    else:
        print(f"  [no data]")
    time.sleep(0.2)

ser.close()
print("Done.")
