import serial
import csv
import re
import time

# Update this to your actual serial port and baud rate
SERIAL_PORT = '/dev/ttyACM0'  # e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux
BAUD_RATE = 115200
CSV_FILENAME = 'bno_serial_data.csv'

# Regex pattern to parse your line format:
pattern = re.compile(r"ID:(\d+) A:([-.\d]+),([-.\d]+),([-.\d]+) T:(\d+)")

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, \
         open(CSV_FILENAME, mode='w', newline='') as csvfile:

        csv_writer = csv.writer(csvfile)
        # Write header
        csv_writer.writerow(['ID', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Timestamp'])

        print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud. Ctrl+C to stop.")

        try:
            while True:
                line = ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                match = pattern.match(line)
                if match:
                    sensor_id = int(match.group(1))
                    ax = float(match.group(2))
                    ay = float(match.group(3))
                    az = float(match.group(4))
                    timestamp = int(match.group(5))

                    csv_writer.writerow([sensor_id, ax, ay, az, timestamp])
                    csvfile.flush()

                    print(f"ID:{sensor_id} A:{ax},{ay},{az} T:{timestamp}")
                else:
                    print(f"Unrecognized line: {line}")

        except KeyboardInterrupt:
            print("\nStopped by user.")

if __name__ == "__main__":
    main()