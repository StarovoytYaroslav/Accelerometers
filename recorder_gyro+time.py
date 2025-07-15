import serial
import csv
import re
import datetime

# Update this to your actual serial port and baud rate
SERIAL_PORT = '/dev/ttyACM0'  # e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux
BAUD_RATE = 115200
CSV_FILENAME = 'bno_serial_data.csv'

# Regex to match: ID, Accel, Gyro, Timestamp
pattern = re.compile(
    r"ID:(\d+)\s+A:([-.\d]+),([-.\d]+),([-.\d]+)\s+G:([-.\d]+),([-.\d]+),([-.\d]+)\s+T:(\d+)"
)

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, \
         open(CSV_FILENAME, mode='w', newline='') as csvfile:

        csv_writer = csv.writer(csvfile)
        # Add 'DateTime' column
        csv_writer.writerow([
            'DateTime', 'ID',
            'Accel_X', 'Accel_Y', 'Accel_Z',
            'Gyro_X', 'Gyro_Y', 'Gyro_Z',
            'Sensor_Timestamp'
        ])

        print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud. Ctrl+C to stop.")

        try:
            while True:
                line = ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue

                match = pattern.match(line)
                if match:
                    now = datetime.datetime.now().isoformat(timespec='milliseconds')

                    sensor_id = int(match.group(1))
                    ax = float(match.group(2))
                    ay = float(match.group(3))
                    az = float(match.group(4))
                    gx = float(match.group(5))
                    gy = float(match.group(6))
                    gz = float(match.group(7))
                    sensor_timestamp = int(match.group(8))

                    csv_writer.writerow([
                        now, sensor_id, ax, ay, az, gx, gy, gz, sensor_timestamp
                    ])
                    csvfile.flush()

                    print(f"[{now}] ID:{sensor_id} A:{ax},{ay},{az} G:{gx},{gy},{gz} T:{sensor_timestamp}")
                else:
                    print(f"Unrecognized line: {line}")

        except KeyboardInterrupt:
            print("\nStopped by user.")

if __name__ == "__main__":
    main()