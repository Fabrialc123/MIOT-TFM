import telnetlib
import time
import csv
import os
from datetime import datetime

HOST = "192.168.4.1"
PORT = 23
DELAY = 5
CSV_FILE = "ESP32_LIGHTSLEEP_5MIN.csv"

def parse_response(data):
    try:
        voltage, ampere, power, wh = map(float, data.strip().split(','))
        timestamp = int(time.time())
        datetime_str = datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S")
        return {
            "Timestamp (epoch)": timestamp,
            "Datetime": datetime_str,
            "Voltage (V)": voltage,
            "Ampere (A)": ampere,
            "Power (W)": power,
            "Watt Hour (Wh)": wh
        }
    except Exception as e:
        print(f"ERROR in data: {data} -> {e}")
        return None
    
def save_to_csv(entry):
    with open(CSV_FILE, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            entry["Timestamp (epoch)"],
            entry["Datetime"],
            entry["Voltage (V)"],
            entry["Ampere (A)"],
            entry["Power (W)"],
            entry["Watt Hour (Wh)"]
        ])

def main():
    if not os.path.isfile(CSV_FILE):
        with open(CSV_FILE, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["timestamp", "datetime", "voltage", "ampere", "power", "wh"])
    while True:
        try:
            with telnetlib.Telnet(HOST, PORT, timeout=5) as tn:
                time.sleep(1)  # Wait a sec to receive data
                raw_data = tn.read_some().decode("utf-8").strip()

                parsed = parse_response(raw_data)
                if parsed:
                    response = ""
                    for k, v in parsed.items():
                       response += f"{k}: {v} " 
                    print(response)
                    save_to_csv(parsed)
        except Exception as e:
            print(f"ERROR: {e}")

        #time.sleep(DELAY)

if __name__ == "__main__":
    main()
