import serial
import time

class ESP32:
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # allow ESP32 reset
            print(f"[ESP32] Connected on {port}")
        except Exception as e:
            print(f"[ESP32 ERROR] {e}")
            self.ser = None

    # -----------------------------
    # Send 3 servo angles
    # -----------------------------
    def send_angles(self, angles):
        if self.ser is None:
            return

        try:
            # format: a,b,c\n
            msg = f"{angles[0]:.2f},{angles[1]:.2f},{angles[2]:.2f}\n"
            self.ser.write(msg.encode())

        except Exception as e:
            print(f"[ESP32 SEND ERROR] {e}")

    # -----------------------------
    # Optional debug read
    # -----------------------------
    def read(self):
        if self.ser and self.ser.in_waiting:
            return self.ser.readline().decode(errors='ignore').strip()
        return None

    # -----------------------------
    def close(self):
        if self.ser:
            self.ser.close()
            print("[ESP32] Connection closed")
