import cv2
import numpy as np
import subprocess

class Camera:
    def __init__(self):
        # 🔧 You can change this
        self.width = 1280   # try 1280 first (stable)
        self.height = 720

        # ✅ Start Pi camera (NO preview window)
        cmd = [
            "rpicam-vid",
            "-t", "0",
            "--width", str(self.width),
            "--height", str(self.height),
            "--framerate", "30",
            "--codec", "mjpeg",   # 🔥 IMPORTANT FIX
            "--inline",
            "--quality", "90",
            "-o", "-"
        ]

        self.pipe = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)

        # Buffer for MJPEG decoding
        self.buffer = b""

        # 🎯 Pink detection range (tune if needed)
        self.lower_pink = np.array([5, 50, 50])
        self.upper_pink = np.array([25, 255, 255])

        print("✅ Pi Camera started (MJPEG, no preview)")

    # -----------------------------
    def take_pic(self):
        data = self.pipe.stdout.read(4096)

        if not data:
            return None

        self.buffer += data

        # Find JPEG frame
        start = self.buffer.find(b'\xff\xd8')
        end = self.buffer.find(b'\xff\xd9')

        if start != -1 and end != -1:
            jpg = self.buffer[start:end+2]
            self.buffer = self.buffer[end+2:]

            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            return frame

        return None

    # -----------------------------
    def find_ball(self, image):
        if image is None:
            return -1, -1, 0

        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Mask
        mask = cv2.inRange(hsv, self.lower_pink, self.upper_pink)

        # Clean noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            self.display(image, mask)
            return -1, -1, 0

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 300:
            self.display(image, mask)
            return -1, -1, 0

        (x, y), radius = cv2.minEnclosingCircle(largest)

        # Draw detection
        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
        cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)

        self.display(image, mask)

        # 🎯 Center coordinates
        cx = x - self.width / 2
        cy = y - self.height / 2

        # 🔄 Rotate coordinates (IMPORTANT)
        cx, cy = -cy, cx

        return int(cx), int(cy), int(area)

    # -----------------------------
    def display(self, image, mask):
        cv2.imshow("Live", image)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

    # -----------------------------
    def clean_up_cam(self):
        self.pipe.terminate()
        cv2.destroyAllWindows()
        print("📷 Camera stopped")
