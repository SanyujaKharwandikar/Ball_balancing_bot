import cv2
import numpy as np
import subprocess

class Camera:
    def __init__(self):
        self.width = 1640
        self.height = 1232

        # 🔥 BEST PIPELINE (NO STREAK)
        cmd = [
            "rpicam-vid",
            "-t", "0",
            "--width", str(self.width),
            "--height", str(self.height),
            "--framerate", "30",
            "--codec", "mjpeg",   # 🔥 THIS FIXES STREAKING
            "--quality", "90",
            "--inline",
            "-o", "-"
        ]

        self.pipe = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)

        self.buffer = b""

        self.lower_pink = np.array([140, 120, 70])
        self.upper_pink = np.array([180, 255, 255])

        print("✅ Pi Camera running at 1640x1232 (NO preview, NO streak)")

    # -----------------------------
    def take_pic(self):
        while True:
            data = self.pipe.stdout.read(4096)

            if not data:
                return None

            self.buffer += data

            start = self.buffer.find(b'\xff\xd8')
            end = self.buffer.find(b'\xff\xd9')

            if start != -1 and end != -1:
                jpg = self.buffer[start:end+2]
                self.buffer = self.buffer[end+2:]

                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                return frame

    # -----------------------------
    def show_video(self, image):
        cv2.imshow("Live", image)
        cv2.waitKey(1)

    # -----------------------------
    def find_ball(self, image):
        if image is None:
            return -1, -1, 0

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_pink, self.upper_pink)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 300:
                (x, y), radius = cv2.minEnclosingCircle(largest)

                cv2.circle(image, (int(x), int(y)), int(radius), (0,255,0), 2)
                cv2.circle(image, (int(x), int(y)), 5, (0,0,255), -1)

                cv2.imshow("Mask", mask)
                self.show_video(image)

                # center transform
                cx = x - self.width / 2
                cy = y - self.height / 2
                cx, cy = -cy, cx

                return int(cx), int(cy), int(area)

        cv2.imshow("Mask", mask)
        self.show_video(image)

        return -1, -1, 0

    # -----------------------------
    def clean_up_cam(self):
        self.pipe.terminate()
        cv2.destroyAllWindows()
        print("📷 Camera stopped")
