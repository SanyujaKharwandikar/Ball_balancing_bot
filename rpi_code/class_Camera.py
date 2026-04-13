import cv2
import numpy as np
import subprocess

class Camera:
    def __init__(self):
        self.width = 640
        self.height = 480

        # ✅ Start Pi camera (NO preview window)
        cmd = [
            "rpicam-vid",
            "-t", "0",
            "--width", str(self.width),
            "--height", str(self.height),
            "--framerate", "30",
            "--codec", "yuv420",
            "-o", "-"
        ]

        self.pipe = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)

        # Frame size for YUV420
        self.frame_size = self.width * self.height * 3 // 2

        # Pink detection range
        self.lower_pink = np.array([5, 50, 50])
        self.upper_pink = np.array([25, 255, 255])

        print("✅ Pi Camera started (no preview)")

    # -----------------------------
    def take_pic(self):
        raw = self.pipe.stdout.read(self.frame_size)

        if len(raw) != self.frame_size:
            print("❌ Frame read failed")
            return None

        # Convert YUV → BGR
        yuv = np.frombuffer(raw, dtype=np.uint8).reshape((self.height * 3 // 2, self.width))
        frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

        return frame

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

        # Convert to centered coords
        cx = x - self.width / 2
        cy = y - self.height / 2

        # Rotate frame
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
