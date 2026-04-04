import cv2
import numpy as np

class CameraTest:
    def __init__(self):
        self.width = 640
        self.height = 480

        # Pink range (tune if needed)
        self.lower_pink = np.array([140, 120, 70])
        self.upper_pink = np.array([180, 255, 255])

    def show_video(self, image):
        cv2.imshow("Live", image)

    def find_ball(self, image):
        # ❌ REMOVE BGRA conversion (not needed on laptop)

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
            self.show_video(image)
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 300:
            self.show_video(image)
            return None

        (x, y), radius = cv2.minEnclosingCircle(largest)

        # Draw
        center = (int(x), int(y))
        cv2.circle(image, center, int(radius), (0, 255, 0), 2)
        cv2.circle(image, center, 5, (0, 0, 255), -1)

        # Show mask for debugging
        cv2.imshow("Mask", mask)

        self.show_video(image)

        # Center coordinates
        cx = x - self.width / 2
        cy = y - self.height / 2

        # Rotate
        cx, cy = -cy, cx

        return int(cx), int(cy), int(area)


# 🚀 MAIN
cam = CameraTest()

# ✅ Use your webcam index = 2
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

if not cap.isOpened():
    print("❌ Camera not opening")
    exit()

# Set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Frame not received")
        break

    result = cam.find_ball(frame)

    if result is not None:
        print("Ball:", result)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
