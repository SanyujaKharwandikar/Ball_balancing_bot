import cv2
import numpy as np

class Camera:
    def __init__(self):
        # Camera setup (webcam instead of Picamera2)
        self.height = 480
        self.width = 640

        # ✅ IMPORTANT: use working camera index
        self.cap = cv2.VideoCapture(2)

        if not self.cap.isOpened():
            print("❌ Camera not opening")
            exit()

        # Force stable format (fix black screen)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # Pink HSV range (your working values)
        self.lower_pink = np.array([140, 120, 70])
        self.upper_pink = np.array([180, 255, 255])

        print("✅ Webcam initialized")

    # -----------------------------
    def take_pic(self):
        ret, frame = self.cap.read()

        if not ret:
            print("❌ Frame not received")
            return None

        return frame

    # -----------------------------
    def show_video(self, image):
        cv2.imshow("Live", image)
        cv2.waitKey(1)

    # -----------------------------
    def find_ball(self, image):
        if image is None:
            return -1, -1, 0

        # Convert to HSV
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Mask
        mask = cv2.inRange(image_hsv, self.lower_pink, self.upper_pink)

        # Clean noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 200:
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)

                # Draw
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)

                # Show debug
                cv2.imshow("Mask", mask)
                self.show_video(image)

                # Center transform
                x -= self.width / 2
                y -= self.height / 2
                x, y = -y, x

                return int(x), int(y), int(area)

        # Show even if not found
        cv2.imshow("Mask", mask)
        self.show_video(image)

        return -1, -1, 0

    # -----------------------------
    def clean_up_cam(self):
        self.cap.release()
        cv2.destroyAllWindows()
