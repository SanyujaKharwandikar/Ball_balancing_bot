from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import board
import busio

class ServoController:
    def __init__(self):
        # I2C setup
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)

        # Set frequency (IMPORTANT for servos)
        self.pca.frequency = 50

        # ✅ Use channels 0,1,2 (change if needed)
        self.servos = [
            servo.Servo(self.pca.channels[0]),
            servo.Servo(self.pca.channels[1]),
            servo.Servo(self.pca.channels[2])
        ]

        print("✅ PCA9685 Servo Controller Initialized")

    # -----------------------------
    def set_angles(self, angles):
        """
        angles: [a, b, c] in degrees (0–180)
        """
        for i in range(3):
            angle = max(0, min(180, angles[i]))  # clamp
            self.servos[i].angle = angle

    # -----------------------------
    def cleanup(self):
        self.pca.deinit()
        print("🔌 PCA9685 stopped")
