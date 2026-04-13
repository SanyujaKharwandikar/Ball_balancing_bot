import math
import time
from adafruit_servokit import ServoKit

# -----------------------------
# PCA9685 Setup
# -----------------------------
kit = ServoKit(channels=16)

# -----------------------------
# OFFSETS (TUNE THESE!)
# -----------------------------
OFFSET_S1 = 0
OFFSET_S2 = 0
OFFSET_S3 = 0

# -----------------------------
# CLAMP FUNCTION
# -----------------------------
def clamp(angle):
    return max(0, min(180, angle))


# -----------------------------
# ROBOT CONTROLLER
# -----------------------------
class RobotController:

    def __init__(self, model):
        self.model = model

        print("✅ RobotController initialized (PCA9685)")

    # -----------------------------
    # LOW LEVEL SERVO CONTROL
    # -----------------------------
    def set_motor_angles(self, theta1, theta2, theta3):

        # Apply clamp + offsets
        a1 = clamp(theta1 + OFFSET_S1)
        a2 = clamp(theta2 + OFFSET_S2)
        a3 = clamp(theta3 + OFFSET_S3)

        # Send to PCA9685
        kit.servo[0].angle = a1
        kit.servo[1].angle = a2
        kit.servo[2].angle = a3

        # Debug (optional)
        # print(f"Servo Angles → {a1:.2f}, {a2:.2f}, {a3:.2f}")

    # -----------------------------
    # VECTOR MODE CONTROL
    # -----------------------------
    def Goto_N_time_vector(self, x, y, z, speed=8):

        # Convert vector → motor angles using IK
        result = self.model.inverse_kinematics_vector(x, y, z)

        if not result:
            print("❌ IK failed")
            return

        theta1, theta2, theta3 = result

        self.set_motor_angles(theta1, theta2, theta3)

    # -----------------------------
    # SPHERICAL CONTROL (USED IN MAIN)
    # -----------------------------
    def Goto_N_time_spherical(self, theta, phi, speed=8):

        # Convert spherical → Cartesian
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))

        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))

        self.Goto_N_time_vector(x, y, z, speed)

    # -----------------------------
    # SMOOTH MOVE (OPTIONAL)
    # -----------------------------
    def Goto_time_vector(self, x, y, z, duration=1.0):

        steps = int(duration * 50)  # 50 Hz

        for i in range(steps):
            self.Goto_N_time_vector(x, y, z)
            time.sleep(duration / steps)

    # -----------------------------
    # INIT POSITION
    # -----------------------------
    def initialize(self):
        print("Initializing robot...")

        # flat plate (z = 1)
        self.Goto_N_time_vector(0, 0, 1)

        time.sleep(1)


# -----------------------------
# TEST BLOCK
# -----------------------------
if __name__ == "__main__":

    from robotKinematics import RobotKinematics

    model = RobotKinematics()
    rc = RobotController(model)

    rc.initialize()

    print("Testing motion...")

    while True:
        # small tilt test
        rc.Goto_N_time_spherical(0, 5)
        time.sleep(1)

        rc.Goto_N_time_spherical(90, 5)
        time.sleep(1)

        rc.Goto_N_time_spherical(180, 5)
        time.sleep(1)

        rc.Goto_N_time_spherical(270, 5)
        time.sleep(1)
