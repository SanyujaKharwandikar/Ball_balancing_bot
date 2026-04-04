import class_BBRobot
import class_Camera
import class_PID
import time
import cv2

# -----------------------------
# INIT
# -----------------------------
K_PID = [0.12, 0.0, 0.02]
k = 1
alpha = 0.7

Robot = class_BBRobot.BBrobot()
camera = class_Camera.Camera()
pid = class_PID.PID(K_PID, k, alpha)

Robot.Initialize_posture()
pz_ini = Robot.ini_pos[2]

goal = [0, 0]

print("🚀 System Started")

# -----------------------------
# MAIN LOOP
# -----------------------------
try:
    while True:

        # -------------------------
        # 1. Capture frame
        # -------------------------
        frame = camera.take_pic()

        if frame is None:
            print("⚠️ No frame")
            continue

        # -------------------------
        # 2. Detect ball
        # -------------------------
        x, y, area = camera.find_ball(frame)

        # -------------------------
        # 3. If ball not found
        # -------------------------
        if x == -1:
            pid.reset()

            # still show frame (important to avoid freeze)
            cv2.imshow("Live", frame)

            if cv2.waitKey(1) & 0xFF == 27:
                break

            continue

        # -------------------------
        # 4. Deadzone
        # -------------------------
        if abs(x) < 5 and abs(y) < 5:
            theta, phi = 0, 0
        else:
            theta, phi = pid.compute(goal, [x, y])

        # -------------------------
        # 5. Send to robot
        # -------------------------
        pos = [theta, phi, pz_ini]
        Robot.control_t_posture(pos)

        # -------------------------
        # 6. Debug
        # -------------------------
        print(f"x:{x} y:{y} theta:{theta:.2f} phi:{phi:.2f}")

        # -------------------------
        # 7. Exit key
        # -------------------------
        if cv2.waitKey(1) & 0xFF == 27:
            break

        # -------------------------
        # 8. Timing (CRITICAL)
        # -------------------------
        time.sleep(0.03)

# -----------------------------
# CLEANUP
# -----------------------------
except KeyboardInterrupt:
    print("Stopping...")

finally:
    Robot.clean_up()
    camera.clean_up_cam()
