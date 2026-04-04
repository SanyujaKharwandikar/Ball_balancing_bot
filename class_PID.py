import math
import time

class PID:
    def __init__(self, K_PID, k, alpha):
        self.kp = K_PID[0]
        self.ki = K_PID[1]
        self.kd = K_PID[2]

        self.k = k
        self.alpha = alpha  # low-pass filter

        # state
        self.last_output_x = 0
        self.last_output_y = 0

        self.last_error_x = 0
        self.last_error_y = 0

        self.integral_x = 0
        self.integral_y = 0

        self.last_time = None

        #  IMPORTANT: anti-windup
        self.integral_limit = 50

        #  IMPORTANT: output limit (phi limit)
        self.output_limit = 20

    # -----------------------------
    def compute(self, Goal, Current_value):
        current_time = time.perf_counter()

        if self.last_time is None:
            self.last_time = current_time
            return 0, 0

        dt = current_time - self.last_time

        # safety
        if dt <= 0:
            return 0, 0

        # -------------------------
        # ERROR
        # -------------------------
        error_x = Goal[0] - Current_value[0]
        error_y = Goal[1] - Current_value[1]

        # -------------------------
        # INTEGRAL (with clamp)
        # -------------------------
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt

        self.integral_x = max(-self.integral_limit, min(self.integral_limit, self.integral_x))
        self.integral_y = max(-self.integral_limit, min(self.integral_limit, self.integral_y))

        # -------------------------
        # DERIVATIVE
        # -------------------------
        derivative_x = (error_x - self.last_error_x) / dt
        derivative_y = (error_y - self.last_error_y) / dt

        # -------------------------
        # PID OUTPUT
        # -------------------------
        output_x = (
            self.kp * error_x +
            self.ki * self.integral_x +
            self.kd * derivative_x
        )

        output_y = (
            self.kp * error_y +
            self.ki * self.integral_y +
            self.kd * derivative_y
        )

        # -------------------------
        # LOW PASS FILTER (IMPORTANT)
        # -------------------------
        output_x = self.alpha * output_x + (1 - self.alpha) * self.last_output_x
        output_y = self.alpha * output_y + (1 - self.alpha) * self.last_output_y

        # -------------------------
        # CONVERT → theta, phi
        # -------------------------
        if abs(output_x) < 1e-6 and abs(output_y) < 1e-6:
            theta = 0
            phi = 0
        else:
            theta = math.degrees(math.atan2(output_y, output_x))
            if theta < 0:
                theta += 360

            phi = self.k * math.sqrt(output_x**2 + output_y**2)

            #  LIMIT OUTPUT
            phi = min(phi, self.output_limit)

        # -------------------------
        # SAVE STATE
        # -------------------------
        self.last_error_x = error_x
        self.last_error_y = error_y

        self.last_output_x = output_x
        self.last_output_y = output_y

        self.last_time = current_time

        return theta, phi

    # -----------------------------
    def reset(self):
        self.last_output_x = 0
        self.last_output_y = 0

        self.last_error_x = 0
        self.last_error_y = 0

        self.integral_x = 0
        self.integral_y = 0

        self.last_time = None
