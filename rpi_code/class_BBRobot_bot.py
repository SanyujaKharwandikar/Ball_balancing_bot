import math
from class_servo import ServoController

class BBrobot:
    def __init__(self):
        # ✅ Use PCA9685 instead of ESP
        self.servo = ServoController()

        # Robot geometry
        self.L = [0.04, 0.04, 0.065, 0.065]

        # Initial pose
        self.ini_pos = [0, 0, 0.0632]

        self.pz_max = 0.0732
        self.pz_min = 0.0532
        self.phi_max = 20

    # -----------------------------
    def clean_up(self):
        self.servo.cleanup()

    # -----------------------------
    def safe_sqrt(self, x):
        return math.sqrt(max(0, x))

    # -----------------------------
    # Inverse Kinematics (UNCHANGED)
    # -----------------------------
    def kinema_inv(self, n, Pz):
        L = self.L

        try:
            # ---------- Base ----------
            A = (L[0]+L[1])/Pz
            B = (Pz**2+L[2]**2-(L[0]+L[1])**2-L[3]**2)/(2*Pz)
            C = A**2+1
            D = 2*(A*B-(L[0]+L[1]))
            E = B**2+(L[0]+L[1])**2-L[2]**2

            Pmx = (-D + self.safe_sqrt(D**2 - 4*C*E)) / (2*C)
            Pmz = self.safe_sqrt(L[2]**2 - Pmx**2 + 2*(L[0]+L[1])*Pmx - (L[0]+L[1])**2)

            # ---------- Servo A ----------
            denom = math.sqrt(n[0]**2 + n[2]**2) + 1e-6

            a_m_x = (L[3]/denom)*n[2]
            a_m_z = Pz + (L[3]/denom)*(-n[0])

            A = (L[0]-a_m_x)/a_m_z
            B = (a_m_x**2 + a_m_z**2 - L[2]**2 - L[0]**2 + L[1]**2)/(2*a_m_z)

            C = A**2+1
            D = 2*(A*B - L[0])
            E = B**2 + L[0]**2 - L[1]**2

            ax = (-D + self.safe_sqrt(D**2 - 4*C*E)) / (2*C)
            az = self.safe_sqrt(L[1]**2 - ax**2 + 2*L[0]*ax - L[0]**2)

            if a_m_z < Pmz:
                az = -az

            theta_a = 90 - math.degrees(math.atan2(ax - L[0], az))

            # ---------- Servo B ----------
            denom = math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*math.sqrt(3)*n[0]*n[1]) + 1e-6

            b_m_x = (L[3]/denom)*(-n[2])
            b_m_y = (L[3]/denom)*(-math.sqrt(3)*n[2])
            b_m_z = Pz + (L[3]/denom)*(math.sqrt(3)*n[1]+n[0])

            A = -(b_m_x + math.sqrt(3)*b_m_y + 2*L[0]) / b_m_z
            B = (b_m_x**2 + b_m_y**2 + b_m_z**2 + L[1]**2 - L[2]**2 - L[0]**2)/(2*b_m_z)

            C = A**2+4
            D = 2*A*B + 4*L[0]
            E = B**2 + L[0]**2 - L[1]**2

            x = (-D - self.safe_sqrt(D**2 - 4*C*E)) / (2*C)
            y = math.sqrt(3)*x
            z = self.safe_sqrt(L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2)

            if b_m_z < Pmz:
                z = -z

            theta_b = 90 - math.degrees(math.atan2(math.sqrt(x**2+y**2)-L[0], z))

            # ---------- Servo C ----------
            denom = math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*math.sqrt(3)*n[0]*n[1]) + 1e-6

            c_m_x = (L[3]/denom)*(-n[2])
            c_m_y = (L[3]/denom)*(math.sqrt(3)*n[2])
            c_m_z = Pz + (L[3]/denom)*(-math.sqrt(3)*n[1]+n[0])

            A = -(c_m_x - math.sqrt(3)*c_m_y + 2*L[0]) / c_m_z
            B = (c_m_x**2 + c_m_y**2 + c_m_z**2 + L[1]**2 - L[2]**2 - L[0]**2)/(2*c_m_z)

            C = A**2+4
            D = 2*A*B + 4*L[0]
            E = B**2 + L[0]**2 - L[1]**2

            x = (-D - self.safe_sqrt(D**2 - 4*C*E)) / (2*C)
            y = -math.sqrt(3)*x
            z = self.safe_sqrt(L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2)

            if c_m_z < Pmz:
                z = -z

            theta_c = 90 - math.degrees(math.atan2(math.sqrt(x**2+y**2)-L[0], z))

            return [theta_a, theta_b, theta_c]

        except Exception as e:
            print("IK error:", e)
            return [0, 0, 0]

    # -----------------------------
    # Control robot (UPDATED)
    # -----------------------------
    def control_t_posture(self, pos):
        theta, phi, Pz = pos

        phi = min(phi, self.phi_max)
        Pz = max(self.pz_min, min(self.pz_max, Pz))

        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))

        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))

        n = [x, y, z]

        angles = self.kinema_inv(n, Pz)

        # ✅ clamp IK output
        angles = [max(-45, min(45, a)) for a in angles]

        # ✅ convert to servo range (IMPORTANT)
        servo_angles = [90 + a for a in angles]

        # ✅ send to PCA9685
        self.servo.set_angles(servo_angles)

    # -----------------------------
    def Initialize_posture(self):
        print("Initializing robot...")
        self.control_t_posture(self.ini_pos)
