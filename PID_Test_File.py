from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch

class Robot:
    """Robot control wrapper for motion and PID heading corrections."""

    TURN_FLOOR = 18
    STR_FLOOR = 25
    SETTLE_TIME = 500
    I_CLAMP = 25
    ACCEL_ZONE = 100
    DECEL_ZONE = 100

    STR_KP = 5.0
    STR_KD = 12.0
    STR_KI = 0.05
    TURN_KP = 3.0
    TURN_KD = 7.0
    TURN_KI = 0.06

    def __init__(self, left_port=Port.B, right_port=Port.D):
        self.hub = PrimeHub()
        self.left_motor = Motor(left_port, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(right_port, Direction.CLOCKWISE)
        self.timer = StopWatch()
        
        # Data Logging
        self.report_card = []

    def get_yaw(self):
        """Return the current IMU heading in degrees."""
        return self.hub.imu.heading()

    def get_shortest_error(self, target_angle):
        """Return the smallest signed heading error to the target angle."""
        error = target_angle - self.get_yaw()
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error

    def log_result(self, move_name, target):
        """Saves final accuracy against the target heading."""
        final_err = self.get_shortest_error(target)
        self.report_card.append((move_name, round(float(target), 2), final_err))

    def print_diagnostic_report(self):
        """Prints the diagnostic report of the robot's moves."""
        print("\n" + "="*35)
        print("    HOME DIAGNOSTIC REPORT")
        print("="*35)
        print(f"{'MOVE':15} | {'TARGET':7} | {'ERROR':5}")
        print("-" * 35)
        for name, target, err in self.report_card:
            status = "OK" if abs(err) < 0.8 else "Check Me"
            print(f"{name:15} | {target:7} | {err:5.2f} [{status}]")
        print("="*35 + "\n")
        
        total_err = sum(abs(e) for _, _, e in self.report_card)
        avg_err = total_err / len(self.report_card)
        bias = sum(e for _, _, e in self.report_card) # Net direction of drift

        print("-" * 35)
        print(f"Total Cumulative Error: {total_err:.2f}°")
        print(f"Average Error per Move: {avg_err:.2f}°")
        print(f"Net Bias (Systemic Drift): {bias:.2f}°")
        print("="*35 + "\n")

    def straight(self, target_distance_deg, speed, target_heading=None):
        """Drive the robot straight for a given distance with PID heading correction."""
        wait(200)
        if target_heading is None:
            target_heading = self.get_yaw()
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        last_error, integral = 0, 0

        print(f"\n--- Straight: {target_distance_deg} deg ---")

        while True:
            current_dist = (abs(self.left_motor.angle()) + abs(self.right_motor.angle())) / 2
            if current_dist >= abs(target_distance_deg): break

            if current_dist < self.ACCEL_ZONE:
                current_speed = max(self.STR_FLOOR, (current_dist / self.ACCEL_ZONE) * speed)
            elif (abs(target_distance_deg) - current_dist) < self.DECEL_ZONE:
                dist_remaining = abs(target_distance_deg) - current_dist
                current_speed = max(self.STR_FLOOR, (dist_remaining / self.DECEL_ZONE) * speed)
            else:
                current_speed = speed

            error = self.get_shortest_error(target_heading)
            integral = max(min(integral + error, self.I_CLAMP), -self.I_CLAMP)
            derivative = error - last_error
            correction = (self.STR_KP * error) + (self.STR_KI * integral) + (self.STR_KD * derivative)

            self.left_motor.dc(int(current_speed + correction))
            self.right_motor.dc(int(current_speed - correction))

            if current_dist % 50 == 0:
                print(f"D:{current_dist:4.0f} | E:{error:5.2f}")

            last_error = error
            wait(10)

        self.left_motor.stop()
        self.right_motor.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Straight", target_heading)

    def turn_tank(self, target_angle, speed=100):
        """Rotate the robot in place to a target heading using PID control."""
        print(f"\n--- Tank Turn: {target_angle} ---")
        last_error, integral, stable_count = self.get_shortest_error(target_angle), 0, 0
        while stable_count < 10:
            error = self.get_shortest_error(target_angle)
            integral = max(min(integral + error, self.I_CLAMP), -self.I_CLAMP)
            pwr = (error * self.TURN_KP) + (integral * self.TURN_KI) + (self.TURN_KD * (error - last_error))
            
            if abs(error) > 0.5 and abs(pwr) < self.TURN_FLOOR:
                pwr = self.TURN_FLOOR + 2  if pwr > 0 else -(self.TURN_FLOOR + 2) 
            if abs(error) < 0.5:
                pwr = 0

            pwr = int(max(min(pwr, speed), -speed))
            self.left_motor.dc(pwr)
            self.right_motor.dc(-pwr)

            last_error = error
            stable_count = stable_count + 1 if abs(error) < 0.6 else 0
            wait(10)

        self.left_motor.stop()
        self.right_motor.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Tank Turn", target_angle)

    def turn_pivot(self, target_angle, speed=40, pivot_side="left"):
        """Rotate the robot on one wheel to a target heading using PID control."""
        print(f"\n--- Pivot Turn: {target_angle} ---")
        kp, ki, kd = 4.5, 0.12, 6.0
        last_error, integral, stable_count = self.get_shortest_error(target_angle), 0, 0
        
        while stable_count < 10:
            error = self.get_shortest_error(target_angle)
            integral = max(min(integral + error, self.I_CLAMP), -self.I_CLAMP)
            pwr = (error * kp) + (integral * ki) + (kd * (error - last_error))
            
            if abs(error) > 0.4:
                floor = self.TURN_FLOOR + 5
                if abs(pwr) < floor:
                    pwr = floor if pwr > 0 else -floor
            else:
                pwr = 0

            pwr = int(max(min(pwr, speed), -speed))
            if pivot_side.lower() == "left":
                self.left_motor.stop()
                self.right_motor.dc(-pwr)
            else:
                self.right_motor.stop()
                self.left_motor.dc(pwr)

            last_error = error
            stable_count = stable_count + 1 if abs(error) < 0.6 else 0
            wait(10)

        self.left_motor.stop()
        self.right_motor.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Pivot Turn", target_angle)

# --- EXECUTION ---
# --- FINAL STRESS TEST ---
bot = Robot()
bot.hub.imu.reset_heading(0)

# 1. Long high-speed sprint
bot.straight(1200, 85, target_heading=0) 

# 2. Fast 180-degree pivot (Testing KD damping)
bot.turn_tank(180, speed=80) 

# 3. Return sprint at different heading
bot.straight(1200, 85, target_heading=180)

# 4. Final precision alignment
bot.turn_tank(0, speed=50)

bot.print_diagnostic_report()