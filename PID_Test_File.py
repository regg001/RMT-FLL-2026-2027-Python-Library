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

    STR_KP = 7.0
    STR_KD = 10.0
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

    def straight(self, target_distance_deg, speed, target_heading=None):
        """Drive the robot straight for a given distance with PID heading correction."""
        if target_heading is None:
            target_heading = self.get_yaw()
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        last_error, integral = 0, 0
        accel_zone, decel_zone = 50, 100 

        print(f"\n--- Straight: {target_distance_deg} deg ---")

        while True:
            current_dist = (abs(self.left_motor.angle()) + abs(self.right_motor.angle())) / 2
            if current_dist >= abs(target_distance_deg): break
            
            # Ramping Logic
            if current_dist < accel_zone:
                current_speed = max(self.STR_FLOOR, (current_dist / accel_zone) * speed)
            elif (abs(target_distance_deg) - current_dist) < decel_zone:
                dist_remaining = abs(target_distance_deg) - current_dist
                current_speed = max(self.STR_FLOOR, (dist_remaining / decel_zone) * speed)
            else:
                current_speed = speed

            # FIX: Used get_shortest_error to prevent ±180 flip issues
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
        # FIX: Log the target_heading so the error actually means something
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
                pwr = self.TURN_FLOOR if pwr > 0 else -self.TURN_FLOOR
            if abs(error) < 0.5:
                pwr = 0

            pwr = int(max(min(pwr, speed), -speed))
            self.left_motor.dc(pwr)
            self.right_motor.dc(-pwr)

            last_error = error
            stable_count = stable_count + 1 if abs(error) < 0.6 else 0
            wait(10)

        # FIX: Standardized motor stop to include both motors
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
bot = Robot()
bot.hub.imu.reset_heading(0)

for i in range(4):
        bot.straight(600, 60, target_heading=0)
        bot.turn_tank(90)
        bot.straight(600, 60, target_heading=90)
        bot.turn_tank(180)
        bot.straight(600, 60, target_heading=180)
        bot.turn_tank(-90)
        bot.straight(600, 60, target_heading=-90)


bot.print_diagnostic_report()