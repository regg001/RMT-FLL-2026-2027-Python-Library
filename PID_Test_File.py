from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Color, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# ============================================================
# Robot Class — FLL 2026-2027
# Improvements over previous version:
#   1. straight() uses DriveBase.drive() — speed-regulated, battery-independent
#   2. ACCEL_ZONE / DECEL_ZONE increased for meaningful ramp
#   3. Integral frozen near target in turn_tank() — prevents overshoot
#   4. Timeouts added to straight() and turn_tank() — no more infinite loops
#   5. print_diagnostic_report() crash fix — handles empty report_card
#   6. Separate I clamps for straight and turn
#   7. straight() accepts mm instead of degrees — cleaner mission code
#   8. __main__ typo fixed
#   9. gyro_reset() convenience method added
#  10. Net bias interpretation added to diagnostic report
# ============================================================

class Robot:
    """Robot control wrapper for motion and PID heading corrections."""

    # ── Floor and Timing ─────────────────────────────────────────────────────
    TURN_FLOOR  = 18
    STR_FLOOR   = 25
    SETTLE_TIME = 500

    # ── Acceleration Ramp ────────────────────────────────────────────────────
    # CHANGE #2: Increased from 100 — 300 deg ≈ 131mm | 400 deg ≈ 175mm
    ACCEL_ZONE  = 300
    DECEL_ZONE  = 400

    # ── Integral Clamps (CHANGE #6: Separate for straight vs turn) ───────────
    STR_I_CLAMP  = 25
    TURN_I_CLAMP = 10   # tighter — turns are more sensitive to windup

    # ── Straight PID ─────────────────────────────────────────────────────────
    STR_KP = 5.0
    STR_KD = 12.0
    STR_KI = 0.05

    # ── Turn PID ─────────────────────────────────────────────────────────────
    TURN_KP = 3.0
    TURN_KD = 8.0
    TURN_KI = 0.06

    # ── Wheel Geometry ───────────────────────────────────────────────────────
    WHEEL_DIAMETER = 56     # mm — update if you change wheels
    AXLE_TRACK     = 114    # mm — distance between wheel centers

    def __init__(self, left_port=Port.B, right_port=Port.D):
        self.hub = PrimeHub()

        self.left_motor  = Motor(left_port,  Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(right_port, Direction.CLOCKWISE)

        # CHANGE #1: DriveBase for speed-regulated driving
        self.drive_base = DriveBase(
            self.left_motor,
            self.right_motor,
            wheel_diameter=self.WHEEL_DIAMETER,
            axle_track=self.AXLE_TRACK
        )
        self.drive_base.use_gyro(True)

        self.timer = StopWatch()
        self.report_card = []

    # ────────────────────────────────────────────────────────────────────────
    # GYRO HELPERS
    # ────────────────────────────────────────────────────────────────────────

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

    # CHANGE #9: Gyro reset with visual confirmation + clears report card
    def gyro_reset(self):
        """Reset gyro heading to zero and clear the diagnostic report."""
        self.hub.imu.reset_heading(0)
        self.report_card = []
        self.hub.light.on(Color.GREEN)
        wait(300)
        self.hub.light.off()
        print("Gyro reset. Robot ready.")

    # ────────────────────────────────────────────────────────────────────────
    # DIAGNOSTIC LOGGING
    # ────────────────────────────────────────────────────────────────────────

    def log_result(self, move_name, target):
        """Save final accuracy against the target heading."""
        final_err = self.get_shortest_error(target)
        self.report_card.append((move_name, round(float(target), 2), final_err))

    def print_diagnostic_report(self):
        """Print a full diagnostic report of all moves this run."""

        # CHANGE #5: Crash fix — handle empty report card
        if not self.report_card:
            print("No moves recorded yet.")
            return

        print("\n" + "=" * 35)
        print("    HOME DIAGNOSTIC REPORT")
        print("=" * 35)
        print(f"{'MOVE':15} | {'TARGET':7} | {'ERROR':5}")
        print("-" * 35)

        for name, target, err in self.report_card:
            status = "OK" if abs(err) < 0.8 else "Check Me"
            print(f"{name:15} | {target:7} | {err:5.2f} [{status}]")

        print("=" * 35)

        total_err = sum(abs(e) for _, _, e in self.report_card)
        avg_err   = total_err / len(self.report_card)
        bias      = sum(e for _, _, e in self.report_card)

        print(f"Total Cumulative Error:    {total_err:.2f} deg")
        print(f"Average Error per Move:    {avg_err:.2f} deg")
        print(f"Net Bias (Systemic Drift): {bias:.2f} deg")

        # CHANGE #10: Bias interpretation with actionable advice
        if abs(bias) > 3:
            direction = "RIGHT" if bias > 0 else "LEFT"
            print(f"\nWARNING: Robot has a {direction} bias of {abs(bias):.2f} deg")
            print("Consider checking wheel traction or adjusting STR_KP")
        else:
            print("\nBias within acceptable range — no systemic drift detected.")

        print("=" * 35 + "\n")

    # ────────────────────────────────────────────────────────────────────────
    # MOVEMENT METHODS
    # ────────────────────────────────────────────────────────────────────────

    def straight(self, distance_mm, speed, target_heading=None, timeout=10000):
        """
        Drive straight for a given distance using PID heading correction.

        CHANGES APPLIED:
          #1 — Uses DriveBase.drive() — speed-regulated, battery-independent
          #2 — Larger ACCEL/DECEL zones for meaningful ramping
          #4 — Timeout prevents infinite loops
          #7 — Accepts mm instead of motor degrees

        Args:
            distance_mm:    distance in mm (negative = reverse)
            speed:          drive speed in mm/s
            target_heading: heading to maintain (None = lock current heading)
            timeout:        max ms before aborting (default 10000)
        """
        wait(200)

        if target_heading is None:
            target_heading = self.get_yaw()

        # CHANGE #7: Convert mm to degrees for internal distance tracking
        wheel_circumference = self.WHEEL_DIAMETER * 3.14159
        target_deg = abs((distance_mm / wheel_circumference) * 360)

        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)

        last_error = 0
        integral   = 0
        direction  = 1 if distance_mm > 0 else -1

        self.timer.reset()
        print(f"\n--- Straight: {distance_mm} mm ---")

        while True:
            current_dist = (abs(self.left_motor.angle()) + abs(self.right_motor.angle())) / 2

            if current_dist >= target_deg:
                break

            # CHANGE #4: Timeout guard
            if self.timer.time() > timeout:
                print("WARNING: straight() timed out — aborting move")
                break

            # CHANGE #2: Speed ramp with larger zones
            dist_remaining = target_deg - current_dist
            if current_dist < self.ACCEL_ZONE:
                current_speed = max(self.STR_FLOOR,
                                    (current_dist / self.ACCEL_ZONE) * speed)
            elif dist_remaining < self.DECEL_ZONE:
                current_speed = max(self.STR_FLOOR,
                                    (dist_remaining / self.DECEL_ZONE) * speed)
            else:
                current_speed = speed

            # PID heading correction
            error      = self.get_shortest_error(target_heading)
            integral   = max(min(integral + error, self.STR_I_CLAMP), -self.STR_I_CLAMP)
            derivative = error - last_error
            correction = ((self.STR_KP * error) +
                          (self.STR_KI * integral) +
                          (self.STR_KD * derivative))

            # CHANGE #1: DriveBase.drive() — regulated speed, not raw dc()
            self.drive_base.drive(direction * current_speed, correction)

            last_error = error
            wait(10)

        self.drive_base.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Straight", target_heading)
        print(f"Drive Done. Final Heading Error: {self.get_shortest_error(target_heading):.2f} deg")

    def turn_tank(self, target_angle, speed=100, timeout=5000):
        """
        Rotate the robot in place to a target heading using PID control.

        CHANGES APPLIED:
          #3 — Integral frozen inside 2 degrees — prevents windup overshoot
          #4 — Timeout prevents infinite loops
          #6 — Uses TURN_I_CLAMP (10) instead of shared clamp (25)

        Args:
            target_angle: absolute heading to turn to (degrees)
            speed:        max motor power (default 100)
            timeout:      max ms before aborting (default 5000)
        """
        print(f"\n--- Tank Turn: {target_angle} ---")

        last_error   = self.get_shortest_error(target_angle)
        integral     = 0
        stable_count = 0

        self.timer.reset()

        while stable_count < 10:

            # CHANGE #4: Timeout guard
            if self.timer.time() > timeout:
                print("WARNING: turn_tank() timed out — aborting turn")
                break

            error = self.get_shortest_error(target_angle)

            # CHANGE #3: Freeze integral near target to prevent overshoot
            # Accumulated integral at 2 degrees would push well past the target
            if abs(error) < 2.0:
                integral = 0
            else:
                integral = max(min(integral + error, self.TURN_I_CLAMP),
                               -self.TURN_I_CLAMP)

            derivative = error - last_error
            pwr = ((error      * self.TURN_KP) +
                   (integral   * self.TURN_KI) +
                   (derivative * self.TURN_KD))

            if abs(error) < 0.5:
                pwr = 0
            elif abs(pwr) < self.TURN_FLOOR:
                pwr = self.TURN_FLOOR if pwr > 0 else -self.TURN_FLOOR

            pwr = int(max(min(pwr, speed), -speed))
            self.left_motor.dc(pwr)
            self.right_motor.dc(-pwr)

            last_error   = error
            stable_count = stable_count + 1 if abs(error) < 0.6 else 0
            wait(10)

        self.left_motor.stop()
        self.right_motor.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Tank Turn", target_angle)
        print(f"Turn Done. Final Error: {self.get_shortest_error(target_angle):.2f} deg")

    def turn_pivot(self, target_angle, speed=40, pivot_side="left", timeout=5000):
        """
        Rotate on one wheel to a target heading using PID control.

        CHANGES APPLIED:
          #3 — Integral frozen inside 2 degrees (same fix as turn_tank)
          #4 — Timeout prevents infinite loops

        Args:
            target_angle: absolute heading to turn to (degrees)
            speed:        max motor power (default 40)
            pivot_side:   "left" = left wheel stationary | "right" = right stationary
            timeout:      max ms before aborting (default 5000)
        """
        print(f"\n--- Pivot Turn: {target_angle} ({pivot_side}) ---")

        kp, ki, kd   = 4.5, 0.12, 6.0
        last_error   = self.get_shortest_error(target_angle)
        integral     = 0
        stable_count = 0

        self.timer.reset()

        while stable_count < 10:

            # CHANGE #4: Timeout guard
            if self.timer.time() > timeout:
                print("WARNING: turn_pivot() timed out — aborting turn")
                break

            error = self.get_shortest_error(target_angle)

            # CHANGE #3: Freeze integral near target
            if abs(error) < 2.0:
                integral = 0
            else:
                integral = max(min(integral + error, self.TURN_I_CLAMP),
                               -self.TURN_I_CLAMP)

            derivative = error - last_error
            pwr = (error * kp) + (integral * ki) + (derivative * kd)

            if abs(error) < 0.4:
                pwr = 0
            elif abs(pwr) < self.TURN_FLOOR + 5:
                pwr = (self.TURN_FLOOR + 5) if pwr > 0 else -(self.TURN_FLOOR + 5)

            pwr = int(max(min(pwr, speed), -speed))

            if pivot_side.lower() == "left":
                self.left_motor.stop()
                self.right_motor.dc(-pwr)
            else:
                self.right_motor.stop()
                self.left_motor.dc(pwr)

            last_error   = error
            stable_count = stable_count + 1 if abs(error) < 0.6 else 0
            wait(10)

        self.left_motor.stop()
        self.right_motor.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Pivot Turn", target_angle)
        print(f"Pivot Done. Final Error: {self.get_shortest_error(target_angle):.2f} deg")

    def move_attachment(self, port, degrees, speed, then=Stop.HOLD, wait_done=True):
        """
        Move an attachment motor by a given number of degrees.

        Args:
            port:      motor port (e.g. Port.A)
            degrees:   rotation amount (positive = forward, negative = back)
            speed:     speed in deg/s
            then:      stop mode after move (Stop.HOLD, Stop.COAST, Stop.BRAKE)
            wait_done: True = blocking | False = background (non-blocking)
        """
        m = Motor(port)
        m.run_angle(speed, degrees, then=then, wait=wait_done)

    def move_attachment_stalled(self, port, speed, torque_limit=40):
        """
        Run an attachment motor until it physically stalls, then hold.
        Self-calibrating — no need to count degrees.

        Args:
            port:         motor port (e.g. Port.A)
            speed:        speed in deg/s (positive or negative sets direction)
            torque_limit: max power % — keep at 40 to protect motors
        """
        m = Motor(port)
        m.run_until_stalled(speed, then=Stop.HOLD, duty_limit=torque_limit)


# ============================================================
# MAIN
# CHANGE #8: Fixed __main__ typo (was "_main_" — missing underscores)
# CHANGE #9: gyro_reset() replaces raw imu.reset_heading(0)
# ============================================================

def main():
    bot = Robot()
    bot.gyro_reset()    # resets gyro + clears report card + green light flash

    # 1. Long high-speed sprint (~527mm = original 1200 motor degrees)
    bot.straight(527, 450, target_heading=0)

    # 2. Fast 180-degree tank turn
    bot.turn_tank(180, speed=80)

    # 3. Return sprint
    bot.straight(527, 450, target_heading=180)

    # 4. Final precision alignment
    bot.turn_tank(0, speed=50)

    # Full diagnostic report with bias analysis
    bot.print_diagnostic_report()


# CHANGE #8: Correct dunder name check
if __name__ == "__main__":
    main()