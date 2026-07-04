from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Color, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# ============================================================
# Robot Class — FLL 2026-2027  (merged: structure from doc12 +
# working telemetry/_peak_load from doc13. Kalman intentionally
# kept separate / not included here.)
#
# CHANGE LOG (this file):
#   - turn_curve(): outer/inner wheel direction is now locked
#     ONCE before the loop starts, instead of being re-checked
#     every iteration. Re-checking caused an instant motor-role
#     flip if the robot overshot and error crossed zero, which
#     prevented small curve turns from settling (timeout +
#     leftover error).
# ============================================================

class Robot:
    """Robot control wrapper for motion and PID heading corrections."""

    # ── Wheel Geometry ───────────────────────────────────────────────────────
    WHEEL_DIAMETER = 62.3
    AXLE_TRACK     = 114

    # ── Timing ───────────────────────────────────────────────────────────────
    SETTLE_TIME = 500

    # ── Floor Values ─────────────────────────────────────────────────────────
    TURN_FLOOR  = 19
    STR_FLOOR   = 25
    PIVOT_FLOOR = 20
    # ── Acceleration Ramp ────────────────────────────────────────────────────
    ACCEL_ZONE  = 300
    DECEL_ZONE  = 400

    # ── Integral Clamps ──────────────────────────────────────────────────────
    STR_I_CLAMP  = 25
    TURN_I_CLAMP = 10

    # ── Straight PID ─────────────────────────────────────────────────────────
    STR_KP = 5.0
    STR_KD = 12.0
    STR_KI = 0.05

    # ── Turn PID (tank + pivot) ──────────────────────────────────────────────
    TURN_KP = 3.0
    TURN_KD = 8.0
    TURN_KI = 0.06

    # ── Curve Turn PID (independent — tune separately after mat testing) ──────
    CURVE_KP = 3.0
    CURVE_KD = 8.0
    CURVE_KI = 0.06

    def __init__(self, left_port=Port.E, right_port=Port.A):
        self.hub         = PrimeHub()
        self.timer       = StopWatch()
        self.report_card = []
        self.left_motor  = Motor(left_port,  Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(right_port, Direction.CLOCKWISE)
        self.drive_base  = DriveBase(
            self.left_motor, self.right_motor,
            wheel_diameter=self.WHEEL_DIAMETER,
            axle_track=self.AXLE_TRACK
        )
        self.drive_base.use_gyro(True)

    # ────────────────────────────────────────────────────────────────────────
    # PRIVATE HELPERS
    # ────────────────────────────────────────────────────────────────────────

    def _mm_to_degrees(self, distance_mm):
        """Convert a linear distance in mm to wheel rotation in degrees."""
        circumference = self.WHEEL_DIAMETER * 3.14159
        return abs((distance_mm / circumference) * 360)

    def _peak_load(self):
        """
        Sample the highest motor load across both drive motors.
        Called each loop iteration inside movement methods to track
        the worst-case mechanical stress seen during a move.
        Returns a percentage (0-100).
        """
        try:
            left_load  = abs(self.left_motor.load())
            right_load = abs(self.right_motor.load())
            return max(left_load, right_load) / 8.0
        except:
            return 0

    # ────────────────────────────────────────────────────────────────────────
    # GYRO HELPERS
    # ────────────────────────────────────────────────────────────────────────

    def get_yaw(self):
        """Return the robot's current heading in degrees from the IMU."""
        return self.hub.imu.heading()

    def get_shortest_error(self, target_angle):
        """
        Return the shortest angular distance between current heading and target.
        Wraps error into -180 to +180 so the robot always takes the short way.
        """
        error = target_angle - self.get_yaw()
        while error > 180:  error -= 360
        while error < -180: error += 360
        return error

    def gyro_reset(self):
        """Reset IMU heading to zero and clear the report card."""
        self.hub.imu.reset_heading(0)
        self.report_card = []
        self.hub.light.on(Color.GREEN)
        wait(300)
        self.hub.light.off()

    # ────────────────────────────────────────────────────────────────────────
    # DIAGNOSTIC LOGGING
    # ────────────────────────────────────────────────────────────────────────

    def log_result(self, move_name, target, peak_load=0):
        """
        Append a move result to the report card.

        report_card entries are 4-tuples:
            (move_name, target_angle, final_error, peak_load)

        peak_load is the highest motor load % seen during the move.
        """
        final_err = self.get_shortest_error(target)
        self.report_card.append((move_name, round(float(target), 2), final_err, peak_load))

    def print_diagnostic_report(self, run=None, mission=None, voltage=None, elapsed=None):
        """
        Print a formatted post-run report to the terminal.

        Optional args pull in context from BlackBox for a richer header.
        Call with no args for a basic report, or pass BlackBox values:

            bot.print_diagnostic_report(
                run     = box.run_count,
                mission = box.current_mission,
                voltage = box.current_voltage,
                elapsed = box.timer.time()
            )
        """
        if not self.report_card:
            print("No moves recorded yet.")
            return

        W = 52  # report width

        print("\n" + "=" * W)
        print("    MISSION DIAGNOSTIC REPORT")

        if any(v is not None for v in [run, mission, voltage, elapsed]):
            parts = []
            if run     is not None: parts.append("Run #{}".format(run))
            if mission is not None: parts.append(mission)
            if voltage is not None: parts.append("{}mv".format(voltage))
            if elapsed is not None: parts.append("{:.1f}s".format(elapsed / 1000))
            print("    {}".format(" | ".join(parts)))

        print("=" * W)

        print("{:15} | {:7} | {:7} | {:9}".format(
            "MOVE", "TARGET", "ERROR", "PEAK LOAD"))
        print("-" * W)

        worst_err  = 0
        worst_name = ""
        worst_tgt  = 0

        for name, target, err, peak_load in self.report_card:
            status = "OK" if abs(err) < 0.8 else "CHECK"
            print("{:15} | {:7} | {:+6.2f}° | {:7.0f}%  [{}]".format(
                name, target, err, peak_load, status))
            if abs(err) > worst_err:
                worst_err  = abs(err)
                worst_name = name
                worst_tgt  = target

        print("=" * W)
        total_err = sum(abs(e) for _, _, e, _ in self.report_card)
        avg_err   = total_err / len(self.report_card)
        bias      = sum(e for _, _, e, _ in self.report_card)
        avg_load  = sum(l for _, _, _, l in self.report_card) / len(self.report_card)

        print("Total Cumulative Error:    {:.2f}°".format(total_err))
        print("Average Error per Move:    {:.2f}°".format(avg_err))
        print("Net Bias (Systemic Drift): {:+.2f}°".format(bias))
        print("Average Peak Load:         {:.0f}%".format(avg_load))

        print("-" * W)
        print("Worst Move: {} -> {}° | error {:+.2f}°".format(
            worst_name, worst_tgt, worst_err if bias >= 0 else -worst_err))

        if abs(bias) > 3:
            direction = "RIGHT" if bias > 0 else "LEFT"
            print("WARNING: {} bias of {:.2f}° — check TURN_FLOOR".format(
                direction, abs(bias)))
        else:
            print("Bias within acceptable range.")

        if avg_load > 60:
            print("WARNING: High motor load ({:.0f}%) — check for resistance".format(avg_load))

        if voltage is not None and voltage < 7400:
            print("WARNING: Low battery ({}mv) — consider swapping".format(voltage))

        print("=" * W + "\n")

    # ────────────────────────────────────────────────────────────────────────
    # MOVEMENT METHODS
    # ────────────────────────────────────────────────────────────────────────

    def straight(self, distance_mm, speed, target_heading=None, timeout=10000):
        """Drive straight for a distance while maintaining heading."""
        wait(200)
        if target_heading is None:
            target_heading = self.get_yaw()
        target_deg = self._mm_to_degrees(distance_mm)
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        last_error = 0
        integral   = 0
        peak_load  = 0
        self.timer.reset()
        while True:
            current_dist = (abs(self.left_motor.angle()) + abs(self.right_motor.angle())) / 2
            if current_dist >= target_deg: break
            if self.timer.time() > timeout:
                print("WARNING: straight() timed out")
                break
            dist_remaining = target_deg - current_dist
            if current_dist < self.ACCEL_ZONE:
                current_speed = max(self.STR_FLOOR, (current_dist / self.ACCEL_ZONE) * speed)
            elif dist_remaining < self.DECEL_ZONE:
                current_speed = max(self.STR_FLOOR, (dist_remaining / self.DECEL_ZONE) * speed)
            else:
                current_speed = speed
            error      = self.get_shortest_error(target_heading)
            integral   = max(min(integral + error, self.STR_I_CLAMP), -self.STR_I_CLAMP)
            derivative = error - last_error
            correction = (self.STR_KP * error) + (self.STR_KI * integral) + (self.STR_KD * derivative)
            self.drive_base.drive((1 if distance_mm > 0 else -1) * current_speed, correction)
            last_error = error
            peak_load  = max(peak_load, self._peak_load())
            wait(10)
        self.drive_base.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Straight", target_heading, peak_load)

    def turn_tank(self, target_angle, speed=100, timeout=5000):
        last_error   = self.get_shortest_error(target_angle)
        integral     = 0
        stable_count = 0
        peak_load    = 0
        self.timer.reset()
        while stable_count < 10:
            if self.timer.time() > timeout:
                print("WARNING: turn_tank() timed out")
                break
            error = self.get_shortest_error(target_angle)
            if abs(error) < 2.0:
                integral = 0
            else:
                integral = max(min(integral + error, self.TURN_I_CLAMP), -self.TURN_I_CLAMP)
            derivative = error - last_error
            pwr = (error * self.TURN_KP) + (integral * self.TURN_KI) + (derivative * self.TURN_KD)
            if abs(error) < 0.5:
                pwr = 0
            elif abs(pwr) < self.TURN_FLOOR:
                pwr = self.TURN_FLOOR if pwr > 0 else -self.TURN_FLOOR
            pwr = int(max(min(pwr, speed), -speed))
            self.left_motor.dc(pwr)
            self.right_motor.dc(-pwr)
            last_error   = error
            peak_load    = max(peak_load, self._peak_load())
            stable_count = stable_count + 1 if abs(error) < 0.6 else 0
            wait(10)
        self.left_motor.stop()
        self.right_motor.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Tank Turn", target_angle, peak_load)

    def turn_pivot(self, target_angle, speed=80, pivot_side="left", timeout=5000):
        kp, ki, kd   = 4.5, 0.12, 6.0
        last_error   = self.get_shortest_error(target_angle)
        integral     = 0
        stable_count = 0
        peak_load    = 0
        self.timer.reset()
        while stable_count < 10:
            if self.timer.time() > timeout:
                print("WARNING: turn_pivot() timed out")
                break
            error = self.get_shortest_error(target_angle)
            if abs(error) < 2.0:
                integral = 0
            else:
                integral = max(min(integral + error, self.TURN_I_CLAMP), -self.TURN_I_CLAMP)
            derivative = error - last_error
            pwr = (error * kp) + (integral * ki) + (derivative * kd) 
            if abs(error) < 0.8:
                pwr = 0
            elif abs(pwr) < self.PIVOT_FLOOR:
                pwr = self.PIVOT_FLOOR if pwr > 0 else -self.PIVOT_FLOOR
            pwr = int(max(min(pwr, speed), -speed))
            if pivot_side.lower() == "left":
                self.left_motor.stop()
                self.right_motor.dc(-pwr)
            else:
                self.right_motor.stop()
                self.left_motor.dc(pwr)
            last_error   = error
            peak_load    = max(peak_load, self._peak_load())
            stable_count = stable_count + 1 if abs(error) < 0.6 else 0
            wait(10)
        self.left_motor.stop()
        self.right_motor.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Pivot Turn", target_angle, peak_load)

    def turn_curve(self, target_angle, speed=150, curve_ratio=0.4, timeout=5000):
        """
        Arc turn using both wheels at different speeds.

        The outer wheel runs at full PID-controlled speed.
        The inner wheel runs at speed * curve_ratio, producing a
        smooth arc rather than a point turn or pivot.

        curve_ratio controls the arc shape:
          0.0  ->  inner wheel stopped    (same as pivot turn)
          0.4  ->  gentle arc             (recommended default)
          0.7  ->  wide, gradual sweep
          1.0  ->  straight (no turn)

        Turn direction is determined automatically from target_angle
        relative to current heading, and is locked in ONCE before the
        loop starts. It is not re-checked every iteration — if the
        robot overshoots and error crosses zero mid-turn, re-checking
        the sign would instantly swap which wheel is "outer" vs
        "inner", which prevents the turn from settling (previously
        seen as timeouts + leftover error on small curve targets).
        """
        curve_ratio  = max(0.0, min(1.0, curve_ratio))
        last_error   = self.get_shortest_error(target_angle)
        integral     = 0
        stable_count = 0
        peak_load    = 0

        # Decide turn direction ONCE, before the loop starts.
        turning_right = last_error > 0

        self.timer.reset()

        while stable_count < 10:
            if self.timer.time() > timeout:
                print("WARNING: turn_curve() timed out")
                break

            error = self.get_shortest_error(target_angle)

            if abs(error) < 2.0:
                integral = 0
            else:
                integral = max(min(integral + error, self.TURN_I_CLAMP), -self.TURN_I_CLAMP)

            derivative = error - last_error
            pwr = (error * self.CURVE_KP) + (integral * self.CURVE_KI) + (derivative * self.CURVE_KD)

            if abs(error) < 0.5:
                pwr = 0
            elif abs(pwr) < self.TURN_FLOOR:
                pwr = self.TURN_FLOOR if pwr > 0 else -self.TURN_FLOOR

            pwr = int(max(min(pwr, speed), -speed))

            # Inner wheel is a fixed ratio of outer wheel power
            inner_pwr = int(pwr * curve_ratio)

            if turning_right:
                # Turning right: left = outer, right = inner
                self.left_motor.dc(pwr)
                self.right_motor.dc(inner_pwr)
            else:
                # Turning left: right = outer, left = inner
                self.right_motor.dc(-pwr)
                self.left_motor.dc(-inner_pwr)

            last_error   = error
            peak_load    = max(peak_load, self._peak_load())
            stable_count = stable_count + 1 if abs(error) < 0.6 else 0
            wait(10)

        self.left_motor.stop()
        self.right_motor.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Curve Turn", target_angle, peak_load)

    def move_attachment(self, port, degrees, speed, then=Stop.HOLD, wait_done=True):
        m = Motor(port)
        m.run_angle(speed, degrees, then=then, wait=wait_done)

    def move_attachment_stalled(self, port, speed, torque_limit=40):
        m = Motor(port)
        m.run_until_stalled(speed, then=Stop.HOLD, duty_limit=torque_limit)


def main():
    bot = Robot()
    bot.gyro_reset()
    bot.straight(200, 400)
    bot.turn_tank(90)
    bot.turn_pivot(0, pivot_side="right")
    bot.turn_curve(75)
    bot.move_attachment(Port.B , 500 , 500 )
    bot.print_diagnostic_report()


if __name__ == "__main__":
    main()
