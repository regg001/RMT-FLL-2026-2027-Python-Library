from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Color, Stop, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# ============================================================
# Robot Class — FLL 2026-2027
# ============================================================

class Robot:
    """Robot control wrapper for motion and PID heading corrections."""

    # ── Floor and Timing ─────────────────────────────────────────────────────
    TURN_FLOOR  = 18
    STR_FLOOR   = 25
    SETTLE_TIME = 500

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

    # ── Turn PID ─────────────────────────────────────────────────────────────
    TURN_KP = 3.0
    TURN_KD = 8.0
    TURN_KI = 0.06

    # ── Wheel Geometry ───────────────────────────────────────────────────────
    WHEEL_DIAMETER = 56
    AXLE_TRACK     = 114

    def __init__(self, left_port=Port.B, right_port=Port.D):
        self.hub = PrimeHub()
        self.left_motor  = Motor(left_port,  Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(right_port, Direction.CLOCKWISE)
        self.drive_base  = DriveBase(
            self.left_motor, self.right_motor,
            wheel_diameter=self.WHEEL_DIAMETER,
            axle_track=self.AXLE_TRACK
        )
        self.drive_base.use_gyro(True)
        self.timer       = StopWatch()
        self.report_card = []

    # ────────────────────────────────────────────────────────────────────────
    # GYRO HELPERS
    # ────────────────────────────────────────────────────────────────────────

    def get_yaw(self):
        return self.hub.imu.heading()

    def get_shortest_error(self, target_angle):
        error = target_angle - self.get_yaw()
        while error > 180:  error -= 360
        while error < -180: error += 360
        return error

    def gyro_reset(self):
        """Reset gyro heading to zero and clear the diagnostic report."""
        self.hub.imu.reset_heading(0)
        self.report_card = []
        self.hub.light.on(Color.GREEN)
        wait(300)
        self.hub.light.off()

    # ────────────────────────────────────────────────────────────────────────
    # DIAGNOSTIC LOGGING
    # ────────────────────────────────────────────────────────────────────────

    def log_result(self, move_name, target):
        final_err = self.get_shortest_error(target)
        self.report_card.append((move_name, round(float(target), 2), final_err))

    def print_diagnostic_report(self):
        if not self.report_card:
            print("No moves recorded yet.")
            return
        print("\n" + "=" * 35)
        print("    MISSION DIAGNOSTIC REPORT")
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
        if abs(bias) > 3:
            direction = "RIGHT" if bias > 0 else "LEFT"
            print(f"\nWARNING: {direction} bias of {abs(bias):.2f} deg detected")
        else:
            print("\nBias within acceptable range.")
        print("=" * 35 + "\n")

    # ────────────────────────────────────────────────────────────────────────
    # MOVEMENT METHODS
    # ────────────────────────────────────────────────────────────────────────

    def straight(self, distance_mm, speed, target_heading=None, timeout=10000):
        wait(200)
        if target_heading is None:
            target_heading = self.get_yaw()
        wheel_circumference = self.WHEEL_DIAMETER * 3.14159
        target_deg = abs((distance_mm / wheel_circumference) * 360)
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        last_error = 0
        integral   = 0
        direction  = 1 if distance_mm > 0 else -1
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
            correction = ((self.STR_KP * error) + (self.STR_KI * integral) + (self.STR_KD * derivative))
            self.drive_base.drive(direction * current_speed, correction)
            last_error = error
            wait(10)
        self.drive_base.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Straight", target_heading)

    def turn_tank(self, target_angle, speed=100, timeout=5000):
        last_error   = self.get_shortest_error(target_angle)
        integral     = 0
        stable_count = 0
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
            pwr = ((error * self.TURN_KP) + (integral * self.TURN_KI) + (derivative * self.TURN_KD))
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

    def turn_pivot(self, target_angle, speed=40, pivot_side="left", timeout=5000):
        kp, ki, kd   = 4.5, 0.12, 6.0
        last_error   = self.get_shortest_error(target_angle)
        integral     = 0
        stable_count = 0
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

    def move_attachment(self, port, degrees, speed, then=Stop.HOLD, wait_done=True):
        m = Motor(port)
        m.run_angle(speed, degrees, then=then, wait=wait_done)

    def move_attachment_stalled(self, port, speed, torque_limit=40):
        m = Motor(port)
        m.run_until_stalled(speed, then=Stop.HOLD, duty_limit=torque_limit)


def main():
    bot = Robot()

    # Quick startup flash to confirm hub is alive
    bot.hub.light.on(Color.BLUE)
    wait(500)
    bot.hub.light.off()


if __name__ == "__main__":
    main()