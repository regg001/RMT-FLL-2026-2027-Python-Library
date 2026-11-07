from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch


# ============================================================
# Robot Class  FLL 2026-2027
#
# CHANGE LOG (this file):
#   Merged from the two working copies. The newer copy is the
#     base; nothing from the older copy's turn logic or gains
#     was carried over (they were tuned for the old floor).
#   __init__ accepts optional attach_ports. Each port given is
#     registered with add_attachment(); empty ports are never
#     touched, so a missing motor can't crash startup.
#   turn_point is kept as an alias for turn_tank so older
#     mission files still run.
#   Worst Move in the diagnostic report now prints that move's
#     own signed error (was using the overall bias sign).
#   turn_curve default timeout raised to 2000 ms.
#   Unused sensor imports and the test main() removed.
#
#   Carried from the newer copy:
#     Speed capped at 100 in all turns (dc() is a percent).
#     Floor fades from full to MIN as error shrinks, and only
#       applies when power pushes toward the target.
#     Derivative is low-pass filtered (D_FILTER).
#     Deadband and settle band share one value (TURN_BAND).
#     Turns end with brake(); straight() ends with hold().
#     turn_pivot uses class constants instead of local gains.
#
# report_card entries are 4-tuples:
#     (move_name, target_angle, final_error, peak_load)
# blackbox.py must unpack 4 values to read them.
# ============================================================

class Robot:
    """Robot control wrapper for motion and PID heading corrections."""

    # ── Wheel Geometry ───────────────────────────────────────────────────────
    WHEEL_DIAMETER = 62.3
    AXLE_TRACK     = 114

    # ── Timing ───────────────────────────────────────────────────────────────
    SETTLE_TIME = 300

    # ── Floor Values ─────────────────────────────────────────────────────────
    STR_FLOOR = 25

    TURN_FLOOR       = 19   # tank floor far from target
    TURN_MIN_FLOOR   = 14   # tank floor at target (lowest dc that still rotates)
    PIVOT_FLOOR      = 28
    PIVOT_MIN_FLOOR  = 15
    CURVE_FLOOR      = 25
    CURVE_MIN_FLOOR  = 12
    FLOOR_ZONE       = 4    # degrees over which floor fades from full to MIN

    TURN_BAND = 0.7         # on-target band (deadband + settle check)
    D_FILTER  = 0.6         # derivative smoothing: higher = smoother, slower

    # ── Acceleration Ramp ────────────────────────────────────────────────────
    ACCEL_ZONE      = 100   # wheel degrees to ramp up over
    DECEL_ZONE      = 250   # wheel degrees (~136 mm) to slow down before the end
    STR_STOP_OFFSET = 2     # mm to stop early; set to the average overshoot you measure

    # ── Integral Clamps ──────────────────────────────────────────────────────
    STR_I_CLAMP  = 25
    TURN_I_CLAMP = 10

    # ── Straight PID ─────────────────────────────────────────────────────────
    # Output is a turn rate in deg/s fed to drive_base.drive().
    STR_KP = 8
    STR_KD = 15
    STR_KI = 0.3

    # ── Turn PID (tank) ──────────────────────────────────────────────────────
    TURN_KP = 2.5
    TURN_KD = 10
    TURN_KI = 0.01

    # ── Pivot PID ────────────────────────────────────────────────────────────
    PIVOT_KP = 3.0
    PIVOT_KD = 12
    PIVOT_KI = 0.01

    # ── Curve Turn PID ───────────────────────────────────────────────────────
    CURVE_KP = 3.0
    CURVE_KD = 8.0
    CURVE_KI = 0.06

    def __init__(self, left_port=Port.E, right_port=Port.A, attach_ports=()):
        """
        attach_ports: optional list of attachment ports, e.g. (Port.F, Port.B).
        Each is registered with CLOCKWISE direction. For a reversed motor,
        leave it out here and call add_attachment(port, Direction.COUNTERCLOCKWISE).
        """
        self.hub         = PrimeHub()
        self.timer       = StopWatch()
        self.report_card = []
        self.attachments = {}
        self.left_motor  = Motor(left_port,  Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(right_port, Direction.CLOCKWISE)
        self.drive_base  = DriveBase(
            self.left_motor, self.right_motor,
            wheel_diameter=self.WHEEL_DIAMETER,
            axle_track=self.AXLE_TRACK
        )
        self.drive_base.use_gyro(True)
        for port in attach_ports:
            self.add_attachment(port)

    def add_attachment(self, port, direction=Direction.CLOCKWISE):
        """Register an attachment motor so move_attachment() can use it."""
        self.attachments[port] = Motor(port, direction)

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
        Returns a percentage (0-100).
        """
        try:
            left_load  = abs(self.left_motor.load())
            right_load = abs(self.right_motor.load())
            return max(left_load, right_load) / 8.0
        except:
            return 0

    def _apply_floor(self, pwr, error, full_floor, min_floor):
        """
        Zero power inside TURN_BAND. Otherwise enforce a minimum power that
        fades from full_floor down to min_floor as error shrinks inside
        FLOOR_ZONE. Floor only applies when pwr pushes toward the target,
        so the D term can still brake freely.
        """
        if abs(error) < self.TURN_BAND:
            return 0
        fade  = min(1, abs(error) / self.FLOOR_ZONE)
        floor = min_floor + (full_floor - min_floor) * fade
        if abs(pwr) < floor and pwr * error > 0:
            return floor if pwr > 0 else -floor
        return pwr

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

    def sleepms(self, ms):
        wait(ms)

    def voltage_report(self):
        """Print the current battery level as a percent of 9V."""
        voltage_mV = self.hub.battery.voltage()
        battery = (voltage_mV / 1000) * 100 / 9
        battery = round(battery, 0)
        print(f"Battery voltage: {battery} %")

    # ────────────────────────────────────────────────────────────────────────
    # DIAGNOSTIC LOGGING
    # ────────────────────────────────────────────────────────────────────────

    def log_result(self, move_name, target, peak_load=0):
        """
        Append a move result to the report card.

        report_card entries are 4-tuples:
            (move_name, target_angle, final_error, peak_load)
        """
        final_err = self.get_shortest_error(target)
        self.report_card.append((move_name, round(float(target), 2), final_err, peak_load))

    def print_diagnostic_report(self, run=None, mission=None, voltage=None, elapsed=None):
        """
        Print a formatted post-run report to the terminal.

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

        worst_err  = 0   # signed error of the worst move
        worst_name = ""
        worst_tgt  = 0

        for name, target, err, peak_load in self.report_card:
            status = "OK" if abs(err) < 0.8 else "CHECK"
            print("{:15} | {:7} | {:+6.2f}° | {:7.0f}%  [{}]".format(
                name, target, err, peak_load, status))
            if abs(err) > abs(worst_err):
                worst_err  = err
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
            worst_name, worst_tgt, worst_err))

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
        """
        Drive straight for a distance while holding an absolute heading.

        Pass target_heading whenever you know the intended heading
        (e.g. the angle of the turn you just made). If left as None it
        holds whatever heading the robot is at now, which carries any
        leftover turn error into the drive.
        """
        if target_heading is None:
            target_heading = self.get_yaw()
        direction  = 1 if distance_mm > 0 else -1
        target_deg = self._mm_to_degrees(max(abs(distance_mm) - self.STR_STOP_OFFSET, 0))
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        last_error = self.get_shortest_error(target_heading)
        integral   = 0
        d_filt     = 0
        peak_load  = 0
        max_err    = 0
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
            error    = self.get_shortest_error(target_heading)
            integral = max(min(integral + error, self.STR_I_CLAMP), -self.STR_I_CLAMP)
            d_filt   = self.D_FILTER * d_filt + (1 - self.D_FILTER) * (error - last_error)
            correction = (self.STR_KP * error) + (self.STR_KI * integral) + (self.STR_KD * d_filt)
            self.drive_base.drive(direction * current_speed, correction)
            last_error = error
            peak_load  = max(peak_load, self._peak_load())
            if abs(error) > abs(max_err): max_err = error
            wait(10)
        err_at_stop = self.get_shortest_error(target_heading)
        self.drive_base.stop()
        self.left_motor.hold()
        self.right_motor.hold()
        wait(self.SETTLE_TIME)
        err_settled = self.get_shortest_error(target_heading)
        print("Straight heading: max during drive {:+.2f}° | at stop {:+.2f}° | after settle {:+.2f}°".format(
            max_err, err_at_stop, err_settled))

        traveled_deg = (abs(self.left_motor.angle()) + abs(self.right_motor.angle())) / 2
        traveled_mm  = traveled_deg * self.WHEEL_DIAMETER * 3.14159 / 360
        print("Straight: target {} mm | traveled {:.1f} mm | overshoot {:+.1f} mm".format(
            abs(distance_mm), traveled_mm, traveled_mm - abs(distance_mm)))
        self.log_result("Straight", target_heading, peak_load)

    def turn_tank(self, target_angle, speed=60, timeout=3000):
        """Point turn: both wheels drive in opposite directions."""
        speed        = min(abs(speed), 100)
        last_error   = self.get_shortest_error(target_angle)
        integral     = 0
        d_filt       = 0
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

            d_filt = self.D_FILTER * d_filt + (1 - self.D_FILTER) * (error - last_error)
            pwr = error * self.TURN_KP + integral * self.TURN_KI + d_filt * self.TURN_KD
            pwr = self._apply_floor(pwr, error, self.TURN_FLOOR, self.TURN_MIN_FLOOR)
            pwr = int(max(min(pwr, speed), -speed))

            self.left_motor.dc(pwr)
            self.right_motor.dc(-pwr)

            last_error   = error
            peak_load    = max(peak_load, self._peak_load())
            stable_count = stable_count + 1 if abs(error) < self.TURN_BAND else 0
            wait(10)

        self.left_motor.brake()
        self.right_motor.brake()
        wait(self.SETTLE_TIME)
        self.log_result("Tank Turn", target_angle, peak_load)

    # Older mission files call turn_point(); same method.
    turn_point = turn_tank

    def turn_pivot(self, target_angle, speed=80, pivot_side="left", timeout=5000):
        """Pivot turn: one wheel holds still, the other drives."""
        speed        = min(abs(speed), 100)
        last_error   = self.get_shortest_error(target_angle)
        integral     = 0
        d_filt       = 0
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

            d_filt = self.D_FILTER * d_filt + (1 - self.D_FILTER) * (error - last_error)
            pwr = error * self.PIVOT_KP + integral * self.PIVOT_KI + d_filt * self.PIVOT_KD
            pwr = self._apply_floor(pwr, error, self.PIVOT_FLOOR, self.PIVOT_MIN_FLOOR)
            pwr = int(max(min(pwr, speed), -speed))

            if pivot_side.lower() == "left":
                self.left_motor.brake()
                self.right_motor.dc(-pwr)
            else:
                self.right_motor.brake()
                self.left_motor.dc(pwr)

            last_error   = error
            peak_load    = max(peak_load, self._peak_load())
            stable_count = stable_count + 1 if abs(error) < self.TURN_BAND else 0
            wait(10)

        self.left_motor.brake()
        self.right_motor.brake()
        wait(self.SETTLE_TIME)
        self.log_result("Pivot Turn", target_angle, peak_load)

    def turn_curve(self, target_angle, speed=60, curve_ratio=0.4, timeout=2000):
        """
        Arc turn using both wheels at different speeds.

        The outer wheel runs at full PID-controlled speed.
        The inner wheel runs at speed * curve_ratio.

        curve_ratio controls the arc shape:
          0.0  ->  inner wheel stopped    (same as pivot turn)
          0.4  ->  gentle arc             (recommended default)
          0.7  ->  wide, gradual sweep
          1.0  ->  straight (no turn)

        Turn direction is locked in ONCE before the loop starts so an
        overshoot can't swap which wheel is outer vs inner.
        """
        speed        = min(abs(speed), 100)
        curve_ratio  = max(0.0, min(1.0, curve_ratio))
        last_error   = self.get_shortest_error(target_angle)
        integral     = 0
        d_filt       = 0
        stable_count = 0
        peak_load    = 0

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

            d_filt = self.D_FILTER * d_filt + (1 - self.D_FILTER) * (error - last_error)
            pwr = error * self.CURVE_KP + integral * self.CURVE_KI + d_filt * self.CURVE_KD
            pwr = self._apply_floor(pwr, error, self.CURVE_FLOOR, self.CURVE_MIN_FLOOR)
            pwr = int(max(min(pwr, speed), -speed))

            inner_pwr = int(pwr * curve_ratio)

            if turning_right:
                # left = outer, right = inner
                self.left_motor.dc(pwr)
                self.right_motor.dc(inner_pwr)
            else:
                # right = outer, left = inner
                self.right_motor.dc(-pwr)
                self.left_motor.dc(-inner_pwr)

            last_error   = error
            peak_load    = max(peak_load, self._peak_load())
            stable_count = stable_count + 1 if abs(error) < self.TURN_BAND else 0
            wait(10)

        self.left_motor.brake()
        self.right_motor.brake()
        wait(self.SETTLE_TIME)
        self.log_result("Curve Turn", target_angle, peak_load)

    # ────────────────────────────────────────────────────────────────────────
    # ATTACHMENTS
    # ────────────────────────────────────────────────────────────────────────

    def move_attachment(self, port, degrees, speed, wait_done=True, sleep=0, then=Stop.HOLD):
        if port not in self.attachments:
            print("WARNING: Port Not Defined")
            return
        self.attachments[port].run_angle(speed, degrees, then=then, wait=wait_done)
        if not wait_done:
            wait(sleep)

    def move_attachment_stalled(self, port, speed, torque_limit=40):
        if port not in self.attachments:
            print("WARNING: Port Not Defined")
            return
        self.attachments[port].run_until_stalled(speed, then=Stop.HOLD, duty_limit=torque_limit)

    def move_attachment_torque(self, port, speed, timeout=5000):
        """High-torque movement with safety timeout."""
        if port not in self.attachments:
            print("WARNING: Port Not Defined")
            return
        m = self.attachments[port]
        m.run(speed)
        wait(100)  # let it spin up before checking for stall
        self.timer.reset()
        while self.timer.time() < timeout:
            if abs(m.speed()) < 5:
                break
            wait(10)
        m.stop()
