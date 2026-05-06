from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch

class Robot:
    def __init__(self, left_port=Port.B, right_port=Port.D):
        self.hub = PrimeHub()
        self.left_motor = Motor(left_port, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(right_port, Direction.CLOCKWISE)
        self.timer = StopWatch()
        
        # Data Logging
        self.report_card = []

        # Global Constants
        self.TURN_FLOOR = 18    
        self.STR_FLOOR = 25     
        self.SETTLE_TIME = 500  

        # PID Gains
        self.STR_KP, self.STR_KD, self.STR_KI = 4.0, 10.0, 0.05
        self.TURN_KP, self.TURN_KD, self.TURN_KI = 3.0, 7.0, 0.06

    def get_yaw(self):
        return self.hub.imu.heading()

    def get_shortest_error(self, target_angle):
        error = target_angle - self.get_yaw()
        while error > 180: error -= 360
        while error < -180: error += 360
        return error

    def log_result(self, move_name, target):
        """Saves final accuracy. Rounding target to avoid long decimals."""
        final_err = self.get_shortest_error(target)
        # We use float() here to ensure no weird 'Heading' objects are passed
        self.report_card.append((move_name, round(float(target), 2), final_err))

    def print_diagnostic_report(self):
        print("\n" + "="*35)
        print("    HOME DIAGNOSTIC REPORT")
        print("="*35)
        print("{:15} | {:7} | {:5}".format("MOVE", "TARGET", "ERROR"))
        print("-" * 35)
        for name, target, err in self.report_card:
            status = "OK" if abs(err) < 0.8 else "STALL?"
            print("{:15} | {:7} | {:5.2f} [{}]".format(name, target, err, status))
        print("="*35 + "\n")

    def straight(self, target_distance_deg, speed, target_heading=None):
        if target_heading is None: target_heading = self.get_yaw()
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        last_error, integral = 0, 0
        ACCEL_ZONE, DECEL_ZONE = 50, 100 

        print("\n--- Straight: {} deg ---".format(target_distance_deg))

        while True:
            current_dist = (abs(self.left_motor.angle()) + abs(self.right_motor.angle())) / 2
            if current_dist >= abs(target_distance_deg): break
            
            if current_dist < ACCEL_ZONE:
                current_speed = max(self.STR_FLOOR, (current_dist / ACCEL_ZONE) * speed)
            elif (abs(target_distance_deg) - current_dist) < DECEL_ZONE:
                dist_remaining = abs(target_distance_deg) - current_dist
                current_speed = max(self.STR_FLOOR, (dist_remaining / DECEL_ZONE) * speed)
            else:
                current_speed = speed

            error = target_heading - self.get_yaw()
            integral = max(min(integral + error, 20), -20)
            derivative = error - last_error
            correction = (self.STR_KP * error) + (self.STR_KI * integral) + (self.STR_KD * derivative)
            
            self.left_motor.dc(int(current_speed + correction))
            self.right_motor.dc(int(current_speed - correction))

            if current_dist % 50 == 0:
                print("D:{:4.0f} | E:{:5.2f}".format(current_dist, error))

            last_error = error
            wait(10)

        self.left_motor.stop()
        self.right_motor.stop()
        wait(self.SETTLE_TIME)
        self.log_result("Straight", target_heading)

    def turn_tank(self, target_angle, speed=100):
        print("\n--- Tank Turn: {} ---".format(target_angle))
        last_error, integral, stable_count = self.get_shortest_error(target_angle), 0, 0
        while stable_count < 10:
            error = self.get_shortest_error(target_angle)
            integral = max(min(integral + error, 25), -25)
            pwr = (error * self.TURN_KP) + (integral * self.TURN_KI) + (self.TURN_KD * (error - last_error))
            
            if abs(error) > 0.5 and abs(pwr) < self.TURN_FLOOR:
                pwr = self.TURN_FLOOR if pwr > 0 else -self.TURN_FLOOR
            if abs(error) < 0.5: pwr = 0 

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
        print("\n--- Pivot Turn: {} ---".format(target_angle))
        kp, ki, kd = 3.5, 0.12, 6.0
        last_error, integral, stable_count = self.get_shortest_error(target_angle), 0, 0
        
        while stable_count < 10:
            error = self.get_shortest_error(target_angle)
            integral = max(min(integral + error, 30), -30)
            pwr = (error * kp) + (integral * ki) + (kd * (error - last_error))
            
            if abs(error) > 0.4:
                floor = self.TURN_FLOOR + 5 
                if abs(pwr) < floor: pwr = floor if pwr > 0 else -floor
            else: pwr = 0

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

bot.straight(800, 60)
bot.turn_tank(90)
bot.turn_pivot(0, pivot_side="right")
bot.straight(400, 40)

bot.print_diagnostic_report()