from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Color, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

class Robot:
    def __init__(self, left_port=Port.B, right_port=Port.D, wheel_diameter=56, axle_track=114):
        self.hub = PrimeHub()
        self.left_motor  = Motor(left_port, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(right_port, Direction.CLOCKWISE)
        
        # DriveBase used for internal calculations
        self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter, axle_track)
        self.drive_base.use_gyro(True)

        # --- CONSTANTS ---
        self.TURN_KP = 1.6
        self.TURN_KD = 2.2      
        self.TURN_FLOOR = 18    
        
        self.STR_KP = 9.0       
        self.STR_KD = 4.5   
        self.STR_FLOOR = 0      # Keep at 0; DriveBase handles forward movement    
        
        self.SETTLE_TIME = 500  # Pause in ms to let physical wobble die down

    def get_yaw(self):
        return self.hub.imu.heading()

    def get_shortest_error(self, target_angle):
        error = target_angle - self.get_yaw()
        while error > 180: error -= 360
        while error < -180: error += 360
        return error

    def turn_tank(self, target_angle, speed=150):
       
        print("Turning to {}...".format(target_angle))
        last_error = self.get_shortest_error(target_angle)
        stable_count = 0
        
        while stable_count < 8:
            error = self.get_shortest_error(target_angle)
            derivative = error - last_error
            
            pwr = (error * self.TURN_KP) + (derivative * self.TURN_KD)
            
            if abs(error) < 0.7: 
                pwr = 0
            else:
                if abs(pwr) < self.TURN_FLOOR:
                    pwr = self.TURN_FLOOR if pwr > 0 else -self.TURN_FLOOR
            
            pwr = int(max(min(pwr, speed), -speed))
            self.left_motor.dc(pwr)
            self.right_motor.dc(-pwr)

            last_error = error
            if abs(error) < 0.8: stable_count += 1
            else: stable_count = 0
            wait(10)

        self.left_motor.stop()
        wait(self.SETTLE_TIME) # Critical pause for gyro stability
        print("Turn Done. Error: {:.2f}".format(self.get_shortest_error(target_angle)))

    def gyro_straight(self, distance_mm, speed, accel=100, target_heading=None):
        """Precision Straight: High KP ensures no drifting."""
        if target_heading is None:
            target_heading = self.get_yaw()
            
        self.left_motor.reset_angle(0)
        self.drive_base.settings(straight_speed=speed, straight_acceleration=accel)
        
        last_error = 0
        target_deg = (distance_mm * 360) / (56 * 3.14159)
        
        while abs(self.left_motor.angle()) < abs(target_deg):
            error = self.get_shortest_error(target_heading)
            derivative = error - last_error
            
            correction = int((error * self.STR_KP) + (derivative * self.STR_KD))
            self.drive_base.drive(speed, correction)
            
            last_error = error
            wait(10)
            
        self.drive_base.stop()
        wait(self.SETTLE_TIME) # Pause before next maneuver
        print("Drive Done. Error: {:.2f}".format(self.get_shortest_error(target_heading)))

    def move_arm_until_stalled(self, port, speed, torque_limit=40):
        m = Motor(port)
        m.run_until_stalled(speed, then=Stop.HOLD, duty_limit=torque_limit)



# --- EXECUTION ---
robot = Robot()
robot.hub.imu.reset_heading(0)

# Run test
robot.turn_tank(-90)
robot.gyro_straight(200, 300, accel=80)
robot.turn_tank(90)
robot.gyro_straight(200,100)
