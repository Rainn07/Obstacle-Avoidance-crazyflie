from controller import Robot, Motor, InertialUnit, GPS, Gyro, DistanceSensor
import numpy as np
import sys
sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller

# Constants
FLYING_ATTITUDE = 0.3
SAFETY_THRESHOLD = 300
V_AVOID = 0.2
WAYPOINT_THRESHOLD = 0.1

class CrazyflieController:
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.setup_devices()
        self.PID_CF = pid_velocity_fixed_height_controller()
        self.height_desired = FLYING_ATTITUDE
        self.past_time = self.robot.getTime()
        self.past_x_global = 0
        self.past_y_global = 0

    def setup_devices(self):
        # Initialize motors
        self.motors = [self.robot.getDevice(f"m{i+1}_motor") for i in range(4)]
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        # Initialize sensors
        self.imu = self.robot.getDevice("inertial_unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)

        # Distance sensors
        self.sensors = {
            'front': self.robot.getDevice("range_front"),
            'left': self.robot.getDevice("range_left"),
            'back': self.robot.getDevice("range_back"),
            'right': self.robot.getDevice("range_right")
        }
        for sensor in self.sensors.values():
            sensor.enable(self.timestep)

    def get_sensor_readings(self):
        return {name: sensor.getValue() for name, sensor in self.sensors.items()}

    def set_motor_speeds(self, motor_power):
        self.motors[0].setVelocity(-motor_power[0])
        self.motors[1].setVelocity(motor_power[1])
        self.motors[2].setVelocity(-motor_power[2])
        self.motors[3].setVelocity(motor_power[3])

    def avoid_obstacles(self):
        print("checking for obstacles")
        sensors = self.get_sensor_readings()
        print(sensors)
        vx, vy = 0, 0

        if sensors['front'] < SAFETY_THRESHOLD:
            print("obstacle in front")
            vy = -3 * V_AVOID  # Move right
        elif sensors['left'] < SAFETY_THRESHOLD:
            vy = -V_AVOID  # Move right
        elif sensors['right'] < SAFETY_THRESHOLD:
            vy = V_AVOID  # Move left
        elif sensors['back'] < SAFETY_THRESHOLD:
            vy = V_AVOID  # Move forward

        return vx, vy

    def move_to_waypoint(self, waypoint, is_final_waypoint=False):
        while self.robot.step(self.timestep) != -1:
            dt = self.robot.getTime() - self.past_time

            roll = self.imu.getRollPitchYaw()[0]
            pitch = self.imu.getRollPitchYaw()[1]
            yaw = self.imu.getRollPitchYaw()[2]
            yaw_rate = self.gyro.getValues()[2]
            altitude = self.gps.getValues()[2]

            x_global = self.gps.getValues()[0]
            v_x_global = (x_global - self.past_x_global) / dt
            y_global = self.gps.getValues()[1]
            v_y_global = (y_global - self.past_y_global) / dt

            cosyaw = np.cos(yaw)
            sinyaw = np.sin(yaw)
            v_x = v_x_global * cosyaw + v_y_global * sinyaw
            v_y = -v_x_global * sinyaw + v_y_global * cosyaw

            distance = np.linalg.norm(np.array(waypoint) - np.array([x_global, y_global]))
            if distance < WAYPOINT_THRESHOLD:
                print(f"Reached waypoint: {waypoint}")
                if is_final_waypoint:
                    self.stop_drone()
                break

            vx, vy = self.avoid_obstacles()

            if vx == 0 and vy == 0:  # No obstacles
                direction = np.array(waypoint) - np.array([x_global, y_global])
                normalized_direction = direction / np.linalg.norm(direction)
                vx, vy = normalized_direction * 0.25

            motor_power = self.PID_CF.pid(dt, vx, vy, 0, self.height_desired, roll, pitch, yaw_rate, altitude, v_x, v_y)
            self.set_motor_speeds(motor_power)

            self.past_time = self.robot.getTime()
            self.past_x_global = x_global
            self.past_y_global = y_global
            
    def stop_drone(self):
        print("Final waypoint reached. Stopping drone.")
        for motor in self.motors:
            motor.setVelocity(0.0)
        self.robot.step(1000)  # Pause to ensure the drone stops

    def run(self):
        waypoints = [
            [1.5, 0.0]#,
            #[0.0, 1.0],
            #[1.0, 1.0]
        ]

        for waypoint in waypoints:
            self.move_to_waypoint(waypoint)
            self.robot.step(1000)  # Small pause after each waypoint


if __name__ == '__main__':
    robot = Robot()
    controller = CrazyflieController(robot)
    controller.run()
