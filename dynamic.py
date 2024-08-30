import logging
import sys
import time
from threading import Event
import csv
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper

# Configuration
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
DEFAULT_HEIGHT = 0.6
BOX_LIMIT = 0.5
SAFETY_THRESHOLD = 0.3
V_AVOID = 0.2
DELTA_D = 0.2
LOGGING_INTERVAL = 0.1  # seconds

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class CrazyflieController:
    def __init__(self):
        self.position_estimate = [0, 0]
        self.deck_attached_event = Event()
        self.battery_level = 100  # Placeholder for battery level

    def log_sensor_data(self, multiranger, csv_writer, Vx, Vy):
        csv_writer.writerow([
            time.time(),
            multiranger.front,
            multiranger.left,
            multiranger.right,
            multiranger.back,
            multiranger.up,
            self.position_estimate[0],
            self.position_estimate[1],
            Vx,
            Vy,

            self.battery_level
        ])
    def log_yaw_callback(self, timestamp, data, logconf):
        """
        Callback function to update the current yaw value.
        """
        self.yaw = np.degrees(data['stateEstimate.yaw'])  # Convert from radians to degrees

    def get_current_yaw(self):
        """
        Returns the current yaw angle.
        """
        return self.yaw
    
    def perform_rotations(self, mc):
        rotations = [
            (mc.turn_left, 90),
            (mc.turn_right, 90),
            (mc.turn_right, 90),
            (mc.turn_left, 90)
        ]
        for rotation_func, angle in rotations:
            rotation_func(angle)
            time.sleep(1)

    def take_off_and_rotate(self, scf):
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            logger.info("Taking off and performing rotations")
            time.sleep(3)
            self.perform_rotations(mc)
            mc.stop()

    def avoid_obstacles(self, mc, multiranger):
            """
            Continuously checks for obstacles in all directions and moves the drone accordingly to avoid them.

            Parameters:
                mc (MotionCommander): The motion commander instance to control the drone.
                multiranger (MultiRanger): The MultiRanger instance for obstacle detection.
            """
            # Avoid obstacles in front
            while multiranger.front is not None and multiranger.front < SAFETY_THRESHOLD:
                logger.info("Obstacle detected in front, moving right")
                mc.right(V_AVOID)
                time.sleep(LOGGING_INTERVAL)

            # Avoid obstacles on the left
            while multiranger.left is not None and multiranger.left < SAFETY_THRESHOLD:
                logger.info("Obstacle detected on the left, moving right")
                mc.right(V_AVOID)
                time.sleep(LOGGING_INTERVAL)

            # Avoid obstacles on the right
            while multiranger.right is not None and multiranger.right < SAFETY_THRESHOLD:
                logger.info("Obstacle detected on the right, moving left")
                mc.left(V_AVOID)
                time.sleep(LOGGING_INTERVAL)

            # Avoid obstacles behind (if needed)
            while multiranger.back is not None and multiranger.back < SAFETY_THRESHOLD:
                logger.info("Obstacle detected behind, moving forward")
                mc.forward(V_AVOID)
                time.sleep(LOGGING_INTERVAL)


    def move_to_waypoint(self, mc, waypoint, multiranger, csv_writer):
        logger.info(f"Moving to waypoint: {waypoint}")

        while True:
            current_pos = self.position_estimate
            distance = ((waypoint[0] - current_pos[0])**2 + 
                        (waypoint[1] - current_pos[1])**2)**0.5

            logger.info(f"Distance to waypoint: {distance}")

            if distance < 0.1:
                logger.info(f"Reached waypoint: {waypoint}")
                break

            # Call the obstacle avoidance function
            self.avoid_obstacles(mc, multiranger)

            # Calculate direction towards the waypoint
            direction = [
                waypoint[0] - current_pos[0],
                waypoint[1] - current_pos[1]
            ]

            # Calculate the yaw angle required to face the waypoint
            target_yaw = np.degrees(np.arctan2(direction[1], direction[0]))

            # Get the current yaw and determine the yaw difference
            current_yaw = self.get_current_yaw()  # Implement this method based on your setup
            yaw_diff = target_yaw - current_yaw

            # Normalize yaw_diff to the range [-180, 180]
            yaw_diff = (yaw_diff + 180) % 360 - 180

            # Perform the yaw rotation
            if yaw_diff > 0:
                mc.turn_right(yaw_diff)
            else:
                mc.turn_left(-yaw_diff)
            time.sleep(1)

            # Normalize the direction and calculate velocity
            magnitude = (direction[0]**2 + direction[1]**2)**0.5
            normalized_direction = [d / magnitude for d in direction]
            velocity = [d * 0.2 for d in normalized_direction]

            # Move the drone towards the waypoint
            mc.start_linear_motion(velocity[0], velocity[1], 0)

            # Log sensor data and velocities
            self.log_sensor_data(multiranger, csv_writer, velocity[0], velocity[1])
            time.sleep(LOGGING_INTERVAL)

        mc.stop()


    def move_through_waypoints(self, scf, waypoints):
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            with Multiranger(scf) as multiranger:
                with open('flight_log.csv', 'w', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(['Time', 'Front', 'Left', 'Right', 'Back', 'Up', 'X', 'Y', 'Vx', 'Vy' 'Battery'])
                    
                    for waypoint in waypoints:
                        self.move_to_waypoint(mc, waypoint, multiranger, csv_writer)
                        time.sleep(1)

    def log_pos_callback(self, timestamp, data, logconf):
        self.position_estimate[0] = data['stateEstimate.x']
        self.position_estimate[1] = data['stateEstimate.y']
        logger.debug(f"Position update: {self.position_estimate}")

    def param_deck_flow(self, _, value_str):
        value = int(value_str)
        if value:
            self.deck_attached_event.set()
            logger.info('Flow deck is attached!')
        else:
            logger.warning('Flow deck is NOT attached!')

    def run(self):
        cflib.crtp.init_drivers()
        
        try:
            with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
                scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=self.param_deck_flow)
                time.sleep(1)
                
                logconf = LogConfig(name='Position', period_in_ms=10)
                logconf.add_variable('stateEstimate.x', 'float')
                logconf.add_variable('stateEstimate.y', 'float')
                scf.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self.log_pos_callback)

                if not self.deck_attached_event.wait(timeout=5):
                    logger.error('No flow deck detected!')
                    return

                logconf.start()

                waypoints = [
                    [2.0, 0.0],
                    [0.0, 2.0],
                    [2.0, 2.0]
                ]

                self.move_through_waypoints(scf, waypoints)

                logconf.stop()

        except Exception as e:
            logger.error(f"An error occurred: {e}")
            sys.exit(1)

if __name__ == '__main__':
    controller = CrazyflieController()
    controller.run()
