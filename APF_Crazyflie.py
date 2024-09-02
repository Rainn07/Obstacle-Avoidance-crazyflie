"""
An attempt at implementing the artificial potential field obstacle
avoidance algorithm
"""

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
SAFETY_THRESHOLD = 0.4
V_AVOID = 0.2
LOGGING_INTERVAL = 0.1  # seconds

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class CrazyflieController:
    def __init__(self):
        self.position_estimate = [0, 0]
        self.deck_attached_event = Event()
        self.battery_level = 100  # Placeholder for battery level
        self.kattract = 1.0
        self.krep = 0.5
        self.rmax = SAFETY_THRESHOLD

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

    def calculate_net_force(self, current_pos, waypoint, obstacles):
        """
        calculate the net force acting on the drone based on attractive and repulsive forces
        
        Args:
            * param current_pos(list):  current position of drone [x, y].
            * param waypoint(list):     current target waypoint [x, y].
            * obstacle(list):           list of obstacle positions [x, y].
        Returns:
        list: Net force vector [Fx, Fy].
        """
        logger.info("calculating net force")
        # step 1: attractive force:
        attractive = self.kattract * np.array([waypoint[0] - current_pos[0], waypoint[1]- current_pos[1]])
        # step 2: repulsive force:
        repulsive = ([0.0, 0.0])
        for obstacle in obstacles:
            distance = np.sqrt((obstacle[0] - current_pos[0])**2 + (obstacle[1] - current_pos[1]))
            # obstacle inside safety radius:
            if distance < self.rmax:
                rep = self.krep * (1/distance - 1/self.rmax) * (1/distance**2) * np.array([obstacle[0] - current_pos[0], 
                                                                                                 obstacle[1] - current_pos[1]])
                repulsive +=rep
        
        net_force = attractive + repulsive
        return net_force
    
    def avoid_obstacles(self, mc, current_pos, waypoint, multiranger):
        """
        actuates drone to avoid obstacles

        Args:
            * self(obj):            drone instance
            * mc(class):            motion commander
            * current_pos(list):    current position of the drone [x, y]
            * waypoint(list):       trget coordinates [x, y]
            * multiranger(obj):     distance sensor object
        """
        obstacles = []
        if multiranger.front is not None and multiranger.front < self.rmax:
            obstacles.append([current_pos[0] + multiranger.front, current_pos[1]])
        if multiranger.left is not None and multiranger.left < self.rmax:
            obstacles.append([current_pos[0], current_pos[1] + multiranger.left])
        if multiranger.right is not None and multiranger.right < self.rmax:
            obstacles.append([current_pos[0], current_pos[1] - multiranger.right])
        
        net_force = self.calculate_net_force(current_pos, waypoint, obstacles)
        
        # normailse net force and determine velocity
        magnitude = np.linalg.norm(net_force)
        if magnitude > 0:
            velocity = (net_force/magnitude) * V_AVOID
            mc.start_linear_motion(velocity[0], velocity[1], 0)
        else:
            mc.stop()

        return magnitude
    
    def move_to_waypoint(self, mc, waypoint, multiranger, csv_writer):
        logger.info(f"Moving to waypoint: {waypoint}")
        while True:
            current_pos = self.position_estimate
            distance = np.linalg.norm(np.array(waypoint) - np.array(current_pos))
            
            logger.info(f"Distance to waypoint: {distance}")
            if distance < 0.1:
                logger.info(f"Reached waypoint: {waypoint}")
                break

            # Call the obstacle avoidance function
            self.avoid_obstacles(mc, current_pos, waypoint, multiranger)

            # Log sensor data and velocities
            self.log_sensor_data(multiranger, csv_writer, 0, 0)
            time.sleep(LOGGING_INTERVAL)

        mc.stop()
    
    def move_through_waypoints(self, scf, waypoints):
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            with Multiranger(scf) as multiranger:
                with open('flight_log.csv', 'w', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(['Time', 'Front', 'Left', 'Right', 'Back', 'Up', 'X', 'Y', 'Vx', 'Vy', 'Battery'])
                    
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