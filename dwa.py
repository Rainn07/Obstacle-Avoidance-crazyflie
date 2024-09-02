import numpy as np
import logging
import sys
import time
import csv
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper

# Configuration
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7')
DEFAULT_HEIGHT = 0.6
SAFETY_THRESHOLD = 0.3
LOGGING_INTERVAL = 0.1  # seconds

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class DynamicWindowApproach:
    def __init__(self, max_linear_vel, min_linear_vel, safety_threshold):
        self.max_linear_vel = max_linear_vel
        self.min_linear_vel = min_linear_vel
        self.safety_threshold = safety_threshold

    def evaluate_trajectory(self, current_pos, current_vel, waypoint, obstacles):
        distance_to_waypoint = np.sqrt((waypoint[0] - current_pos[0])**2 + (waypoint[1] - current_pos[1])**2)
        distance_to_obstacles = min([np.sqrt((obstacle[0] - current_pos[0])**2 + (obstacle[1] - current_pos[1])**2) for obstacle in obstacles], default=float('inf'))
        cost = distance_to_waypoint + 100.0 / (distance_to_obstacles + 0.01)
        return cost

    def find_best_velocity(self, current_pos, current_vel, waypoint, obstacles):
        best_cost = float('inf')
        best_vel = [0, 0]

        for vx in np.arange(self.min_linear_vel, self.max_linear_vel + 0.05, 0.05):
            for vy in np.arange(self.min_linear_vel, self.max_linear_vel + 0.05, 0.05):
                cost = self.evaluate_trajectory(current_pos, [vx, vy], waypoint, obstacles)
                if cost < best_cost:
                    best_cost = cost
                    best_vel = [vx, vy]

        return best_vel

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

    def move_through_waypoints(self, scf, waypoints):
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            with Multiranger(scf) as multiranger:
                with open('flight_log.csv', 'w', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(['Time', 'Front', 'Left', 'Right', 'Back', 'Up', 'X', 'Y', 'Vx', 'Vy', 'Battery'])
                    
                    # Initialize the DWA algorithm
                    dwa = DynamicWindowApproach(max_linear_vel=0.5, min_linear_vel=0.1, safety_threshold=SAFETY_THRESHOLD)

                    for waypoint in waypoints:
                        while True:
                            current_pos = self.position_estimate
                            current_vel = [0, 0]  # Assuming the drone can only move linearly

                            # Get the obstacle positions from the multiranger sensor
                            obstacle_positions = []
                            if multiranger.front is not None and multiranger.front < SAFETY_THRESHOLD:
                                obstacle_positions.append([current_pos[0] + multiranger.front, current_pos[1]])
                            if multiranger.left is not None and multiranger.left < SAFETY_THRESHOLD:
                                obstacle_positions.append([current_pos[0], current_pos[1] + multiranger.left])
                            if multiranger.right is not None and multiranger.right < SAFETY_THRESHOLD:
                                obstacle_positions.append([current_pos[0], current_pos[1] - multiranger.right])
                            if multiranger.back is not None and multiranger.back < SAFETY_THRESHOLD:
                                obstacle_positions.append([current_pos[0] - multiranger.back, current_pos[1]])

                            # Find the best linear velocities using the DWA algorithm
                            best_vel = dwa.find_best_velocity(current_pos, current_vel, waypoint, obstacle_positions)

                            # Move the drone based on the best velocities
                            mc.start_linear_motion(best_vel[0], best_vel[1], 0)

                            # Log sensor data and velocities
                            self.log_sensor_data(multiranger, csv_writer, best_vel[0], best_vel[1])
                            time.sleep(LOGGING_INTERVAL)

                            distance = ((waypoint[0] - current_pos[0])**2 + (waypoint[1] - current_pos[1])**2)**0.5
                            if distance < 0.1:
                                break

                    mc.stop()

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
