import logging
import sys
import time
from threading import Event
import csv

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

# Constants and initial setup
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5
deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0]  # Placeholder for the drone's estimated position

def perform_rotations(mc):
    """Perform a series of 90-degree rotations."""
    rotations = [mc.turn_left, mc.turn_right, mc.turn_right, mc.turn_left]
    for rotate in rotations:
        rotate(90)
        time.sleep(1)

def take_off_and_rotate(scf):
    """Take off, perform rotations, and then land."""
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        perform_rotations(mc)
        mc.stop()

def move_to_waypoint(mc, waypoint, csv_writer):
    """Move the drone to a specific waypoint and log the data."""
    start_time = time.time()
    
    while True:
        current_pos = position_estimate
        distance = ((waypoint[0] - current_pos[0])**2 + 
                    (waypoint[1] - current_pos[1])**2)**0.5
        
        if distance < 0.1:  # If within 10cm of the waypoint
            break
        
        # Calculate direction vector
        direction = [
            waypoint[0] - current_pos[0],
            waypoint[1] - current_pos[1]
        ]
        
        # Normalize direction vector
        magnitude = (direction[0]**2 + direction[1]**2)**0.5
        normalized_direction = [d / magnitude for d in direction]
        
        # Set velocity (adjust the factor to change speed)
        velocity = [d * 0.2 for d in normalized_direction]
        
        # Move the drone
        mc.start_linear_motion(velocity[0], velocity[1], 0)
        
        # Log the data
        csv_writer.writerow([time.time() - start_time] + current_pos + velocity + [DEFAULT_HEIGHT])
        
        time.sleep(0.1)
    
    mc.stop()

def move_through_waypoints(scf, waypoints):
    """Move the drone through a series of waypoints."""
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        with open('flight_log.csv', 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Time', 'X', 'Y', 'Vx', 'Vy', 'Z'])
            
            for waypoint in waypoints:
                move_to_waypoint(mc, waypoint, csv_writer)
                time.sleep(1)  # Pause at each waypoint

def move_box_limit(scf):
    """Move the drone within a box limit, reversing direction upon reaching the edges."""
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        max_vel = 0.2
        
        while True:
            if position_estimate[0] > BOX_LIMIT:
                body_x_cmd = -max_vel
            elif position_estimate[0] < -BOX_LIMIT:
                body_x_cmd = max_vel
            if position_estimate[1] > BOX_LIMIT:
                body_y_cmd = -max_vel
            elif position_estimate[1] < -BOX_LIMIT:
                body_y_cmd = max_vel
            
            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)
            time.sleep(0.1)

def take_off_simple(scf):
    """Simple take off, hover, and land."""
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()

def log_pos_callback(timestamp, data, logconf):
    """Callback to update the position estimate based on logged data."""
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']

def param_deck_flow(_, value_str):
    """Callback to handle flow deck attachment."""
    value = int(value_str)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def move_linear_simple(scf):
    """Simple linear movements with rotations."""
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=param_deck_flow)
        time.sleep(1)
        
        # Configure and start logging position data
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)
        
        logconf.start()

        # Define waypoints (X, Y only)
        waypoints = [
            [1.0, 0.0],
            [1.0, 1.0],
            [-1.0, -1.0]
        ]

        # Move through the defined waypoints
        move_through_waypoints(scf, waypoints)

        logconf.stop()
