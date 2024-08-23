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

URI = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E7E7')

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5
deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0]

def perform_rotations(mc):
    # Rotate 90 degrees to the left
    mc.turn_left(90)
    time.sleep(1)
    
    # Rotate back to the center (90 degrees to the right)
    mc.turn_right(90)
    time.sleep(1)
    
    # Rotate 90 degrees to the right
    mc.turn_right(90)
    time.sleep(1)
    
    # Rotate back to the center (90 degrees to the left)
    mc.turn_left(90)
    time.sleep(1)

def take_off_and_rotate(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        # Take off and hold position at default height
        time.sleep(3)
        
        # Perform the series of rotations
        perform_rotations(mc)
        
        # Stop all motion and land
        mc.stop()

def move_to_waypoint(mc, waypoint, csv_writer):
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
        
        mc.start_linear_motion(velocity[0], velocity[1], 0)
        
        # Log data
        csv_writer.writerow([time.time() - start_time] + current_pos + velocity + [DEFAULT_HEIGHT])
        
        time.sleep(0.1)
    
    mc.stop()

def move_through_waypoints(scf, waypoints):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        with open('flight_log.csv', 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Time', 'X', 'Y', 'Vx', 'Vy', 'Z'])
            
            for waypoint in waypoints:
                move_to_waypoint(mc, waypoint, csv_writer)
                time.sleep(1)  # Pause at each waypoint


def move_box_limit(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        max_vel = 0.2
       
        while (1):
            #if position_estimate[0] > BOX_LIMIT:
            #    mc.start_back()
            #elif position_estimate[0] < -BOX_LIMIT:
            #    mc.start_forward()
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
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        #mc.up(0.3)
        time.sleep(3)
        mc.stop()

def log_pos_callback(timestamp,data, logconf):
    print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def move_linear_simple(scf):
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
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',cb=param_deck_flow)
        time.sleep(1)
        
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

        move_through_waypoints(scf, waypoints)

        logconf.stop()