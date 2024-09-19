import logging
import sys
import time
from threading import Event
import csv # import csv to write to csv file

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/1/2M/E7E7E7E7E7')

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0]

take_off_z = .5 # take off to .5 m above ground
forward_distance = .2 # move forward .2 meters

file = 'file' # replace this with filename

def move_forward(scf):
    with MotionCommander(scf) as mc:
        mc.take_off(take_off_z)
        time.sleep(2) # wait 2 seconds
        mc.forward(forward_distance)
        time.sleep(2) # wait 2 seconds
        mc.land()

# def take_off_simple(scf):
#     with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
#         time.sleep(3)
#         mc.stop()

def log_pos_callback(timestamp, data, logconf):
    # print(data) # now don't print iti to terminal
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


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    with open(csv_file, 'w', newline ='') as csv:
        csv_writer = csv.writer(csv_file_handle)
        csv_writer.writerow(['Timestamp', 'X', 'Y'])

        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

            scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                            cb=param_deck_flow)
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
            move_forward(scf)
            logconf.stop()

            for timestamp, pos_x, pos_y in zip(logconf.timestamps, position_estimate[0], position_estimate[1]):
                csv_writer.writerow([timestamp, pos_x, pos_y])
