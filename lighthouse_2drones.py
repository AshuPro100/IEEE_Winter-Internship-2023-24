import sys
import time
import math
import numpy

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper


# URI to the Crazyflie to connect to
uri1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
uri2 = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E7E7')


def wait_for_position_estimator(scf1,scf2):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf1, log_config) as logger1:
        for log_entry1 in logger1:
            data = log_entry1[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

    with SyncLogger(scf2, log_config) as logger2:
        for log_entry2 in logger2:
            data = log_entry2[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def reset_estimator(scf1,scf2):
    cf1 = scf1.cf
    cf2 = scf2.cf

    cf1.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf1.param.set_value('kalman.resetEstimation', '0')

    cf2.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf2.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf1,cf2)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def vector_substract(v0, v1):
    return [v0[0] - v1[0], v0[1] - v1[1], v0[2] - v1[2]]


def vector_add(v0, v1):
    return [v0[0] + v1[0], v0[1] + v1[1], v0[2] + v1[2]]


def run_sequence(scf1,scf2):
    cf1 = scf1.cf
    cf2 = scf2.cf

    # cf.param.set_value('ring.effect','0')
    center = 1.3
    
    with PositionHlCommander(
            scf1,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.5,
            default_height=1.0,
            controller=PositionHlCommander.CONTROLLER_PID) as pc1:
        with PositionHlCommander(
            scf2,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.5,
            default_height=1.0,
            controller=PositionHlCommander.CONTROLLER_PID) as pc2:
            time.sleep(5.0)

            setpoint1 = [-1, -1, 1]
            setpoint2 = [1, -1, 1]
            setpoint3 = [1, 1, 1]
            setpoint4 = [-1, 1, 1]

            pc1.go_to(setpoint1[0], setpoint1[1],setpoint1[2])
            pc2.go_to(setpoint3[0], setpoint3[1],setpoint3[2])
            time.sleep(3.0)
        

            pc1.go_to(setpoint2[0], setpoint2[1],setpoint2[2])
            pc2.go_to(setpoint4[0], setpoint4[1],setpoint4[2])
            time.sleep(0.5)
            pc1.go_to(setpoint3[0], setpoint3[1],setpoint3[2])
            pc2.go_to(setpoint1[0], setpoint1[1],setpoint1[2])
            time.sleep(0.5)
            pc1.go_to(setpoint4[0], setpoint4[1],setpoint4[2])
            pc2.go_to(setpoint2[0], setpoint2[1],setpoint2[2])
            time.sleep(0.5)
            pc1.go_to(setpoint1[0], setpoint1[1],setpoint1[2])
            pc2.go_to(setpoint3[0], setpoint3[1],setpoint3[2])

            pc1.go_to(setpoint1[0], setpoint1[1],0.5)
            pc2.go_to(setpoint3[0], setpoint3[1],0.5)
            time.sleep(0.5)
  
    
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        #time.sleep(0.5)
        
        


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri1, cf=Crazyflie(rw_cache='./cache')) as scf1:
        with SyncCrazyflie(uri2, cf=Crazyflie(rw_cache='./cache')) as scf2:
            reset_estimator(scf1,scf2)
            # start_position_printing(scf)
            run_sequence(scf1,scf2)
