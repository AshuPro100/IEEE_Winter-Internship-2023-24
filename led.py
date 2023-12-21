import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
import random

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

sequence = [
    (0.0, 0.0, 0.25, 0),
    (0.0, 0.0, 0.5, 0),
    (0.1, 0.1, 0.5, 0),
    (0.0, 0.2, 0.5, 0),
    (-0.1, 0.1, 0.5, 0),
    (0.0, 0.0, 0.5, 0),
    (0.1, -0.1, 0.5, 0),
    (0.0, -0.2, 0.5, 0),
    (-0.1, -0.1, 0.5, 0),
    (0.0, 0.0, 0.5, 0),
    (0.0, 0.0, 0.25, 0),
    (0.0, 0.0, 0.05, 0),
]
scaling = (2.0, 2.0, 1.0)
offset = (0.0, 0.0, 0.0)


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

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

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

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

def run_sequence(scf, sequence):
    cf = scf.cf

    for position in sequence:
        print('Setting position {}'.format(position))
        cf.param.set_value('ring.solidRed', str(random.randint(0, 255)))
        cf.param.set_value('ring.solidGreen', str(random.randint(0, 255)))
        cf.param.set_value('ring.solidBlue', str(random.randint(0, 255)))
        for i in range(10):
            cf.commander.send_position_setpoint(position[0] * scaling[0] + offset[0],
                                                position[1] * scaling[1] + offset[1],
                                                position[2] * scaling[2] + offset[2],
                                                position[3])
            time.sleep(0.1)

    # Turn off the lights (set RGB values to 0)
    cf.param.set_value('ring.solidRed', '0')
    cf.param.set_value('ring.solidGreen', '0')
    cf.param.set_value('ring.solidBlue', '0')

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # Set solid color effect
        cf.param.set_value('ring.effect', '7')
        # Set the RGB values
        cf.param.set_value('ring.solidRed', '0')
        cf.param.set_value('ring.solidGreen', '200')
        cf.param.set_value('ring.solidBlue', '150')

        reset_estimator(scf)
        # start_position_printing(scf)
        run_sequence(scf, sequence)