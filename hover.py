import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Connect to the Crazyflie
cflib.crtp.init_drivers()
cf = Crazyflie()
with SyncCrazyflie('radio://0/80/2M/E7E7E7E7E7', cf=cf) as scf:
    # Reset the Kalman filter and wait for the position to converge
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    # Use the MotionCommander to make the Crazyflie hover for 5 seconds
    with MotionCommander(scf) as motion_commander:
        motion_commander.up(1)
        time.sleep(5)
        motion_commander.down(1)
