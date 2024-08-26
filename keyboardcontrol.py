import logging
import time
import keyboard
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Initialize Crazyflie URI for Drones
URI1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')
URI4 = uri_helper.uri_from_env(default='radio://0/50/2M/E7E7E7E7E1')
URI6 = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E1')
URI7 = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E7E1')

cflib.crtp.init_drivers()
logging.basicConfig(level=logging.ERROR)

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

def main():
    
    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf1, \
         SyncCrazyflie(URI4, cf=Crazyflie(rw_cache='./cache')) as scf4, \
         SyncCrazyflie(URI6, cf=Crazyflie(rw_cache='./cache')) as scf6, \
         SyncCrazyflie(URI7, cf=Crazyflie(rw_cache='./cache')) as scf7: 
        
        print('Connected to Crazyflies')
        
        reset_estimator(scf1)
        reset_estimator(scf4)
        reset_estimator(scf6)
        reset_estimator(scf7)

        print('Estimators Reset')
        
        hlc1 = scf1.cf.high_level_commander
        hlc4 = scf4.cf.high_level_commander
        hlc6 = scf6.cf.high_level_commander
        hlc7 = scf7.cf.high_level_commander
        
        hlc1.go_to(.5, 0, 1, 0, 2, relative=False)
        hlc4.go_to(-.5, 0, 1, 0, 2, relative=False)
        hlc6.go_to(0, .5, 1, 0, 2, relative=False)
        hlc7.go_to(0, -.5, 1, 0, 2, relative=False)
        
        time.sleep(2)
        playing = True
        while playing:
            control_drone1 = keyboard.is_pressed('1')
            control_drone4 = keyboard.is_pressed('2')
            control_drone6 = keyboard.is_pressed('3')
            control_drone7 = keyboard.is_pressed('4')
            
            if control_drone1:
                selected_drones = [hlc1]
            elif control_drone4:
                selected_drones = [hlc4]
            elif control_drone6:
                selected_drones = [hlc6]
            elif control_drone7:
                selected_drones = [hlc7]
            else:
                selected_drones = [hlc1, hlc4, hlc6, hlc7]

            if keyboard.is_pressed('up'):
                for hlc in selected_drones:
                    hlc.go_to(0, .2, 0, 0, .3, relative=True)

            if keyboard.is_pressed('down'):
                for hlc in selected_drones:
                    hlc.go_to(0, -.2, 0, 0, .3, relative=True)

            if keyboard.is_pressed('left'):
                for hlc in selected_drones:
                    hlc.go_to(-.2, 0, 0, 0, .3, relative=True)

            if keyboard.is_pressed('right'):
                for hlc in selected_drones:
                    hlc.go_to(.2, 0, 0, 0, .3, relative=True)

            if keyboard.is_pressed('plus'):
                for hlc in selected_drones:
                    hlc.go_to(0, 0, .2, 0, .3, relative=True)

            if keyboard.is_pressed('enter'):
                for hlc in selected_drones:
                    hlc.go_to(0, 0, -.2, 0, .3, relative=True)

            if keyboard.is_pressed('space'):
                for hlc in selected_drones:
                    hlc.land(0, 2)

            # Exit loop if necessary
            if keyboard.is_pressed('q'):
                playing = False

if __name__ == "__main__":
    main()
