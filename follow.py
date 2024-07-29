import logging
import time
import cv2
import numpy as np
from cvzone.HandTrackingModule import HandDetector
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper 

# Initialize Crazyflie URI for Drone
URI1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')
#URI2 = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E7E1')
#URI3 = uri_helper.uri_from_env(default='radio://0/60/2M/E7E7E7E7E1')
URI4 = uri_helper.uri_from_env(default='radio://0/50/2M/E7E7E7E7E1')
#URI5 = uri_helper.uri_from_env(default='radio://0/40/2M/E7E7E7E7E1')
URI6 = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E1')
URI7 = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E7E1')
#URI8 = uri_helper.uri_from_env(default='radio://0/10/2M/E7E7E7E7E1')
#URI9 = uri_helper.uri_from_env(default='radio://0/5/2M/E7E7E7E7E1')


current_position1 = {'x': 0.0, 'y': 0.0, 'z': 0.0}
current_position2 = {'x': 0.0, 'y': 0.0, 'z': 0.0}
current_position3 = {'x': 0.0, 'y': 0.0, 'z': 0.0}
cflib.crtp.init_drivers()
logging.basicConfig(level=logging.ERROR)

# Initialize hand tracking
cap = cv2.VideoCapture(0)
detector = HandDetector(detectionCon=0.8, maxHands=2)
hand_grabbed = False
pointer = False
initialized = False
# Convert screen width to initial point
center_x = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) // 2
center_y = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) // 2


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

def main():
    global hand_grabbed
    global initialized
    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf1: 
        print('connecttehted')
        reset_estimator(scf1)
        print('connectd')
        
        hlc1 = scf1.cf.high_level_commander
        hlc1.go_to(0, -1, 1, 0, 2, relative=False)
        time.sleep(2)         
        try:
            
            while cap.isOpened():
                success, img = cap.read()
                if not success:
                    break

                img = cv2.flip(img, 1)
                hands, img = detector.findHands(img)

                hand1 = hands[0]
                #hand2 = hands[1]
                
                lmList1 = hand1['lmList']
                #lmList2 = hand2['lmlist']
                    
                fingers1 = detector.fingersUp(hand1)
                #fingers2 = detector.fingersUp(hand2)

                palm1 = lmList1[9]
                #palm2 = lmList2[9]
                if not initialized:
                    point3_xinitial = (palm1[0] - center_x) / 150
                    point3_yinitial = (palm1[1]) / 200
                    depthinitial = (lmList1[18][0]-lmList1[6][0])/25-2
                    P3 = np.array([point3_xinitial, 2 - point3_yinitial])
                    depthinitial = (lmList1[18][0]-lmList1[6][0])/25-2
                    initialized = True

                #xpoint3_x = (palm1[0] - center_x) / 150
                #xpoint3_y = (palm2[1]) / 200
                #X3 = np.array([xpoint3_x, 2 - xpoint3_y])
                #depthx = (lmList2[18][0]-lmList2[6][0])/25-2

                if hand1:
                    xpoint3_x = (palm1[0] - center_x) / 150
                    xpoint3_y = (palm1[1]) / 200
                    X3 = np.array([xpoint3_x, 2 - xpoint3_y])
                    depthx = (lmList1[18][0]-lmList1[6][0])/25-1
                    cv2.putText(img, f'X3x: {X3[0]:.2f}', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 
                    cv2.putText(img, f'X3y: {X3[1]:.2f}', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 
                    cv2.putText(img, f'P3x: {P3[0]:.2f}', (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 
                    cv2.putText(img, f'P3y: {P3[1]:.2f}', (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 
                    cv2.putText(img, f'depthx: {depthx:.2f}', (10, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 
                    cv2.putText(img, f'depthinitial: {depthinitial:.2f}', (10, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 

                    if fingers1 == [0, 0, 0, 0, 0]:
                        hand_grabbed = True
                    else:
                        if hand_grabbed:
                            hand_grabbed = False
                    if hand_grabbed:
                        
                        print("P3:", P3)
                        hlc1.go_to(.5+X3[0]-P3[0], depthx-depthinitial, 1.786, 0, 1, relative=False)

                    #if fingers1 == [0,1,0,0,0] & fingers2[0,0,0,0,0]:
                            #pointer = True
                            #hlc1.go_to(-X3[0], -depthx, X3[1], 0, 2, relative=False)

                    cv2.imshow('Hand Tracking', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            pass

        finally:
            hlc1.land(0,2)
            
            cap.release()
            cv2.destroyAllWindows()
if __name__ == "__main__":
    main()
