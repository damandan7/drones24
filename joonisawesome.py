import logging
import time
import cv2
import math
import numpy as np
from cvzone.HandTrackingModule import HandDetector
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Initialize Crazyflie URI for Drone
URI1 = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E7E1')
URI2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')



cflib.crtp.init_drivers()
logging.basicConfig(level=logging.ERROR)

# Initialize hand tracking
cap = cv2.VideoCapture(0)
detector = HandDetector(detectionCon=0.8, maxHands=1)
hand_grabbed = False

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

    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf1, SyncCrazyflie(URI2, cf=Crazyflie(rw_cache='./cache')) as scf2:
        
        reset_estimator(scf1)
        reset_estimator(scf2)

        # Take off
        hlc1 = scf1.cf.high_level_commander
        hlc2 = scf2.cf.high_level_commander
        hlc1.go_to(0.5, 0, 1, 0, 1, relative=False)
        hlc2.go_to(0, 0, 1, 0, 1, relative=False)
        time.sleep(2)  

        try:
            while cap.isOpened():
                success, img = cap.read()
                if not success:
                    break

                img = cv2.flip(img, 1)
                hands, img = detector.findHands(img)

                if hands:
                    hand = hands[0]
                    lmList = hand['lmList']
                    fingers = detector.fingersUp(hand)
                    palm = lmList[9]

                    if fingers == [0, 0, 0, 0, 0]:
                        hand_grabbed = True
                        point3_x = (palm[0] - center_x) / 150
                        point3_y = (palm[1]) / 200
                    else:
                        if hand_grabbed:
                            hand_grabbed = False

                if hand_grabbed:
                    P3 = np.array([point3_x, 2 - point3_y])
                    print("P3:", P3)

                    
                    hlc1.go_to(P3[0]+0.5, 0, P3[1], 0, 0.5, relative=False)
                    hlc2.go_to(P3[0], 0, P3[1], 0, 0.5, relative=False)

                cv2.imshow('Hand Tracking', img)

                if fingers == [1, 1, 0, 0, 0]:
                    distance = math.sqrt(math.pow(lmList[8][0]-lmList[4][0], 2)+math.pow(lmList[8][1]-lmList[4][1], 2))
                    hlc1.go_to(P3[0]+0.5+distance, 0, P3[1], 0, 0.5, relative=False)
                    hlc2.go_to(P3[0]-distance, 0, P3[1], 0, 0.5, relative=False)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            pass

        finally:
            hlc1.land(0.0, 2.0)
            hlc2.land(0.0, 2.0)
            cap.release()
            cv2.destroyAllWindows()
