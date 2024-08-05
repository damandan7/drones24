import logging
import time
import cv2
import math
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

    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf1, SyncCrazyflie(URI4, cf=Crazyflie(rw_cache='./cache')) as scf4, SyncCrazyflie(URI6, cf=Crazyflie(rw_cache='./cache')) as scf6, SyncCrazyflie(URI7, cf=Crazyflie(rw_cache='./cache')) as scf7: 
        print('connecttehted')
        reset_estimator(scf1)
        print('connectd')
        #reset_estimator(scf2)
        #print('connectd')
        #reset_estimator(scf3)
        #print('connectd')
        reset_estimator(scf4)
        print('connectd')
        #reset_estimator(scf5)
        #print('connectd')
        reset_estimator(scf6)
        print('connectd')
        reset_estimator(scf7)
        print('connectd')
        #reset_estimator(scf8)
        #print('connectd')
        #reset_estimator(scf9)
        #print('connectd')
        
        hlc1 = scf1.cf.high_level_commander
        #hlc2 = scf2.cf.high_level_commander
        #hlc3 = scf3.cf.high_level_commander
        hlc4 = scf4.cf.high_level_commander
        #hlc5 = scf5.cf.high_level_commander
        hlc6 = scf6.cf.high_level_commander
        hlc7 = scf7.cf.high_level_commander
        #hlc8 = scf8.cf.high_level_commander
        #hlc9 = scf9.cf.high_level_commander




        
        hlc1.go_to(.5, 0, 1, 0, 2, relative=False)
        #hlc2.go_to(1, 0, 1, 0, 2, relative=False)
        #hlc3.go_to(1, -1, 1, 0, 2, relative=False)
        hlc4.go_to(-.5, 0, 1, 0, 2, relative=False)
        #hlc5.go_to(-1, -1, 1, 0, 2, relative=False)
        hlc6.go_to(0, .5, 1, 0, 2, relative=False)
        hlc7.go_to(0, -.5, 1, 0, 2, relative=False)
        #hlc8.go_to(0, 1, 1, 0, 2, relative=False)
        #hlc9.go_to(1, 1, 1, 0, 2, relative=False)
        
        time.sleep(2) 
         

        try:
            
            while cap.isOpened():
                success, img = cap.read()
                if not success:
                    break

                img = cv2.flip(img, 1)
                hands, img = detector.findHands(img)

                hand1 = hands[0]
                

                if hand1:
                    
                    lmList1 = hand1['lmList']
                    
                    fingers1 = detector.fingersUp(hand1)
                    
                    palm1 = lmList1[9]
                    point3_x = (palm1[0] - center_x) / 150
                    point3_y = (palm1[1]) / 200
                    P3 = np.array([point3_x, 2 - point3_y])
                    depth = (lmList1[18][0]-lmList1[6][0])/25-2

                    distance = math.sqrt(math.pow(lmList1[8][0]-lmList1[4][0], 2)+math.pow(lmList1[8][1]-lmList1[4][1], 2))
                    distance = distance/100
                    distancex = distance*math.cos(45)
                    distancey= distance*math.sin(45)

                    if fingers1 == [0, 0, 0, 0, 0]:
                        hand_grabbed = True
                        
                    else:
                        if hand_grabbed:
                            hand_grabbed = False



                    if hand_grabbed:
                        
                        print("P3:", P3)
                        hlc1.go_to(P3[0]+.5, depth, P3[1], 0, 1, relative=False)
                        #hlc2.go_to(P3[0]+1, depth, P3[1], 0, 0.5, relative=False)
                        #hlc3.go_to(P3[0]+1, depth-1, P3[1], 0, 0.5, relative=False)
                        hlc4.go_to(P3[0]-.5, depth, P3[1], 0, 1, relative=False)
                        #hlc5.go_to(P3[0]-1, depth-1, P3[1], 0, 0.5, relative=False)
                        hlc6.go_to(P3[0], depth+.5, P3[1], 0, 1, relative=False)
                        hlc7.go_to(P3[0], depth-.5, P3[1], 0, 1, relative=False)
                        #hlc8.go_to(P3[0], depth+1, P3[1], 0, 0.5, relative=False)
                        #hlc9.go_to(P3[1]+1, depth+1, P3[1], 0, 0.5, relative=False)

                        P3FinalX=P3[0]
                        P3FinalY=P3[1]
                        depthFinal=depth
                        

                    if fingers1 == [1, 1, 0, 0, 0]:
                        distance = math.sqrt(math.pow(lmList1[8][0]-lmList1[4][0], 2)+math.pow(lmList1[8][1]-lmList1[4][1], 2))
                        distance = distance//10/10-.2
                        distancex = distance*math.cos(45)
                        distancey= distance*math.sin(45)
                        cv2.putText(img, f'Spreading: {distance:.2f}', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        hlc1.go_to(P3FinalX+.25+distance, depthFinal, P3FinalY, 0, 1, relative=False)
                        #hlc2.go_to(P3FinalX+distance+1, depthFinal, P3FinalY, 0, 1, relative=False)   
                        #hlc3.go_to(P3FinalX+distance+1, depthFinal-distance-1, P3FinalY, 0, 1, relative=False)
                        hlc4.go_to(P3FinalX-.25-distance, depthFinal, P3FinalY, 0, 1, relative=False)
                        #hlc5.go_to(P3FinalX-distance-1, depthFinal-distance-1, P3FinalY, 0, 1, relative=False)   
                        hlc6.go_to(P3FinalX, depthFinal+.25+distance, P3FinalY, 0, 1, relative=False) 
                        hlc7.go_to(P3FinalX, depthFinal-.25-distance, P3FinalY, 0, 1, relative=False)
                        #hlc8.go_to(P3FinalX, depthFinal+distance+1, P3FinalY, 0, 1, relative=False)   
                        #hlc9.go_to(P3FinalX+distance+1, depthFinal+distance+1, P3FinalY, 0, 1, relative=False)    

                    if fingers1 == [0,0,0,0,1]:
                        P3 = np.array([point3_x, 2 - point3_y])
                        print("P3:", P3)
                        hlc1.go_to(P3[0], depth, P3[1], 0, 0.5, relative=False)
                    if fingers1 == [0,0,0,1,1]:                       
                        P3 = np.array([point3_x, 2 - point3_y])
                        print("P3:", P3)
                        hlc4.go_to(P3[0], depth, P3[1], 0, 0.5, relative=False)
                    if fingers1 == [0,0,1,1,1]:
                        P3 = np.array([point3_x, 2 - point3_y])
                        print("P3:", P3)
                        hlc6.go_to(P3[0], depth, P3[1], 0, 0.5, relative=False)
                    if fingers1 == [0,1,1,1,1]:
                        P3 = np.array([point3_x, 2 - point3_y])
                        print("P3:", P3)
                        hlc7.go_to(P3[0], depth, P3[1], 0, 0.5, relative=False)


                    cv2.imshow('Hand Tracking', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            pass

        finally:
            hlc1.land(0,2)
            #hlc2.land(0,2)
            #hlc3.land(0,2)
            hlc4.land(0,2)
            #hlc5.land(0,2)
            hlc6.land(0,2)
            hlc7.land(0,2)
            #hlc8.land(0,2)
            #hlc9.land(0,2)
            
            cap.release()
            cv2.destroyAllWindows()
if __name__ == "__main__":
    main()
