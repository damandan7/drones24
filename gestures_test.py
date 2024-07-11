from cvzone.HandTrackingModule import HandDetector
import cv2
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.position_hl_commander import PositionHlCommander


cflib.crtp.init_drivers()
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')


# Setup and tracking stuff
cap = cv2.VideoCapture(0)
detector = HandDetector(detectionCon=0.8, maxHands=2)
hand_grabbed1 = False
hand_grabbed2 = False
peace = False
rock = False
call = False
thumb = False
pointer = False
middle = False
ring = False
pinky = False
palm = False

# Get the frame width to center the hand position
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
center_x = frame_width // 2


with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf) as pc:
    
        try:

            while True:
                success, img = cap.read()
                img = cv2.flip(img, 1)
                hands, img = detector.findHands(img)
                
                if hands:
                    hand1 = hands[0]
                    lmList1 = hand1['lmList']
                    fingers1 = detector.fingersUp(hand1)

            #Thumb
                    if fingers1 == [1,0,0,0,0]:
                        cv2.putText(img, f'Thumb', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)  
                        if not thumb:
                            thumb = True
                            pc.back(0.4, 0.3)
                    else:
                        thumb = False
            #Pointer
                    if fingers1 == [0,1,0,0,0]:
                        cv2.putText(img, f'Pointer', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        
                        if not pointer:
                            pointer = True
                            pc.forward(0.4, 0.3)
                        
                    else:
                        pointer = False
            #Middle
                    if fingers1 == [0,0,1,0,0]:
                        middle = True
                        cv2.putText(img, f'Middle', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    else:
                        if middle:
                            middle = False
            #Ring       
                    if fingers1 == [0,0,0,1,0]:
                        ring = True
                        cv2.putText(img, f'Ring', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    else:
                        if ring:
                            ring = False
            #Pinky        
                    if fingers1 == [0,0,0,0,1]:
                        cv2.putText(img, f'Pinky', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        if not pinky:
                            pinky = True
                            pc.up(0.4, 0.3)
                    else:
                        pinky = False
            #Peace
                    if fingers1 == [0,1,1,0,0]:
                        cv2.putText(img, f'Peace', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        if not peace:
                            peace = True
                            pc.down(0.4, 0.3)
                    else:
                        peace = False
            #Rock n roll       
                    if fingers1 == [0,1,0,0,1]:
                        rock = True
                        cv2.putText(img, f'Rock', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    else:
                        if rock:
                            rock = False
                            
            #Call
                    if fingers1 == [1,0,0,0,1]:
                        call = True
                        cv2.putText(img, f'Call', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    else:
                        if call:
                            call = False
            #Hand Grabbed 1
                    if fingers1 == [0, 0, 0, 0, 0]:  
                        hand_grabbed1 = True
                        cv2.putText(img, f'Hand Grabbed', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        
            #Palm
                    if fingers1 == [1, 1, 1, 1, 1]:
                        cv2.putText(img, f'Palm', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        if not palm:
                            palm = True
                            pc.land(0.4, 0.1)
                        else:
                            palm = False
                        

                #img Display
                cv2.imshow("Image", img)


                if cv2.waitKey(1) & 0xFF == ord('x'):
                    break

        finally:
            print('Landing.')
            cap.release()
            cv2.destroyAllWindows()
        
cap.release()
cv2.destroyAllWindows()
