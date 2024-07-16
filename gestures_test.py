from cvzone.HandTrackingModule import HandDetector
import cv2
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.position_hl_commander import PositionHlCommander


cflib.crtp.init_drivers()
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')


# Setup and tracking stuff
cap = cv2.VideoCapture(0)
detector = HandDetector(detectionCon=0.90, maxHands=2)
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

lock = False

# Get the frame width and height to center the hand position
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
center_x = frame_width // 2

frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) 
center_y = frame_height // 2


with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf) as pc:
    
        try:

            while True:
                success, img = cap.read()
                img = cv2.flip(img, 1)
                hands, img = detector.findHands(img)
                pc.set_default_height(1)
                
                
                if hands:
                    hand1 = hands[0]
                    lmList1 = hand1['lmList']
                    fingers1 = detector.fingersUp(hand1)
                    wrist = lmList1[0] 

                #Hand postions
                    hand_positionx = (wrist[0] - center_x) / 100    
                    hand_positiony = -(wrist[1] - center_y) / 100   

                    if fingers1 == [0,1,0,0,1]:
                        rock = True
                    if rock == True:
                        cv2.putText(img, f'LOCK', (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 
                        if fingers1 == [1,1,0,0,0]:
                            cv2.putText(img, f'LOCK', (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 
                            cv2.putText(img, f'Half Pinch', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 
                            cv2.putText(img, f'Hand Position X: {hand_positionx:.2f}', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)   #&&&&&&&&&&
                            cv2.putText(img, f'Hand Position Y: {hand_positiony+1:.2f}', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)   #&&&&&&&&&&
                            pc.go_to(hand_positionx, 0, hand_positiony+1)
                            if not halfpinch:
                                halfpinch = True
                        else:
                            halfpinch = False
                        if fingers1 == [0,1,1,0,0]:
                            cv2.putText(img, f'Peace', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                            pc.land()
                            
    #####################  
                    else:
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
                                pc.land()
                        else:
                            peace = False
                #Rock n roll       
                        if fingers1 == [0,1,0,0,1]:
                            rock = True
                            cv2.putText(img, f'Rock', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                            lock = True
                        else:
                            if rock:
                                rock = False
                                
                #Call
                        if fingers1 == [1,0,0,0,1]:
                            call = True
                            cv2.putText(img, f'Call', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                            lock = False
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
