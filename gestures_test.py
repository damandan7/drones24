from cvzone.HandTrackingModule import HandDetector
import cv2
import math

# Set and tracking stuff
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

# Get the frame width to center the hand position
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
center_x = frame_width // 2

#Running and Gesture Recognition
while True:
    
    success, img = cap.read()
    img = cv2.flip(img, 1)
    hands, img = detector.findHands(img)
    
    if hands:
        hand1 = hands[0]
        lmList1 = hand1['lmList']
        fingers1 = detector.fingersUp(hand1)

        if len(hands) == 2:
                cv2.putText(img, f'Hand Grabbed', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                hand2 = hands[1]
                lmList2 = hand2["lmList"]
                fingers2 = detector.fingersUp(hand2)

            #Hand Grabbed 2
                if fingers2 == [0,0,0,0,0]:
                    hand_grabbed2 = True
                    
                else:
                    if hand_grabbed2:
                        hand_grabbed2 = False
                

#Thumb
        if fingers1 == [1,0,0,0,0]:
            thumb = True
            cv2.putText(img, f'Thumb', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        else:
            if thumb:
                thumb = False
#Pointer
        if fingers1 == [0,1,0,0,0]:
            pointer = True
            cv2.putText(img, f'Pointer', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        else:
            if pointer:
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
            pinky = True
            cv2.putText(img, f'Pinky', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        else:
            if pinky:
                pinky = False
#Peace
        if fingers1 == [0,1,1,0,0]:
            peace = True
            cv2.putText(img, f'Peace', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        else:
            if peace:
                peace = False
#Rock        
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
                
        else:
            if hand_grabbed1:
                hand_grabbed1 = False


    #img Display
    cv2.imshow("Image", img)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()