import numpy as np 
import cv2 
import time

loss_factor = 16

videoFeed = cv2.VideoCapture(0)
while (True): 
    now = time.clock()
    ret, frame = videoFeed.read()
    cv2.imshow('qwe', frame)
    if ret == False:
        print("Failed to retrieve frame")
        break 
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR555)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR5552RGB)

    frame = cv2.resize(frame, (256, 144))
    frame = cv2.resize(frame, (1280, 720))

    cv2.imshow('feed', frame)
    if cv2.waitKey(10) & 0xFF == ord("q"): 
        break 
    diff = time.clock() - now
    print(diff)
videoFeed.release()
cv2.destroyAllWindows() 