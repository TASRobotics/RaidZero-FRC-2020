# DEBUG ONLY


import socket
import numpy as np
import cv2
UDP_IP = '127.0.0.1'                  
UDP_PORT = 5802      

SIZE = int( ( 144 * 256 * 3 ) / 16)

cap = cv2.VideoCapture(0)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while(True):
    ret, frame = cap.read()
    #cv2.imshow('frame',frame)
    frame = cv2.resize(frame, (256, 144))
    s = frame.tobytes ()
    for i in range(16):
        sock.sendto (s[i*SIZE:(i+1)*SIZE],(UDP_IP, UDP_PORT))
        if cv2.waitKey(1) & 0xFF == ord('q'):
           break

cap.release()
cv2.destroyAllWindows()