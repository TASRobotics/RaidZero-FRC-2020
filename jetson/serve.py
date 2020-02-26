# DEBUG ONLY


import socket
import numpy
import time
import cv2

UDP_IP="127.0.0.1"
UDP_PORT = 5802
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

SPLIT = 16
SIZE = int( ( 144 * 256 * 3 )/ SPLIT)

s=b""

while True:
    data, addr = sock.recvfrom(SIZE)
    s+= data
    print(data[0:20])
    if len(s) == (SIZE*SPLIT):
        print("hello")
        frame = numpy.fromstring (s, dtype=numpy.uint8)
        frame = frame.reshape(144,256,3)
        cv2.imshow("frame",frame)
        #cv2.waitKey(1)
        s=b""
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break