# DEBUG ONLY


import socket
import numpy
import time
import cv2

UDP_IP="127.0.0.1"
UDP_PORT = 5802
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

SPLIT = 8
SIZE = int((480 * 270 * 3) / SPLIT)

s=b""

while True:
    data, addr = sock.recvfrom(SIZE+1)
    s+= data[1:]
    if len(s) == (SIZE*SPLIT):
        frame = numpy.fromstring (s, dtype=numpy.uint8)
        frame = frame.reshape(270,480,3)
        cv2.imshow("frame",frame)
        #cv2.waitKey(1)
        s=b""
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break