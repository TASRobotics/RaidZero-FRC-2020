import socket
import numpy as np
import time
import cv2

DATA_SIZE = 640 * 144 * 3

#UDP_IP="192.168.66.16"
UDP_IP="127.0.0.1"
UDP_PORT = 5802
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

dat = [[], [], [], [], [], [], [], [], [], [], [], []]

while True:
    data, addr = sock.recvfrom(DATA_SIZE)
    data = np.fromstring(data, dtype=np.uint8)
    data = np.array(data)

    if data[0] == ord("0"):
        data = np.delete(data, 0)
        dat[0] = data
    elif (data[0] == ord("1")) & (data[1] == ord("0")):
        data = np.delete(data, 0)
        data = np.delete(data, 0)
        dat[10] = data
    elif data[0] == ord("1") & data[1] == ord("1"):
        data = np.delete(data, 0)
        data = np.delete(data, 0)
        dat[11] = data
        boo = 0
        for array in dat:
            try:
                array[0] 
                boo += 1
            except IndexError:
                break
        
        if boo == 12:
            frame = np.array(dat)
            frame = frame.reshape(144, 256, 2)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR5552RGB)
            frame = cv2.resize(frame, (1280, 720))

            cv2.imshow("frame",frame)
            cv2.waitKey(1)
    else:
        a = chr(data[0])
        a = int(a)
        data = np.delete(data, 0)
        dat[a] = data


#      data, addr = sock.recvfrom(46080)
#      s+= data
#      if len(s) == (46080*20):
#          frame = numpy.fromstring (s, dtype=numpy.uint8)
#          frame = frame.reshape(480,640,3)
#          cv2.imshow("frame",frame)
#
#          s=""
#      if cv2.waitKey(1) & 0xFF == ord('q'):
#          break