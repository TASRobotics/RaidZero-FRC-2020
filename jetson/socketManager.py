import numpy as np
import cv2
import io
import socket
import struct
import time
import pickle
import zlib
import camera

HOST = '192.168.66.16'
#HOST = '127.0.0.1'
BASE_PORT = 5802
socks = [None] * 4
#cap = cv2.VideoCapture(0)



def openSocket():
    global socks
    for i in range(4):
        socks[i] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        socks[i].settimeout(60)

def sendData(id, out):
    global socks
    #ret, fm = cap.read() #camera.getFrame(id)
    #cv2.imshow('fr', fm)
    #cv2.waitKey(1)
    
    #frame = cv2.resize(fm, (256, 144))
    #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR555)
    #frame = np.reshape(frame, (12, -1))
    #out = [None] * 12
    #for i in range(12):
    #    out[i] = bytes(str(i), 'utf8') + frame[i].tostring()

    for i in range(12):
        socks[id].sendto( out[i],(HOST, BASE_PORT + id) )

if __name__ == '__main__':
    openSocket()
    while True:
        sendData(0)
    cap.release()
    cv2.destroyAllWindows()

#client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#client_socket.connect(('192.168.66.12', 5802))
##client_socket.connect(('10.42.53.140', 5802))
#connection = client_socket.makefile('wb')
#
#cam = cv2.VideoCapture(0)
#
#cam.set(3, 320);
#cam.set(4, 240);
#
#img_counter = 0
#
#encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
#
#while True:
#    ret, frame = cam.read()
#    result, frame = cv2.imencode('.jpg', frame, encode_param)
##    data = zlib.compress(pickle.dumps(frame, 0))
#    data = pickle.dumps(frame, 0)
#    size = len(data)
#
#
#    print("{}: {}".format(img_counter, size))
#    client_socket.sendall(struct.pack(">L", size) + data)
#    img_counter += 1
#
#cam.release()