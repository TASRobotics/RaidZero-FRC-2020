import numpy as np
import cv2
import io
import socket
import struct
import time
import pickle
import zlib
import camera

#HOST = '192.168.66.16'
HOST = '127.0.0.1'
BASE_PORT = 5802
socks = [None] * 4
#cap = cv2.VideoCapture(0)

PACKETS = 8
SIZE = int((480 * 270 * 3) / PACKETS)

def openSocket():
    global socks
    socks = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def sendData(id, out):
    global socks
    print(BASE_PORT+id)
    socks.sendto( out,(HOST, BASE_PORT + id))

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