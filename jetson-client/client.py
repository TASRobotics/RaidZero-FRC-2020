import socket
import numpy as np
import time
import threading
import cv2

CAMS = 4
PACKETS = 12
SIZE = int((640 * 360 * 3) / PACKETS)

#UDP_IP="192.168.66.16"
UDP_IP="127.0.0.1"
UDP_PORT = 5802
data = [[b""] * PACKETS] * CAMS
sock = [None] * CAMS
tt=0

def read(cam):
    global data
    global frame
    global UDP_IP
    global UDP_PORT
    global tt
    global sock
    sock[cam] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock[cam].bind((UDP_IP, UDP_PORT + cam))

    while True:
        rec, addr = sock[cam].recvfrom(SIZE+1)
        data[cam][rec[0]] = rec
        #print(rec[0])
        #tt = time.time() - tt
        #tt = tt * 1000
        #print("\n\nprocess time")
        #print(tt)
        #tt = time.time()


def display():
    global data
    global CAMS
    frame = [b""] * CAMS
    while True:
        for cam in range(CAMS):
            frame[cam] = b""
            for packet in range(PACKETS):
                frame[cam] += data[cam][packet][1:]
            try:
                frame[cam] = np.fromstring (frame[cam], dtype=np.uint8)
                frame[cam] = np.reshape(frame[cam], (360, 640, 3))
            except:
                continue
            #cv2.imshow("camera "+str(cam), frame[cam])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                return

if __name__ == "__main__":
    reading0 = threading.Thread(target=read, args=(0,)).start()
    #reading1 = threading.Thread(target=read, args=(1,)).start()
    #reading2 = threading.Thread(target=read, args=(2,)).start()
    #reading3 = threading.Thread(target=read, args=(3,)).start()
    disp = threading.Thread(target=display, args=()).start()

#      data, addr = sock.recvfrom(CAMS6080)
#      s+= data
#      if len(s) == (CAMS6080*20):
#          frame = numpy.fromstring (s, dtype=numpy.uint8)
#          frame = frame.reshape(CAMS80,6CAMS0,3)
#          cv2.imshow("frame",frame)
#
#          s=""
#      if cv2.waitKey(1) & 0xFF == ord('q'):
#          break