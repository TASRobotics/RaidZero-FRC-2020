# CameraStream uses udp 1180-1190
# important files:
# cams.py
# socketManager.py

import absl
import socket
import cv2
import sys
import os
import time
import numpy as np
import threading as thred

import camera
import socketManager

PACKETS = 8
SIZE = int((480 * 270 * 3) / PACKETS)
camFrames = [[None] * PACKETS] * 4
camsFrameReady = []
tt = 0

def frameManager(cams):
    global camsFrameReady
    global camFrames
    while True:
        for cam in range(len(camsFrameReady)):
            cams.capFrame(cam)
            frame = cams.getFrame(cam).tobytes()
            for pack in range(PACKETS):
                camFrames[cam][pack] = bytes(chr(pack), "utf8") + frame[pack*SIZE:(pack+1)*SIZE]
                #camFrames[cam][pack] = bytes(chr(cam), "utf8") + frame[pack*SIZE:(pack+1)*SIZE]
            camsFrameReady[cam] = True
            print('switched read')
            #frame = np.fromstring (frame, dtype=np.uint8)
            #frame = np.reshape(frame, (270, 480, 3))
            #cv2.imshow(str(cam), frame)
            #cv2.waitKey(1)
            #cv2.imwrite('frame'+str(cam), cams.getFrame(cam))
            

def commManager(cam):
    global camsFrameReady
    global tt
    global camFrames
    global PACKETS
    #comms start
    while True:
        #for cam in range(len(camsFrameReady)):
        if True:
            if camsFrameReady[cam]: 
                frame = b""
                for packet in range(PACKETS):
                    print(cam, " ", packet)
                    frame += camFrames[cam][packet][1:]
                    #print(packet)
                    #print(len(out))
                    socketManager.sendData(cam, camFrames[cam][packet])
                    #time.sleep(1)
                    #tt = time.time() - tt
                    #tt = tt * 1000
                    #print("\n\nprocess time")
                    #print(tt)
                    #tt = time.time()
                camsFrameReady[cam] = False
                frame = np.fromstring (frame, dtype=np.uint8)
                frame = np.reshape(frame, (270, 480, 3))
                cv2.imshow(str(cam), frame)
                cv2.waitKey(1)

def main():
    #cams start
    global camFrames
    global camsFrameReady
    cams = camera.cameraSet()
    cams.startCap()
    camsFrameReady = [False] * cams.getLen()
    socketManager.openSocket()

    cams = thred.Thread(target=frameManager, args=(cams, )).start()
    comms = thred.Thread(target=commManager, args=(0,)).start()
    comms = thred.Thread(target=commManager, args=(1,)).start()
    #nn = thred.Thread(target=commManager, args=(camsFrameReady, camFrames, )).start()

if __name__ == '__main__':
    main()