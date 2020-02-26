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

def frameManager(camsFrameReady, cams):
    global camFrames
    while True:
        for cam in range(len(camsFrameReady)):
            global tt
            if camsFrameReady[cam] == False:
                cams.capFrame(cam)
                cv2.imshow(str(cam), cams.getFrame(cam))
                frame = cams.getFrame(cam).tobytes()
                for pack in range(PACKETS):
                    camFrames[cam][pack] = bytes(chr(pack), "utf8") + frame[pack*SIZE:(pack+1)*SIZE]
                camsFrameReady[cam] = True
            #cv2.imwrite('frame'+str(cam), cams.getFrame(cam))
            

def commManager(camsFrameReady,):
    global tt
    global camFrames
    #comms start
    while True:
        for cam in range(len(camsFrameReady)):
            if camsFrameReady[cam]: 
                for packet in range(PACKETS):
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

def main():
    #cams start
    global camFrames
    global camsFrameReady
    cams = camera.cameraSet()
    cams.startCap()
    camsFrameReady = [False] * cams.getLen()
    socketManager.openSocket()

    cams = thred.Thread(target=frameManager, args=(camsFrameReady, cams, )).start()
    comms = thred.Thread(target=commManager, args=(camsFrameReady, )).start()
    #nn = thred.Thread(target=commManager, args=(camsFrameReady, camFrames, )).start()

if __name__ == '__main__':
    main()