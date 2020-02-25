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

camFrames = [None] * 12
camsFrameSet = []
tt = 0
    
def frameManager(camsFrameSet, frames, cams):
    while True:
        for cam in range(len(camsFrameSet)):
            global tt
            if camsFrameSet[cam] == False:
                cams.capFrame(cam)
                frame = cams.getFrame(cam)
                out = [None] * 12
                for i in range(12):
                    out[i] = bytes(str(i), 'utf8') + frame[i].tostring()
                frames[cam] = out
                camsFrameSet[cam] = True
            #cv2.imwrite('frame'+str(cam), cams.getFrame(cam))
            

def commManager(camsFrameSet, frames):
    global tt
    #comms start
    socketManager.openSocket()
    while True:
        for cam in range(len(camsFrameSet)):
            if camsFrameSet[cam]:
                socketManager.sendData(cam, frames[cam])
                camsFrameSet[cam] = False
                #tt = time.time() - tt
                #tt = tt * 1000
                #print("\n\nprocess time")
                #print(tt)
                #
                #tt = time.time()
 
def main():
    #cams start
    global camFrames
    global camsFrameSet
    cams = camera.cameraSet()
    cams.startCap()
    camsFrameSet = [False] * cams.getLen()

    cams = thred.Thread(target=frameManager, args=(camsFrameSet, camFrames, cams, )).start()
    comms = thred.Thread(target=commManager, args=(camsFrameSet, camFrames, )).start()
    #nn = thred.Thread(target=commManager, args=(camsFrameSet, camFrames, )).start()

if __name__ == '__main__':
    main()