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
import multiprocessing as mp
import numpy as np

import camera
import socketManager

def frameManager(camsFrameSet, frames):
    while True:
        for cam in range(len(camsFrameSet)):
            if camsFrameSet[cam] == False:
                camsFrameSet[cam] = True
                cams.capFrame(cam)
                frame = cams.getFrame(cam)
                
                out = [None] * 12
                for i in range(12):
                    out[i] = bytes(str(i), 'utf8') + frame[i].tostring()
                frames[cam] = out
            #cv2.imwrite('frame'+str(cam), cams.getFrame(cam))

def commManager(camsFrameSet, frames):
    #comms start
    socketManager.openSocket()
    while True:
        for cam in range(len(camsFrameSet)):
            socketManager.sendData(cam, frames[cam])
            camsFrameSet[cam] = False


def main():
    #cams start
    cams = camera.cameraSet()
    cams.startCap()
    print("camss ")
    print(cams.getLen())
    states = [False] * cams.getLen()
    camsFrameSet = mp.Array('b', states)
    
    camFrames = mp.Array('i', np.array([[None] * 12] * 4))

    cams = mp.Process(target=frameManager, args=(camsFrameSet, camFrames, )).start()
    comms = mp.Process(target=commManager, args=(camsFrameSet, camFrames, )).start()

if __name__ == '__main__':
    main()