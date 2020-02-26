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

PACKETS = 12
SIZE = int((640 * 360 * 3) / PACKETS)
camFrames = [[None] * PACKETS] * 4
camsFrameReady = []
tt = 0

def frameManager(camsFrameReady, frames, cams):
    while True:
        for cam in range(len(camsFrameReady)):
            global tt
            if camsFrameReady[cam] == False:
                cams.capFrame(cam)
                frame = cams.getFrame(cam).tobytes()
                for pack in range(PACKETS):
                    frames[cam][pack] = bytes(chr(pack), "utf8") + frame[pack*SIZE:(pack+1)*SIZE]
                camsFrameReady[cam] = True
            #cv2.imwrite('frame'+str(cam), cams.getFrame(cam))
            

def commManager(camsFrameReady, frames):
    global tt
    #comms start
    while True:
        for cam in range(len(camsFrameReady)):
            if camsFrameReady[cam]: 
                for packet in range(PACKETS):
                    #print(packet)
                    #print(len(out))
                    socketManager.sendData(cam, frames[cam][packet])
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

    cams = thred.Thread(target=frameManager, args=(camsFrameReady, camFrames, cams, )).start()
    comms = thred.Thread(target=commManager, args=(camsFrameReady, camFrames, )).start()
    #nn = thred.Thread(target=commManager, args=(camsFrameReady, camFrames, )).start()

if __name__ == '__main__':
    main()