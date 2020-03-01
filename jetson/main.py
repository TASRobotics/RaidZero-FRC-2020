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
camFrames = []
camsFrameReady = [] # 0 = paused; 1 = reading; 2 = sending becuz read
tt = 0
comms = 1

def frameProcessor(cams):
    global comms
    while not comms==1:
        time.sleep(0.1)
    processFrames(cams)
    comms = 2
    while True:
        processFrames(cams)

def processFrames(cams):
    global camsFrameReady
    global camFrames
    for camN in range(len(camsFrameReady)):
        cams.capFrame(camN)
        if camsFrameReady[camN] == 1:
            frame = cams.getFrame(camN).tobytes()
            for pack in range(PACKETS):
                #print("cams: ", camN, " ", pack)
                camFrames[pack] = bytes(chr(pack), "utf8") + frame[pack*SIZE:(pack+1)*SIZE]
                #print("a ", camN, " ", pack)
                #camFrames[pack] = bytes(chr(cam), "utf8") + frame[pack*SIZE:(pack+1)*SIZE]
                #time.sleep(0.01)
            camsFrameReady[camN] = 2
            #frame = np.fromstring (frame, dtype=np.uint8)
            #frame = np.reshape(frame, (270, 480, 3))
            #cv2.imshow(str(cam), frame)
            #cv2.waitKey(1)
            #cv2.imwrite('frame'+str(cam), cams.getFrame(cam))
       
def commManager():
    global camsFrameReady
    #global tt
    global camFrames
    global PACKETS
    global comms
    #comms start
    while not comms==2:
        time.sleep(0.1)
    while True:
        for cam in range(len(camsFrameReady)):
            if camsFrameReady[cam] == 2:
                
                #frame = b""
                
                for packet in range(PACKETS):
                    #print("comms: ", cam, " ", packet)
                    #print(cam, " ", packet)
                
                    #frame += camFrames[packet][1:]
                
                    #print(packet)
                    #print(len(out))
                    #print(camFrames[packet][0])
                    socketManager.sendData(cam, camFrames[packet])
                    #time.sleep(1)
                    #tt = time.time() - tt
                    #tt = tt * 1000
                    #print("\n\nprocess time")
                    #print(tt)
                    #tt = time.time()
                camsFrameReady[cam] = 0
                
                if cam != len(camsFrameReady) - 1:
                    camsFrameReady[cam + 1] = 1
                else:
                    camsFrameReady[0] = 1
                #frame = np.fromstring (frame, dtype=np.uint8)
                #frame = np.reshape(frame, (270, 480, 3))
                #cv2.imshow(str(cam), frame)
                #cv2.waitKey(1)
                
                #time.sleep(0.2)

def main():
    #cams start
    global camFrames
    global camsFrameReady
    cams = camera.cameraSet()
    camsFrameReady = [0] * cams.getLen()
    camsFrameReady[0] = 1
    socketManager.openSocket()
    camFrames = [None] * 12

    image_processing = thred.Thread(target=frameProcessor, args=(cams,)).start()
    comms = thred.Thread(target=commManager, args=()).start()
    #nn = thred.Thread(target=commManager, args=(camsFrameReady, camFrames, )).start()

if __name__ == '__main__':
    main()