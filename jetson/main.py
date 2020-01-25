# CameraStream uses udp 1180-1190
# important files:
# camera.py
# socketManager.py

import absl
import socket
import cv2
import sys
import os
import time
import multiprocessing as mp

import camera

cameraStates

def frameManager(states):
    while True:
        

def main():
    camera.startCap()
    states = [False] * camera.getLen()
    global cameraStates = mp.Array('b', states)

    p = mp.Process(target=frameManager, args=(cameraStates,)).start()

if __name__ == '__main__':
    main()