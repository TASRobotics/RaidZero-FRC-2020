# CameraStream uses udp 1180-1190

import absl
import socket
import cv2
import sys

sys.path.append('CLib/build')
#Cython files
import hello

UDP_IP = ""
UDP_PORT =''

def main():
    hello.hi()

if __name__ == '__main__':
    main()