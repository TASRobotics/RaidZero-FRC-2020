import numpy as np
import cv2

camArray = []
frameArray = []
loss_factor = 16

def capFrame(id):
    ret, frame = camArray[id].read()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR555)
    frame = cv2.resize(frame, (256, 144))
    frameArray[id] = frame

def getFrame(id):
    return frameArray[id]

def decCam(id):
    return cv2.VideoCapture(id)

def getLen():
    return len(camArray)

def startCap():
    global camArray
    global frameArray

    for i in range(4):
        cap = decCam(int(i))
        if cap is None or not cap.isOpened():
            print('camera ', str(i), ' does not exist')
        else:
            print('declared')
            camArray.append(cap)
    print('Cameras added:',len(camArray))
    frameArray = [None] * len(camArray)

if __name__ == '__main__':
    startCap()
    print('started cap')
    while True:
        for id in range(len(camArray)):
            capFrame(id)
            cv2.imshow('id:'+ str(id),getFrame(id))
        if cv2.waitKey(10) & 0xFF == ord("q"): 
            break
    for cam in camArray:
        cam.release()
    cv2.destroyAllWindows()